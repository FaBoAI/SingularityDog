"""CPU-only bitwise equivalence of a batched projection scan for the 30 Hz controller.

The frozen controller evaluates sixteen projection candidates with sixteen
sequential ``feasible_ik`` calls. ``feasible_ik`` is elementwise over its
leading dimension, so this experiment evaluates the identical candidate
expressions in one batch and keeps the ordered acceptance loop unchanged.

The default PLAN reads nothing. ``--execute-cpu`` reads the pinned private
bundle source and controller, builds the candidate in memory and compares it
with the frozen controller tick by tick: observation, actor output, every
``step`` result entry and every controller buffer must be bitwise equal.
No device is opened and the candidate is not a deployable model release.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time

SWING_CORE_SHA256 = '09e6a51e665d382739d48979bd2d54ec9bd5d1c072e64bca1a9079d09dd62542'
CONTROLLER_SHA256 = '37161d7ff58cfca626b4a3a35ff5f84c294d2478584309b436726f3a0768f682'
OPTIONS = dict(use_plane=True, period=.56, duty=.60, swing_height=.035, residual_tau=0., stance_widen_m=.02,
               heading_gain=.8, forward_command_limit=.46, hip_residual_scale=.27)
SEQUENTIAL = """        for k in range(1,self.projection_scan_intervals+1):
            fraction=float(k)/float(self.projection_scan_intervals)
            candidate=start_feet+(requested*fraction).unsqueeze(2)*self.sensor_up.unsqueeze(1)
            q,ok,_domain=self.feasible_ik(candidate)
"""
BATCHED = """        # Batched scan: every candidate is the identical expression evaluated
        # per fraction; feasible_ik is elementwise over its leading dimension.
        fractions=[float(k)/float(self.projection_scan_intervals) for k in range(1,self.projection_scan_intervals+1)]
        candidates=torch.stack([start_feet+(requested*f).unsqueeze(2)*self.sensor_up.unsqueeze(1) for f in fractions],0)
        batch_q,batch_ok,_batch_domain=self.feasible_ik(candidates.reshape(len(fractions)*self.num_envs,4,3))
        batch_q=batch_q.reshape(len(fractions),self.num_envs,4,3);batch_ok=batch_ok.reshape(len(fractions),self.num_envs,4)
        for k in range(1,self.projection_scan_intervals+1):
            fraction=fractions[k-1]
            q=batch_q[k-1];ok=batch_ok[k-1]
"""
INPUT_NAMES = ('gyro_body_rad_s', 'gravity_body_unit', 'command', 'q_model_rad', 'dq_model_rad_s', 'h_hypothesis12')
COMMANDS = ((.46, 0., 0.), (0., 0., 0.), (-.12, 0., 0.), (0., .12, 0.), (0., -.12, 0.), (0., 0., .25), (0., 0., -.25))


def _pinned(path, sha):
    raw = Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest() != sha:
        raise ValueError('Pinned file differs: '+str(path))
    return raw


def batched_source(raw):
    text = raw.decode()
    if text.count(SEQUENTIAL) != 1:
        raise ValueError('Frozen sequential projection scan not found exactly once')
    return text.replace(SEQUENTIAL, BATCHED)


def build(bundle, workdir):
    import torch
    from torch import nn, Tensor
    source = Path(workdir)/'swing_core.py'
    source.write_text(batched_source(_pinned(Path(bundle)/'source/swing_core.py', SWING_CORE_SHA256)))
    spec = importlib.util.spec_from_file_location('swing_core', source)
    module = importlib.util.module_from_spec(spec)
    sys.modules['swing_core'] = module
    spec.loader.exec_module(module)
    controller_path = Path(bundle)/'model/policy_controller74_30hz.pt'
    _pinned(controller_path, CONTROLLER_SHA256)
    reference = torch.jit.load(str(controller_path), map_location='cpu').eval()

    class DeployablePolicy30(nn.Module):  # Same wrapper as the frozen export.
        __constants__ = ['policy_dt_s']

        def __init__(self, actor, options):
            super().__init__()
            self.actor = actor
            self.controller = module.SwingCore(1, 'cpu', **options)
            self.policy_dt_s = 1./30.
            self.register_buffer('last_observation', torch.zeros(1, 74))
            self.register_buffer('last_actor_output', torch.zeros(1, 12))

        @torch.jit.export
        def reset(self, ids: Tensor) -> None:
            self.controller.reset(ids)
            self.last_observation[ids] = 0.
            self.last_actor_output[ids] = 0.

        def forward(self, gyro: Tensor, gravity: Tensor, command: Tensor, q: Tensor, dq: Tensor, h: Tensor) -> Tensor:
            obs = self.controller.observation(gyro, gravity, command, q, dq, h)
            self.last_observation.copy_(obs)
            raw = self.actor(obs)
            self.last_actor_output.copy_(raw)
            return self.controller.step(raw, command)['q_target_rad']

    candidate = DeployablePolicy30(torch.jit.load(str(controller_path), map_location='cpu').actor, OPTIONS)
    for name in module.SwingCore.__constants__:
        if getattr(reference.controller, name) != getattr(candidate.controller, name):
            raise ValueError('Controller constant differs: '+name)
    ref_buffers, new_buffers = dict(reference.controller.named_buffers()), dict(candidate.controller.named_buffers())
    if ref_buffers.keys() != new_buffers.keys() or any(
            ref_buffers[k].dtype != new_buffers[k].dtype or not torch.equal(ref_buffers[k], new_buffers[k])
            for k in ref_buffers):
        raise ValueError('Controller buffers differ before comparison')
    return reference, torch.jit.script(candidate).eval()


def _same(a, b):
    import torch
    return a.dtype == b.dtype and a.shape == b.shape and bool(((a == b) | (torch.isnan(a) & torch.isnan(b))).all())


def _tick(model, args):
    controller = model.controller
    obs = controller.observation(*args)
    raw = model.actor(obs)
    begin = time.perf_counter_ns()
    result = controller.step(raw, args[2])
    return obs, raw, result, time.perf_counter_ns()-begin


def _compare(reference, candidate, args, stats):
    try:
        left = _tick(reference, args)
        left_error = None
    except Exception as error:  # Both models must reject identically.
        left, left_error = None, str(error).strip().splitlines()[-1]
    try:
        right = _tick(candidate, args)
        right_error = None
    except Exception as error:
        right, right_error = None, str(error).strip().splitlines()[-1]
    stats['ticks'] += 1
    if left_error or right_error:
        stats['exceptions_matched' if left_error == right_error else 'exception_mismatch'] += 1
        return False
    if left[2].keys() != right[2].keys():
        raise ValueError('Controller result keys differ')
    pairs = [(left[0], right[0]), (left[1], right[1])] + [(left[2][k], right[2][k]) for k in left[2]]
    rb, cb = dict(reference.controller.named_buffers()), dict(candidate.controller.named_buffers())
    pairs += [(rb[k], cb[k]) for k in rb]
    stats['compared_tensors'] += len(pairs)
    stats['mismatches'] += sum(not _same(a, b) for a, b in pairs)
    stats['step_ns']['reference'].append(left[3])
    stats['step_ns']['candidate'].append(right[3])
    return True


def synthetic(reference, candidate, seeds, stats):
    import torch
    with torch.inference_mode():
        for seed in range(seeds):
            generator = torch.Generator().manual_seed(1000+seed)
            scale = (0.005, 0.05, 0.15, 0.3)[seed % 4]  # Larger scales reach the bisection path.
            ids = torch.tensor([0])
            reference.reset(ids)
            candidate.reset(ids)
            steps = 840
            for k in range(steps):
                if k == steps//2:
                    reference.reset(ids)
                    candidate.reset(ids)
                command = torch.tensor([COMMANDS[min(6, (k*7)//steps)]], dtype=torch.float32)
                gravity = torch.nn.functional.normalize(
                    torch.tensor([[0., 0., -1.]])+torch.randn(1, 3, generator=generator)*.05, dim=1)
                args = (torch.randn(1, 3, generator=generator)*.05, gravity, command,
                        reference.controller.nominal.float()[None]+torch.randn(1, 12, generator=generator)*scale,
                        torch.randn(1, 12, generator=generator)*.3,
                        torch.clamp(torch.full((1, 12), .02+k*.0005), max=1.))
                if not _compare(reference, candidate, args, stats):
                    reference.reset(ids)
                    candidate.reset(ids)


def replay(reference, candidate, reports, stats):
    """Recorded live observer inputs; the frozen model must also reproduce the recorded outputs."""
    import torch
    with torch.inference_mode():
        for path in reports:
            ticks = [c['observed'] for c in json.loads(Path(path).read_bytes())['measurement']['cycles']
                     if c.get('observed')]
            if [o['tick_index'] for o in ticks] != list(range(len(ticks))):
                raise ValueError('Replay requires one contiguous observer run: '+str(path))
            ids = torch.tensor([0])
            reference.reset(ids)
            candidate.reset(ids)
            for o in ticks:
                args = tuple(torch.tensor([o['inputs'][k]], dtype=torch.float32) for k in INPUT_NAMES)
                a, b = reference(*args), candidate(*args)
                stats['replay_ticks'] += 1
                if not (torch.equal(a, b) and torch.equal(reference.last_observation, candidate.last_observation) and
                        torch.equal(reference.last_actor_output, candidate.last_actor_output) and
                        all(torch.equal(x, y) for x, y in zip(reference.controller.buffers(),
                                                              candidate.controller.buffers()))):
                    stats['replay_candidate_mismatch'] += 1
                if ([float(x) for x in a[0]] != o['q_target_rad_diagnostic_only'] or
                        [float(x) for x in reference.last_observation[0]] != o['observation74'] or
                        [float(x) for x in reference.last_actor_output[0]] != o['actor_residual12']):
                    stats['replay_reference_vs_live_mismatch'] += 1
            stats['replay_runs'] += 1


def _summary(values):
    values = sorted(values)
    return None if not values else {'median_ms': values[len(values)//2]/1e6,
                                    'p99_ms': values[int(len(values)*.99)]/1e6, 'count': len(values)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path)
    parser.add_argument('--seeds', type=int, default=4)
    parser.add_argument('--replay-report', type=Path, action='append', default=[])
    parser.add_argument('--execute-cpu', action='store_true')
    args = parser.parse_args(argv)
    if not args.execute_cpu:
        print(json.dumps({'status': 'PLAN', 'reads_files': False, 'opens_devices': False,
                          'deployable_release': False, 'execute_flag': '--execute-cpu'}))
        return 0
    import torch
    torch.set_num_threads(1)
    stats = {'ticks': 0, 'compared_tensors': 0, 'mismatches': 0, 'exceptions_matched': 0,
             'exception_mismatch': 0, 'replay_runs': 0, 'replay_ticks': 0, 'replay_candidate_mismatch': 0,
             'replay_reference_vs_live_mismatch': 0, 'step_ns': {'reference': [], 'candidate': []}}
    with tempfile.TemporaryDirectory() as workdir:
        reference, candidate = build(args.bundle, workdir)
        synthetic(reference, candidate, args.seeds, stats)
        replay(reference, candidate, args.replay_report, stats)
    warm = {k: v[20:] for k, v in stats.pop('step_ns').items()}  # Exclude first-call JIT profiling.
    stats['step'] = {k: _summary(v) for k, v in warm.items()}
    stats['status'] = ('BITWISE_EQUAL_CPU_ONLY' if not stats['mismatches'] and not stats['exception_mismatch'] and
                       not stats['replay_candidate_mismatch'] and not stats['replay_reference_vs_live_mismatch']
                       else 'MISMATCH')
    stats['deployable_release'] = False
    print(json.dumps(stats, indent=1))
    return 0 if stats['status'] == 'BITWISE_EQUAL_CPU_ONLY' else 1


if __name__ == '__main__':
    raise SystemExit(main())
