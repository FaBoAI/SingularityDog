"""Independent four-CAN human current-hold30 foreground; file-only PLAN by default.

The new frozen human namespace owns its adapter and opaque primary admission.
R3 boxed reports are supplemental evidence and never admit this route. Explicit
execution requires the new reviewed source/native receipt, same-source live
prerequisites, current two-person full-support conditions and a visible terminal.
No synthetic pass, PLAN, receipt writer or public boolean grants motor output.
"""
import argparse
import copy
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import sys
import termios
import threading
import time
from types import SimpleNamespace

if not __package__:
    sys.path.insert(0, str(Path(__file__).absolute().parents[2]))
    __package__ = 'experiments.four_bus_human_hold'

from experiments.four_bus_diagnostic import foreground as FG
from experiments.four_bus_diagnostic import model_bridge, pipeline, topology
from experiments.four_bus_diagnostic.transport_adapter import IDS, PORTS, Group
from experiments.four_bus_type1 import type1_foreground as boxed_foreground
from experiments.four_bus_type1 import type1_profile as boxed_profile
from . import profile as P, type1_runner as R, type1_transport as T
from .supervision import HumanSupported30Execution

FALSE_FLAGS = {'output_approval_granted_here': False, 'approved_for_runtime': False,
               'timing_qualification_granted_here': False, 'live_type1_qualified': False,
               'physical_post_trial_observation': None, 'standing_or_walking_authorized': False}
ATTEMPTS = boxed_foreground.ATTEMPTS
STOP_UNCONFIRMED = boxed_profile.STOP_UNCONFIRMED_STATUS
need = FG.need


def verify_origins(root, manifest):
    """All source-owned modules, including this distinct foreground, are frozen."""
    P.verify_manifest(root, {'path': manifest['_manifest_path'],
                            'sha256': manifest['_manifest_sha256']},
                      len(manifest['files']), P.Pins())
    expected = Path(root)/'runtime/experiments/four_bus_human_hold/foreground.py'
    need(Path(__file__).absolute() == expected,
         'Human foreground was not loaded from its separately frozen kit')


def file_plan(args):
    """Read and validate a numeric PLAN, without importing capabilities or granting output."""
    pins = P.Pins()
    plan = pins.json(args.plan, args.plan_sha256)
    need(type(plan) is dict and plan.get('schema') == P.SCHEMA and
         plan.get('status') == 'PLAN_ONLY' and
         all(plan.get(key) is expected for key, expected in P.NO_GRANTS.items()),
         'Exact ungranting new human PLAN required')
    P.validate_contract(copy.deepcopy(plan.get('contract')))
    need(plan.get('mode') in P.MODES and plan.get('duration_s') == P.DURATIONS[plan['mode']] and
         plan.get('contract_sha256') == P.canonical_sha256(plan['contract']),
         'Human PLAN mode/duration/contract digest differs')
    result = copy.deepcopy(plan)
    result.update(status='PLAN_HUMAN_HOLD30_NOT_EXECUTED', opens_devices=False,
        loads_library_model_or_torch=False, active_primary_gates_authenticated_here=False,
        execute_requested=False, existing_R3_execution_changed=False)
    result['blockers'] = sorted(set(result.get('blockers', [])) |
                               {'EXECUTION_PRIMARY_FILES_NOT_RECHECKED_IN_DEFAULT_PLAN'})
    pins.verify()
    return result


def _native_args(plan, receipt, cli):
    """The reviewed primary integration receipt selects native bytes; CLI cannot replace them."""
    refs = plan['evidence']
    base = plan['contract']['base_numerical_contract']
    native, guard = receipt['native_type1'], receipt['native_current_guard']
    def ref(name):
        value = refs['model_inputs'][name]
        return value['path'], value['sha256']
    source_binding, source_binding_sha = ref('source_binding')
    old_profile, old_profile_sha = ref('model_profile')
    return SimpleNamespace(source_root=refs['source_root'], source_count=refs['source_count'],
        source_manifest=refs['source_manifest']['path'], source_manifest_sha256=refs['source_manifest']['sha256'],
        source_binding=source_binding, source_binding_sha256=source_binding_sha,
        original_model_profile=old_profile, original_model_profile_sha256=old_profile_sha,
        type1_library=native['library']['path'], type1_library_sha256=native['library']['sha256'],
        type1_build_sha256=native['build_record']['sha256'],
        ordinary_source_sha256=native['ordinary_source']['sha256'],
        subset_stop_source_sha256=native['subset_stop_source']['sha256'],
        type1_extension_source_sha256=native['extension_source']['sha256'],
        current_guard_library=guard['library']['path'], current_guard_library_sha256=guard['library']['sha256'],
        current_guard_build_sha256=guard['build_record']['sha256'],
        current_guard_source_sha256=guard['source']['sha256'],
        encoder_binary=None, encoder_binary_sha256=None,
        timing_evidence=cli.timing_evidence, audio_device=cli.audio_device,
        output=cli.output, holder_receipt=cli.holder_receipt,
        holder_receipt_sha256=cli.holder_receipt_sha256,
        mode=plan['mode'], duration=plan['duration_s'], power_epoch=base['motor_power_epoch'])


def observer_ticks(duration_s, control_clock):
    from singularitydog_hw.policy_clock import validate_control_clock
    need(type(duration_s) is int and duration_s in (2, 8), 'Exactly two or eight human seconds required')
    clock = validate_control_clock(control_clock)
    need(clock.policy_hz == 30, 'Only the independently retrained30Hz observer is selected')
    return duration_s*30+2


def prepare(cli):
    """Recheck genuine primary evidence before loading a model, library, TTY or hardware."""
    pins = P.Pins()
    plan = pins.json(cli.plan, cli.plan_sha256)
    admitted = P.prepare_from_files(plan)
    spec = P.runtime_spec(admitted)  # Missing integration/primary proofs fail here, before capabilities.
    checked = admitted.plain_plan()
    refs = checked['evidence']
    for path, digest in P._thaw(admitted.pins).items():
        pins.read(path, digest)
    receipt = pins.json(refs['implementation_receipt'])
    args = _native_args(checked, receipt, cli)
    manifest = pins.json(refs['source_manifest'])
    P.verify_manifest(args.source_root, refs['source_manifest'], args.source_count, pins)
    manifest = {**manifest, '_manifest_path': args.source_manifest,
                '_manifest_sha256': args.source_manifest_sha256}
    verify_origins(args.source_root, manifest)
    contract = checked['contract']['base_numerical_contract']
    source = pins.json(args.source_binding, args.source_binding_sha256)
    need(source.get('four_bus_source_manifest') == refs['source_manifest'],
         'Human native foreground source binding differs from the new source inventory')
    dependencies = boxed_foreground.model_dependencies(args, manifest, source, pins,
                                                       policy30_model=contract['policy30_model'])
    selected_capture = (refs['post_diagnostic_capture'] if checked['mode'] == 'human_current_hold'
                        else refs['model_inputs']['topology'])
    selected_events = (refs['post_diagnostic_events'] if checked['mode'] == 'human_current_hold'
                       else refs['model_inputs']['events'])
    capture = pins.json(selected_capture)
    pins.raw(selected_events)
    topology.validate_topology(capture, expected_boot=contract['boot_id'],
                              expected_power_epoch=contract['motor_power_epoch'])
    need(capture['ids_by_port'] == contract['topology_by_port'], 'Human current topology differs')
    options = boxed_profile.pacing_options(contract['pacing'])
    library_plan = boxed_foreground.prepare_type1_library(args, manifest, pins,
        prearmed_hold='prearmed_hold_lead_us' in options, separate_write_deadline=True)
    guard_plan = FG.prepare_native_guard(args, manifest, pins)
    need(type(cli.audio_device) is str and cli.audio_device.strip() == cli.audio_device != '',
         'Explicit Jetson audio output device required')
    audio = {**P.validated_audio(admitted)['clips']['brief'], 'device': cli.audio_device}
    output = FG.fresh_output(cli.output, args.source_root)
    need((cli.holder_receipt is None) == (cli.holder_receipt_sha256 is None),
         'Holder receipt requires both its absolute path and SHA')
    holder = None
    if cli.holder_receipt is not None:
        holder = FG.validate_holder_receipt(pins.json(cli.holder_receipt, cli.holder_receipt_sha256), capture)
    pins.verify()
    planned = R.plan(admitted)
    result = {**admitted.report_binding(), 'status': 'PRIMARY_READY_NOT_EXECUTED', **FALSE_FLAGS,
        **dict.fromkeys(ATTEMPTS, False), 'opens_devices': False, 'loads_library_model_or_torch': False,
        'source_count': args.source_count, 'runner_plan': planned, 'type1_library_plan': library_plan,
        'current_guard_plan': guard_plan, 'encoder': {'kind': 'python_motion_wires'},
        'firmware_by_id': spec['firmware_by_id'],
        'observer_max_ticks': observer_ticks(checked['duration_s'], contract['control_clock']),
        'model_load': 'policy_model_30hz.load(execute_cpu=True)',
        'transport_native_gain_caps': 'Kp6_Kd0.15' if checked['mode'] == 'human_current_hold' else 'zero',
        'input_sha256': dict(pins.values), 'canonical_outer_power_scope_required': True,
        'required_current_terminal': 'same_visible_controlling_terminal_input_and_output'}
    return args, {'plan': result, 'pins': pins, 'manifest': manifest, 'admitted': admitted,
        'contract': contract, 'human_contract': checked['contract'], 'mode': checked['mode'],
        'duration_s': checked['duration_s'], 'runner_admitted': admitted, 'capture': capture,
        'dependencies': dependencies, 'audio': audio, 'output': output,
        'holder_receipt': holder, 'pacing_options': options}


class LinuxEnvironment(boxed_foreground.LinuxEnvironment):
    """Source-owned dedicated Kp6 adapter, timer, audio and original guarded Linux capabilities."""
    def verify_sources(self, args, prepared, *, final):
        if final:
            P.verify_manifest(args.source_root, {'path': args.source_manifest,
                'sha256': args.source_manifest_sha256}, args.source_count, prepared['pins'])
            boxed_foreground.recheck_model_dependencies(prepared)
        verify_origins(args.source_root, prepared['manifest'])

    def verify_terminal(self):
        need(os.isatty(0) and os.isatty(1) and os.fstat(0).st_rdev == os.fstat(1).st_rdev,
             'Human execution requires the same visible terminal on stdin/stdout')
        # The outer performance scope makes a new process session but inherits
        # stdio. /dev/tty and tcgetpgrp are therefore deliberately not used.
        # Authenticate the inherited descriptors and live device, without opening
        # another terminal or treating a pipe/nonvisible root SSH as user input.
        before = []
        for fd in (0, 1):
            path = Path(os.ttyname(fd))
            info, live = os.fstat(fd), path.stat()
            need(path.is_absolute() and stat.S_ISCHR(info.st_mode) and
                 (info.st_dev, info.st_ino, info.st_rdev) == (live.st_dev, live.st_ino, live.st_rdev),
                 'Inherited visible terminal path/device binding differs')
            settings = termios.tcgetattr(fd)
            need(len(settings) == 7, 'Live terminal attributes are unavailable')
            before.append({'fd': fd, 'path': str(path), 'st_dev': info.st_dev,
                           'st_ino': info.st_ino, 'st_rdev': info.st_rdev})
        need({row['path'] for row in before} == {before[0]['path']},
             'Input/output must name the same inherited visible terminal')
        return {'descriptors': before, 'source': 'inherited_actual_stdin_stdout',
                'separate_session_supported': True, 'audibility_or_visibility_inferred': False}

    def human_supervisor(self, prepared):
        from singularitydog_hw.policy_output import HumanStageAudioPlayer, HUMAN_AUDIO_STAGES
        clips = P.validated_audio(prepared['admitted'])['clips']
        player = HumanStageAudioPlayer({name: clips[name] for name in HUMAN_AUDIO_STAGES},
                                       prepared['audio']['device'])
        supervisor = None
        try:
            supervisor = HumanSupported30Execution(0, write_fd=1, stage_audio=player)
            supervisor.bind_profile(prepared['admitted'], active=True)
            return supervisor, player
        except BaseException as primary:
            try:
                if supervisor is not None:
                    supervisor.close()
            except BaseException as cleanup:
                primary.add_note('Human terminal cleanup: '+str(cleanup))
            finally:
                try:
                    player.close()
                except BaseException as cleanup:
                    primary.add_note('Human audio cleanup: '+str(cleanup))
            raise

    def load_model(self, prepared):
        import torch
        from singularitydog_hw import policy_model_30hz as new_model
        from singularitydog_hw.policy_observer import StatefulPolicyObserver
        torch.set_num_threads(1)
        if torch.get_num_interop_threads() != 1:
            torch.set_num_interop_threads(1)
        need(torch.get_num_threads() == torch.get_num_interop_threads() == 1,
             'Actual30Hz Torch one-thread readback failed')
        selected = prepared['dependencies']['policy30_model']
        policy, proof = new_model.load(selected['artifacts'], execute_cpu=True)
        plan = copy.deepcopy(prepared['contract']['model_plan'])
        model_bridge._plan_axes(plan)
        kwargs = copy.deepcopy(plan['observer_kwargs'])
        kwargs.update(max_ticks=observer_ticks(prepared['duration_s'], plan['control_clock']),
                      torch_module=torch, checked_dispatch_wrapper=None)
        delegate = StatefulPolicyObserver(policy, copy.deepcopy(plan['calibration']), **kwargs)
        observer = model_bridge._GuardedObserver(delegate, plan)
        def warm(run, **counts):
            result = FG.selected_warmup(run, policy, None, torch,
                                        plan['observer_kwargs']['h_hypothesis'], **counts)
            result.update(real_model_verified=True,
                model_artifact_sha256=selected['artifacts']['controller']['sha256'],
                model_source_sha256=selected['artifacts']['source_manifest']['sha256'],
                model_source_sha256_by_path={ref['path']: ref['sha256'] for ref in selected['artifacts'].values()},
                control_clock=prepared['contract']['control_clock'],
                previous_50hz_model_or_timing_qualification_reused=False)
            return result
        correction = None
        hypothesis = plan['observer_kwargs']['accel_input_hypothesis']
        if hypothesis is not None:
            from singularitydog_hw.imu_accel_input_hypothesis import load_accel_input_hypothesis
            correction = load_accel_input_hypothesis(hypothesis, R.imu_frame(plan)['rotation'])
        return {'observer': observer, 'model_setup': warm, 'model_source': proof,
                'accel_correction': correction}

    def load_libraries(self, args):
        from experiments.four_bus_diagnostic import native_current_guard
        library = T.load_library(args.type1_library, expected_sha256=args.type1_library_sha256,
            ordinary_source_sha256=args.ordinary_source_sha256,
            subset_stop_source_sha256=args.subset_stop_source_sha256,
            extension_source_sha256=args.type1_extension_source_sha256,
            build_record_sha256=args.type1_build_sha256)
        guard = native_current_guard.load_library(args.current_guard_library,
            expected_sha256=args.current_guard_library_sha256,
            source_sha256=args.current_guard_source_sha256,
            build_record_sha256=args.current_guard_build_sha256)
        return library, guard, None


def transport_limits(contract, group, mode):
    need(mode in P.MODES, 'Select the independent human mode')
    axes = contract['axes']
    bounds = {mid: (axes[str(mid)]['raw_lower_rad'], axes[str(mid)]['raw_upper_rad']) for mid in group.ids}
    kp = {mid: 6. if mode == 'human_current_hold' else 0. for mid in group.ids}
    kd = {mid: .15 if mode == 'human_current_hold' else 0. for mid in group.ids}
    return bounds, kp, kd


def summarize(report, measured, transports, groups):
    boxed_foreground.summarize(report, measured, transports, groups)
    cycles = measured.get('cycles') or []
    report['all_cycles_passed'] = (measured.get('status') in P.COMPLETE_STATUS.values() and
        measured.get('normal_ramp_completed') is True and measured.get('failure_retained') is False and
        len(cycles) == report['completed_cycles'] > 0 and all(row.get('completed') is True for row in cycles))
    report['normal_ramp_completed'] = measured.get('normal_ramp_completed') is True



def execute(args, prepared, environment=None):
    """Independent explicit human-hold30 execution with owned terminal/audio supervision."""
    # Public direct callers must pass the same complete opaque execution proof
    # as the CLI. A genuine Prepared with an integration blocker is only PLAN.
    admitted = prepared['admitted']
    spec = P.runtime_spec(admitted)
    plan = admitted.plain_plan()
    need(prepared['runner_admitted'] is admitted and
         prepared['contract'] == plan['contract']['base_numerical_contract'] and
         prepared['human_contract'] == plan['contract'] and
         prepared['mode'] == spec['mode'] and prepared['duration_s'] == spec['duration_s'],
         'Human foreground execution binding differs from the complete opaque proof')
    refs = plan['evidence']
    binding_pins = P.Pins()
    receipt = binding_pins.json(refs['implementation_receipt'])
    expected_args = _native_args(plan, receipt, args)
    need(vars(args) == vars(expected_args),
         'Human foreground native/source arguments differ from the registered primary references')
    capture_ref = (refs['post_diagnostic_capture'] if spec['mode'] == 'human_current_hold'
                   else refs['model_inputs']['topology'])
    need(prepared['capture'] == binding_pins.json(capture_ref) and
         prepared['dependencies'].get('policy30_model') == spec['policy30_model'] and
         prepared['pacing_options'] == boxed_profile.pacing_options(spec['pacing']),
         'Human foreground capture/model/pacing binding differs from primary evidence')
    expected_audio = {**P.validated_audio(admitted)['clips']['brief'], 'device': args.audio_device}
    need(prepared['audio'] == expected_audio and
         prepared['output'] == FG.fresh_output(args.output, args.source_root),
         'Human foreground reviewed audio/private-output binding differs')
    need((args.holder_receipt is None) == (args.holder_receipt_sha256 is None),
         'Direct execution holder observation needs both path and SHA')
    expected_holder = None
    if args.holder_receipt is not None:
        expected_holder = FG.validate_holder_receipt(
            binding_pins.json(args.holder_receipt, args.holder_receipt_sha256), prepared['capture'])
    need(prepared['holder_receipt'] == expected_holder,
         'Human foreground registered holder observation differs')
    binding_pins.verify()
    env = LinuxEnvironment() if environment is None else environment
    output = prepared['output']
    output.mkdir(mode=0o700)
    admitted, contract, capture = prepared['admitted'], prepared['contract'], prepared['capture']
    groups = tuple(Group(port, tuple(contract['topology_by_port'][port])) for port in PORTS)
    options = prepared['pacing_options']
    evidence = getattr(args, 'timing_evidence', False)
    report = {'schema': prepared['plan']['schema'], 'status': 'STARTING', **FALSE_FLAGS, **admitted.report_binding(),
              **dict.fromkeys(ATTEMPTS, False), 'errors': [], 'failure_retained': False, 'completed_cycles': 0,
              'all_cycles_passed': False, 'first_release_monotonic_ns': None, 'terminal_stop': None,
              'physical_cutoff_required': False, 'restoration_complete': False, 'backend_kind': env.kind,
              'input_sha256': dict(prepared['pins'].values), 'settings_restoration': {},
              'plan_summary': {'runner_plan': prepared['plan']['runner_plan'],
                               'firmware_by_id': prepared['plan']['firmware_by_id'],
                               'observer_max_ticks': prepared['plan']['observer_max_ticks'],
                               'model_load': prepared['plan']['model_load'],
                               'transport_native_gain_caps': prepared['plan']['transport_native_gain_caps']}}
    if 'control_clock' in contract:
        from singularitydog_hw.policy_clock import policy_clock_settings
        selected_clock = policy_clock_settings(contract)
        report.update(control_clock=contract['control_clock'], policy30_model=contract['policy30_model'],
                      inference_tx_budget_ns=selected_clock.inference_tx_budget_ns,
                      cycle_budget_ns=selected_clock.cycle_budget_ns)
    if options:
        report['pacing_options'] = dict(options)
    if evidence:
        report['timing_evidence_selected'] = True
    primary = measured = observer = device = scope = current_guard = human = audio_player = None
    transports, worker_scopes, handlers = {}, {}, {}
    cancellation = FG.CancelBinding()
    stop, cancel = cancellation.stop, cancellation.cancel
    stop_requested = threading.Event()
    cancel_requests = []

    def cancel_from(source):
        # Host clock only: a signal handler must not take an injected clock's lock.
        def request():
            cancel_requests.append({'source': source, 'host_monotonic_ns': time.monotonic_ns()})
            cancel()
        return request

    def note(error):
        nonlocal primary
        report['errors'].append(type(error).__name__+': '+str(error))
        if primary is None:
            primary = error
        else:
            primary.add_note('Foreground cleanup: '+str(error))

    def light():
        need(not stop.is_set(), 'Foreground cancelled')

    try:
        prepared['pins'].verify()
        report['startup'] = env.startup()
        P.verify_prepared(admitted)
        need(env.boot_id() == capture['boot_before'] == contract['boot_id'] == contract['boot_id'],
             'Current boot differs from the admitted capture')
        bindings = env.bind(capture)
        need(bindings == capture['ports'], 'Current four device identities differ from capture')
        report['holders'] = env.holders(bindings)
        if prepared['holder_receipt'] is not None:
            FG.validate_holder_receipt(prepared['holder_receipt'], capture)
            need(prepared['holder_receipt']['checked_monotonic_ns'] <= env.clock(),
                 'Root holder observation is from a future clock')
            report['root_holder_observation'] = prepared['holder_receipt']
        report['terminal_binding'] = env.verify_terminal()
        if prepared['mode'] == 'human_current_hold':
            human, audio_player = env.human_supervisor(prepared)
        model = env.load_model(prepared)
        observer = model['observer']
        report['model_source'] = model['model_source']
        env.verify_sources(args, prepared, final=False)
        library, guard_library, encoder = env.load_libraries(args)
        env.verify_sources(args, prepared, final=False)
        with ExitStack() as stack:
            env.locks(stack, bindings)
            report['holders_after_locks'] = env.holders(bindings)
            cr, cw = os.pipe()
            os.set_blocking(cr, False); os.set_blocking(cw, False)
            stack.callback(os.close, cr); stack.callback(os.close, cw)
            cancellation.bind(stack, cw)
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                handlers[sig] = signal.getsignal(sig)
                signal.signal(sig, lambda signum, frame, request=cancel_from('signal_'+sig.name): request())
            handlers[signal.SIGUSR1] = signal.getsignal(signal.SIGUSR1)
            signal.signal(signal.SIGUSR1, lambda signum, frame: stop_requested.set())
            descriptors, boot_fds = env.open_ports(stack, bindings, light)
            current_guard = env.current_guard(guard_library, bindings, descriptors, boot_fds[PORTS[0]],
                                              capture['boot_before'], stop)
            # LIFO: verifies and closes before the boot/serial descriptors close.
            stack.callback(FG.finish_current_guard, current_guard, report, note)
            device = env.imu()
            try:
                report['imu_configuration'] = device.start()
                current_guard()
                scope = env.main_scope()
                def factory(group):
                    bounds, kp, kd = transport_limits(contract, group, prepared['mode'])
                    value = T.Type1Transport.create(library, descriptors[group.port], group=group, cancel_fd=cr,
                        boot_fd=boot_fds[group.port], boot_id=capture['boot_before'], axis_raw_bounds=bounds,
                        kp_cap_by_id=kp, kd_cap_by_id=kd, cancel_all=cancel_from(group.port+'_transport_failure'),
                        decode_once=options.get('decode_once') is True,
                        prearmed_hold='prearmed_hold_lead_us' in options,
                        **({'separate_write_deadline': True} if 'control_clock' in contract else {}))
                    transports[group.port] = value
                    return value
                def worker(port, mask):
                    value = env.worker_scope(port, mask)
                    worker_scopes[port] = value
                    return value
                measured = R.run(prepared['runner_admitted'], factory=factory,
                    imu_read=lambda: FG.read_imu(device, light), observer=observer, check_current=current_guard,
                    check_cancelled=light, cancel_io=cancel_from('runner_emergency'),
                    model_setup=model['model_setup'], worker_scope=worker, main_scope=lambda: scope, release_wait=env.release_wait(library, cr, observer),
                    announce=env.announcer(prepared['audio'], light), encoder=encoder,
                    stop_requested=stop_requested, accel_correction=model.get('accel_correction'),
                    backend_usage={'kind': env.kind,
                        'library_sha256': args.type1_library_sha256, 'build_record_sha256': args.type1_build_sha256,
                        'source_manifest_sha256': args.source_manifest_sha256},
                    clock=env.clock, execute=True,
                    command_wait=env.command_wait(library, cr) if 'command_phase_offset_us' in options else None,
                    timing_evidence=env.timing_evidence() if evidence else None,
                    human_supervision=human)
                if callable(getattr(observer, 'finish', None)):
                    # The tick budget is an upper bound; inference stops at gain-down.
                    report['observer'] = observer.finish()
                if measured['status'] not in P.COMPLETE_STATUS.values():
                    note(RuntimeError(str(measured.get('primary_error'))))
            finally:
                try:
                    device.close()
                    report['settings_restoration']['imu'] = device.restore_status
                    need(device.restore_status in ('restored', 'not_needed'), 'IMU register restoration failed')
                except BaseException as error:
                    note(error)
            cancellation.clear()
    except BaseException as error:
        note(error)
    finally:
        cancellation.clear()
        report['cancellation_write_errors'] = list(cancellation.errors)
        report['cancel_requests'] = list(cancel_requests)
        for error in cancellation.errors:
            note(OSError('Cancellation writer: '+error))
        if current_guard is not None and 'current_check_profile' not in report:
            report['current_check_profile'] = {'setup': current_guard.setup,
                'end_full_path_resolution_verified': False, 'calls': list(current_guard.calls)}
        for sig, handler in handlers.items():
            try:
                signal.signal(sig, handler)
            except BaseException as error:
                note(error)
        if human is not None:
            try:
                human.close()
                report['human_terminal_audio_supervision'] = {
                    'closed': human.closed, 'timer': human.timer_readback,
                    'restoration': human.restoration_report,
                    'events': list(human.events), 'failure': human.failed}
                need(human.closed and human.restored and audio_player.closed,
                     'Human terminal/audio owner restoration failed')
                if (measured or {}).get('status') in P.COMPLETE_STATUS.values():
                    need(human.failed is None, 'Completed human trial retained a supervision failure')
            except BaseException as error:
                note(error)
        restoration = report['settings_restoration']
        restoration['signal_handlers'] = all(signal.getsignal(sig) == handler for sig, handler in handlers.items())
        restoration['main'] = getattr(scope, 'report', None)
        restoration['workers'] = {port: {'timer_slack': getattr(getattr(value, 'slack', None), 'report', None),
            'original_cpu_mask': sorted(value.original) if getattr(value, 'original', None) is not None else None,
            'restored_by_runner': (measured or {}).get('restoration', {}).get(port)}
            for port, value in worker_scopes.items()}
        report['graceful_stop_requested'] = stop_requested.is_set()
        if measured is not None:
            try:
                summarize(report, measured, transports, groups)
                report['measurement'] = pipeline.evidence_report(measured)
            except BaseException as error:
                report['physical_cutoff_required'] = report['physical_cutoff_required'] or bool(transports)
                note(error)
        elif transports:
            for key in ATTEMPTS[:3]:
                report[key] = any(t.attempts.get(key) is True for t in transports.values())
            report['physical_cutoff_required'] = True
        try:
            prepared['pins'].verify()
            env.verify_sources(args, prepared, final=True)
            report['source_and_input_pins_unchanged'] = True
        except BaseException as error:
            report['source_and_input_pins_unchanged'] = False
            note(error)
        runner_restoration = (measured or {}).get('restoration', {})
        report['restoration_complete'] = (measured is not None and all(runner_restoration.get(key) is True
            for key in (*PORTS, 'imu', 'main', *(port+'_session' for port in PORTS))) and
            restoration.get('imu') in ('restored', 'not_needed') and restoration['signal_handlers'] is True and
            isinstance(restoration['main'], dict) and restoration['main'].get('restored') is True and
            report.get('current_check_profile', {}).get('native_guard_closed') is True and
            report['current_check_profile'].get('end_full_path_resolution_verified') is True and
            not cancellation.errors and
            (human is None or human.closed and human.restored and audio_player.closed))
        status = measured['status'] if measured is not None else 'ABORTED'
        if report['physical_cutoff_required']:
            status = STOP_UNCONFIRMED
        elif primary is not None and status not in (STOP_UNCONFIRMED, 'ABORTED'):
            status = 'ABORTED'
        report['status'] = status
        report['normal_ramp_completed'] = (measured or {}).get('normal_ramp_completed') is True
        report['partial_load_started'] = bool((measured or {}).get('human_supported_trial', {}).get('slight_ease_cue_ns'))
        report['standing_or_walking_authorized'] = False
        report['actual_full_support_or_weight_transfer_measured'] = False
        report['failure_retained'] = primary is not None
        report['physical_post_trial_observation'] = None
        report['outer_cpu_c7_emc_restoration_verified_here'] = False
        report['outer_wrapper_must_append_actual_restoration'] = True
        try:
            target = output/'report.json'
            raw = (json.dumps(report, indent=2, allow_nan=False)+'\n').encode()
            with os.fdopen(os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'wb') as handle:
                handle.write(raw); handle.flush(); os.fsync(handle.fileno())
        except BaseException as error:
            print('Type1 foreground report publication failed: '+repr(error), file=sys.stderr)
            raise
    return report



def parser():
    result = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('plan', 'plan-sha256', 'output', 'audio-device'):
        result.add_argument('--'+name, required=True)
    result.add_argument('--holder-receipt')
    result.add_argument('--holder-receipt-sha256')
    result.add_argument('--timing-evidence', action='store_true')
    result.add_argument('--execute', action='store_true',
        help='Recheck fresh human primary gates, then explicitly execute the reviewed new finite mode')
    return result


def main(argv=None):
    cli = parser().parse_args(argv)
    if not cli.execute:
        print(json.dumps(file_plan(cli), indent=2, allow_nan=False))
        return 0
    args, prepared = prepare(cli)
    report = execute(args, prepared)
    print(json.dumps({'status': report['status'], 'report': str(prepared['output']/'report.json'),
        'physical_cutoff_required': report['physical_cutoff_required'], **FALSE_FLAGS}))
    return 0 if report['status'] in P.COMPLETE_STATUS.values() and not report['failure_retained'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
