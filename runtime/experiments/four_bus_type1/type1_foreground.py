"""Pinned four-bus boxed live Type1 foreground; default is file-only PLAN.

Run with ``python -B -m experiments.four_bus_type1.type1_foreground`` from the
selected kit's runtime PYTHONPATH inside the canonical outer power scope
(``tools/jetson_latency_power_scope.py --supported-characterization``). The
caller establishes nice, CPU/C7/EMC and one-thread math; this module verifies
them. PLAN opens no device and loads no library, Torch or model. ``--execute``
also needs the admitted profile and the current direct-human condition record.
No PLAN, test pass, library or COMPLETE report grants output permission.
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
import subprocess
import sys
import tempfile
import threading
import time

if not __package__:
    sys.path.insert(0, str(Path(__file__).absolute().parents[2]))
    __package__ = 'experiments.four_bus_type1'

from experiments.four_bus_diagnostic import foreground as FG
from experiments.four_bus_diagnostic import model_bridge, pipeline, topology
from experiments.four_bus_diagnostic.transport_adapter import IDS, PORTS, Group
from . import build, type1_profile as P, type1_runner as R, type1_transport as T

FALSE_FLAGS = {'output_approval_granted_here': False, 'approved_for_runtime': False,
               'timing_qualification_granted_here': False, 'live_type1_qualified': False,
               'physical_post_trial_observation': None}
ATTEMPTS = ('motor_enable_sent', 'type1_sent', 'positive_gain_sent', 'learned_targets_attempted')
TIMING_KEYS = ('max_sample_age_ms', 'max_sample_gap_ms', 'hard_cycle_ms', 'period_ms',
               'max_consecutive_20ms_misses', 'voltage_min_v', 'voltage_max_v',
               'startup_damping_duration_s', 'policy_weight')
# (build-directory copy, CLI pin, receipt key, source-kit member)
TYPE1_SOURCES = (
    ('ordinary_transport.cpp', 'ordinary_source_sha256', 'ordinary_source_sha256',
     'runtime/experiments/native_active_transport/transport.cpp'),
    ('subset_stop.cpp', 'subset_stop_source_sha256', 'subset_stop_source_sha256',
     'runtime/experiments/four_bus_diagnostic/subset_stop.cpp'),
    ('transport.cpp', 'type1_extension_source_sha256', 'extension_source_sha256',
     'runtime/experiments/four_bus_type1/subset_active.cpp'))
MAX_ANNOUNCEMENT_S = 8.
TIMING_EVIDENCE_PLAN = {'selected': True, 'contract_input': False, 'diagnostic_only': True,
    'per_cycle': ['submit_done_ns', 'owner_entry_ns', 'thread_counter_delta'],
    'per_run': ['vmstat_thp_compact_delta', 'interrupts_delta'],
    'sampled': 'each_cycle_end_after_post_reply_admission_outside_release_to_output_window'}
STOP_UNCONFIRMED = P.STOP_UNCONFIRMED_STATUS
need = FG.need


def verify_origins(root, manifest):
    """Every imported pinned module, including this package, comes from the kit."""
    runtime = Path(root)/'runtime'
    packages = ('experiments.four_bus_diagnostic', 'experiments.four_bus_type1')
    for name, module in tuple(sys.modules.items()):
        if not (name.startswith('singularitydog_hw.') or name.startswith(packages) or
                name.startswith('experiments.private_checked_policy_dispatch.')):
            continue
        filename = getattr(module, '__file__', None)
        if filename is None:
            continue
        path = Path(filename).absolute()
        expected = runtime/(name.replace('.', '/')+'.py')
        if name in packages:
            expected = runtime/name.replace('.', '/')/'__init__.py'
        need(path == expected and str(path.relative_to(root)) in manifest['files'],
             'Imported module origin differs from selected kit: '+name)
    need(Path(__file__).absolute() == runtime/'experiments/four_bus_type1/type1_foreground.py',
         'Type1 foreground source origin differs')


def model_dependencies(args, manifest, source, pins):
    """FG.prepare model checks: frozen warmed sources and the unapproved checked proof."""
    frozen = source['frozen_model_source_manifest']
    old_manifest = pins.json(frozen['path'], frozen['sha256'])
    need(old_manifest.get('file_count') == 820 and len(old_manifest.get('files', {})) == 820,
         'Exact original warmed model-source inventory required')
    for name, digest in source['frozen_model_source_sha256'].items():
        need(manifest['files'].get('runtime/'+name, {}).get('sha256') == digest and
             old_manifest['files'].get('runtime/'+name, {}).get('sha256') == digest,
             'Model source bytes must remain the original warmed source: '+name)
    from singularitydog_hw import policy_active_fk as fk, policy_checked_dispatch as checked, policy_live_profile
    profile = policy_live_profile.load_profile(args.original_model_profile, require_approved=False)
    need(profile.get('approved_for_supported_policy_output') is False and profile.get('review') is None and
         profile.get('output_allowed', False) is False, 'Original model dependency profile must remain unapproved')
    fk_plan, checked_plan = fk.plan(profile), checked.plan(profile)
    need(profile.get('_checked_model_plan') == checked_plan, 'Original loader checked proof differs')
    for path, digest in checked_plan['input_sha256'].items():
        pins.read(path, digest)
    return {'profile': profile, 'fk_plan': fk_plan, 'checked_plan': checked_plan}


def recheck_model_dependencies(prepared):
    from singularitydog_hw import policy_active_fk as fk, policy_checked_dispatch as checked
    value = prepared['dependencies']
    need(fk.plan(value['profile']) == value['fk_plan'] and
         checked.plan(value['profile']) == value['checked_plan'], 'Model file proof changed after run')


def lineage_envelopes(contract, pins):
    """Pinned prepare_model_plan inputs, exactly the contract lineage."""
    lineage = contract['lineage']
    def text(name):
        ref = lineage[name]
        return {'reference': dict(ref), 'raw_json': pins.read(ref['path'], ref['sha256']).decode('utf-8')}
    inputs = {name: text(name) for name in lineage if name != 'events'}
    events = lineage['events']
    inputs['events'] = {'reference': dict(events),
                        'raw_jsonl': pins.read(events['path'], events['sha256']).decode('utf-8')}
    need(set(inputs) <= {'topology', 'events', 'calibration', 'model_profile', 'source_binding', 'mount',
                         'bias', 'accel_hypothesis'}, 'Unexpected contract lineage input')
    return inputs


def reviewed_firmware(contract, pins):
    """Tested raw firmware bytes from the pinned geometry's hardware review.

    Only a rejection gate (fresh version bytes must equal the firmware whose
    command-loss watchdog was tested); no review acceptance is reused.
    """
    reference = contract['axis_geometry']['profile']
    geometry = pins.json(reference['path'], reference['sha256'])
    need(P.validate_geometry(copy.deepcopy(geometry)) == contract['axis_geometry']['route'] and
         all(geometry['axes'][key]['uid'] == uid for key, uid in contract['uids_by_id'].items()),
         'Pinned reviewed geometry differs from the admitted contract')
    review = geometry['artifacts'].get('hardware_review')
    need(type(review) is dict and set(review) == {'path', 'sha256'}, 'Pinned hardware review reference required')
    path = Path(review['path'])
    if not path.is_absolute():
        path = Path(reference['path']).parent/path
    rows = pins.json(path, review['sha256']).get('device_watchdog')
    need(type(rows) is dict and set(rows) == {str(mid) for mid in IDS}, 'Twelve reviewed watchdog rows required')
    result = {}
    for key, row in rows.items():
        need(type(row) is dict and row.get('motor_model') == 'RS05' and
             row.get('actual_command_loss_test_passed') is True and row.get('disabled_after_loss_verified') is True and
             row.get('configured_timeout_ms') == 200, 'Reviewed RS05 command-loss watchdog evidence required: ID'+key)
        value = row.get('version_bytes_hex')
        need(type(value) is str and len(value) == 8 and all(c in '0123456789abcdef' for c in value),
             'Tested raw firmware version_bytes_hex required: ID'+key)
        result[key] = value
    return result


def runner_admitted(admitted, firmware_by_id):
    """Exact type1_runner mapping of a genuine admission; pure, opens nothing.

    No reviewed first-cycle post-reply allowance exists in the contract, so
    the stricter rule (no startup allowance) is selected explicitly.
    """
    admitted = P.verify_admitted(admitted)
    contract = P.thaw(admitted.contract)
    timing = contract['timing']
    need(type(firmware_by_id) is dict and set(firmware_by_id) == {str(mid) for mid in IDS},
         'Twelve reviewed firmware fingerprints required')
    names = ('physical_lower_rad', 'physical_upper_rad', 'lower_rad', 'upper_rad', *P.CAP_KEYS)
    axes = {key: {'uid': row['uid'], 'sign': row['sign'], 'offset_rad': row['nominal_offset_rad'],
                  **{name: row[name] for name in names}} for key, row in contract['axes'].items()}
    profile = {'axes': axes, 'start_pose_bounds': contract['start_pose_bounds'],
               **{key: timing[key] for key in TIMING_KEYS}, 'duration_s': admitted.duration_s,
               **contract['ramps'], **contract['imu_limits']}
    return {'mode': admitted.mode, 'duration_s': admitted.duration_s,
            'ids_by_port': contract['topology_by_port'], 'boot_id': admitted.boot_id,
            'motor_power_epoch': admitted.motor_power_epoch, 'contract_sha256': admitted.contract_sha256,
            'profile': profile,
            'offsets_by_id': {key: row['fixed_offset_rad'] for key, row in contract['axes'].items()},
            'reference_turns_by_id': {key: row['reference_turns'] for key, row in contract['axes'].items()},
            'uids_by_id': dict(contract['uids_by_id']), 'firmware_by_id': dict(firmware_by_id),
            'pacing': contract['pacing'], 'post_reply_policy': timing['post_reply_deadline_policy'],
            'first_cycle_post_reply': False, 'model_plan': contract['model_plan']}


def transport_limits(contract, group, mode):
    """Native member windows/caps; zero-gain timing pins native kp=kd caps to zero."""
    need(mode in P.MODES, 'Explicit mode required')
    learned = mode == 'learned_boxed'
    axes = contract['axes']
    bounds = {mid: (axes[str(mid)]['raw_lower_rad'], axes[str(mid)]['raw_upper_rad']) for mid in group.ids}
    kp = {mid: axes[str(mid)]['kp'] if learned else 0. for mid in group.ids}
    kd = {mid: axes[str(mid)]['kd'] if learned else 0. for mid in group.ids}
    return bounds, kp, kd


def observer_ticks(duration_s):
    """Every absolute slot of the finite run window (duration+40 ms) at 20 ms."""
    need(type(duration_s) is int and duration_s in P.DURATIONS, 'Duration must be exactly 2, 10 or 20 s')
    return duration_s*50+2


def create_type1_observer(plan, observer_factory, *, policy, duration_s, torch_module, checked_dispatch_wrapper):
    """model_bridge.create_guarded_observer with a duration tick budget (it allows only 5/501)."""
    owned = copy.deepcopy(plan)
    model_bridge._plan_axes(owned)
    need(callable(observer_factory), 'Explicit observer factory required')
    need(checked_dispatch_wrapper is not None, 'Selected checked wrapper must remain explicit')
    kwargs = copy.deepcopy(owned['observer_kwargs'])
    kwargs.update(max_ticks=observer_ticks(duration_s), torch_module=torch_module,
                  checked_dispatch_wrapper=checked_dispatch_wrapper)
    delegate = observer_factory(policy, copy.deepcopy(owned['calibration']), **kwargs)
    need(callable(getattr(delegate, 'consume', None)), 'Ordinary observer consume required')
    return model_bridge._GuardedObserver(delegate, owned)


def prepare_type1_library(args, manifest, pins, *, prearmed_hold=False):
    """File-only subset-active receipt/source/binary binding; never loads the CDLL.

    A contract that selects the pre-armed hold also needs the receipt to
    record the optional sda_subset_exchange_at ABI (rejected here, file-only).
    """
    library = Path(args.type1_library)
    need(library.is_absolute() and library.name == build.LIBRARY_NAME, 'Absolute four-bus Type1 library required')
    pins.read(library, args.type1_library_sha256)
    record = pins.json(library.parent/'build-record.json', args.type1_build_sha256)
    problem = build.receipt_problem(library.parent)
    need(problem is None, problem or '')
    scope = record['four_bus_subset_active']
    for name, pin, key, member in TYPE1_SOURCES:
        digest = getattr(args, pin)
        pins.read(library.parent/name, digest)
        need(scope.get(key) == digest and manifest['files'].get(member, {}).get('sha256') == digest,
             'Type1 build must include the selected current source bytes: '+member)
    need(record.get('binary_sha256') == args.type1_library_sha256 and
         record.get('source_sha256') == args.type1_extension_source_sha256 and
         scope.get('output_allowed') is False and scope.get('timing_admission_eligible') is False,
         'Exact subset-active build/binary binding required')
    result = {'schema': 'singularitydog.four-bus-type1-library-file-plan.v1', 'scope': build.SCOPE,
              'library': {'path': str(library), 'sha256': args.type1_library_sha256},
              'build': {'path': str(library.parent/'build-record.json'), 'sha256': args.type1_build_sha256},
              'sources': {name: getattr(args, pin) for name, pin, _, _ in TYPE1_SOURCES},
              'loads_library_in_plan': False, 'output_allowed': False}
    if prearmed_hold:
        need(scope.get('exchange_at_abi') == 1,
             'Pre-armed hold requires a Type1 build whose receipt records exchange_at_abi 1')
        result['exchange_at_abi'] = 1
    return result


def prepare_announcement(args, pins):
    from singularitydog_hw.policy_output import pinned_audio_file
    audio = pinned_audio_file(args.announcement_audio, args.announcement_audio_sha256)
    need(audio['path'] == str(Path(args.announcement_audio)) and audio['duration_s'] <= MAX_ANNOUNCEMENT_S,
         'Absolute nonsymlink announcement of at most 8 s required')
    pins.read(audio['path'], audio['sha256'])
    need(type(args.audio_device) is str and args.audio_device.strip() == args.audio_device != '',
         'Explicit audio device required')
    return {**audio, 'device': args.audio_device}


def prepare(args):
    """Pure file/source/admission verification, without Torch, libraries or devices."""
    pins = FG.Pins()
    manifest = pins.json(args.source_manifest, args.source_manifest_sha256)
    FG.verify_manifest(args.source_root, manifest, args.source_count, pins)
    verify_origins(args.source_root, manifest)
    files = {'profile': {'path': args.profile, 'sha256': args.profile_sha256},
             'conditions': {'path': args.conditions, 'sha256': args.conditions_sha256}}
    admitted = P.admit(pins.json(args.profile, args.profile_sha256), pins.json(args.conditions, args.conditions_sha256),
                       expected_power_epoch=args.power_epoch, files=files)
    need(admitted.mode == args.mode and admitted.duration_s == args.duration,
         'Explicit --mode/--duration must equal the admitted profile')
    contract = P.thaw(admitted.contract)
    manifest_ref = {'path': args.source_manifest, 'sha256': args.source_manifest_sha256}
    need(contract['source_manifest'] == manifest_ref and
         contract['lineage'].get('source_binding') == {'path': args.source_binding,
                                                       'sha256': args.source_binding_sha256} and
         contract['lineage'].get('model_profile') == {'path': args.original_model_profile,
                                                      'sha256': args.original_model_profile_sha256},
         'Admitted contract lineage differs from the selected source/model inputs')
    source = pins.json(args.source_binding, args.source_binding_sha256)
    need(source.get('four_bus_source_manifest') == manifest_ref, 'New whole-source binding differs')
    dependencies = model_dependencies(args, manifest, source, pins)
    need(model_bridge.prepare_model_plan(**lineage_envelopes(contract, pins)) == contract['model_plan'],
         'Embedded model plan differs from its pinned lineage files')
    current = P.thaw(admitted.profile['evidence']['current_capture'])
    need({'path': args.topology, 'sha256': args.topology_sha256} == current['topology'] and
         {'path': args.events, 'sha256': args.events_sha256} == current['events'],
         'Device topology must be the admitted current capture')
    capture = pins.json(args.topology, args.topology_sha256)
    pins.read(args.events, args.events_sha256)
    topology.validate_topology(capture, expected_boot=contract['boot_id'], expected_power_epoch=args.power_epoch)
    need(capture['ids_by_port'] == contract['topology_by_port'] and capture['boot_before'] == contract['boot_id'],
         'Current capture topology/boot differs from the contract')
    firmware = reviewed_firmware(contract, pins)
    spec = runner_admitted(admitted, firmware)
    planned = R.plan(spec)
    options = P.pacing_options(contract['pacing'])
    library_plan = prepare_type1_library(args, manifest, pins, prearmed_hold='prearmed_hold_lead_us' in options)
    guard_plan = FG.prepare_native_guard(args, manifest, pins)
    need((args.encoder_binary is None) == (args.encoder_binary_sha256 is None), 'Encoder binary needs path and SHA')
    encoder = None
    if args.encoder_binary is not None:
        pins.read(args.encoder_binary, args.encoder_binary_sha256)
        encoder = {'path': args.encoder_binary, 'sha256': args.encoder_binary_sha256, 'loads_in_plan': False}
    audio = prepare_announcement(args, pins)
    output = FG.fresh_output(args.output, args.source_root)
    need((args.holder_receipt is None) == (args.holder_receipt_sha256 is None),
         'Root holder receipt needs both path and SHA')
    holder_receipt = None
    if args.holder_receipt is not None:
        holder_receipt = FG.validate_holder_receipt(pins.json(args.holder_receipt, args.holder_receipt_sha256), capture)
    verify_origins(args.source_root, manifest)  # Includes modules imported by the checks above.
    pins.verify()
    result = {'schema': P.REPORT_SCHEMA, 'status': 'PLAN', **FALSE_FLAGS, **dict.fromkeys(ATTEMPTS, False),
        'opens_devices': False, 'loads_library_model_or_torch': False, **admitted.report_binding(),
        'source_count': args.source_count, 'runner_plan': planned, 'type1_library_plan': library_plan,
        'current_guard_plan': guard_plan, 'encoder': encoder or {'kind': 'python_motion_wires'},
        'announcement': audio, 'firmware_fingerprint_source': 'pinned_geometry_hardware_review_device_watchdog',
        'firmware_by_id': firmware, 'observer_max_ticks': observer_ticks(admitted.duration_s),
        'model_load': 'policy_active_fk.diagnostic_load; checked.load(active=False)',
        'transport_native_gain_caps': 'profile_kp_kd' if admitted.mode == 'learned_boxed' else 'zero',
        'fk_plan': dependencies['fk_plan'], 'checked_plan': dependencies['checked_plan'],
        'input_sha256': dict(pins.values), 'canonical_outer_power_scope_required': True,
        'direct_human_condition_record_sha256': admitted.conditions_sha256}
    # Opt-in fields appear only when selected; the default PLAN is unchanged.
    if options:
        result['pacing_options'] = options
    if getattr(args, 'timing_evidence', False):
        result['timing_evidence'] = TIMING_EVIDENCE_PLAN
    return {'plan': result, 'pins': pins, 'manifest': manifest, 'admitted': admitted, 'contract': contract,
            'runner_admitted': spec, 'capture': capture, 'dependencies': dependencies, 'audio': audio,
            'output': output, 'holder_receipt': holder_receipt, 'pacing_options': options}


def make_command_wait(library, cancel_fd):
    """F1 command-phase wait: the release_wait primitive (native, GIL released,
    cancel FD watched, spin 500 us) without arming the observer."""
    from singularitydog_hw.native_active_transport import wait_until
    def wait(target_ns):
        return wait_until(library, cancel_fd, target_ns, spin_us=500)
    return wait


class LinuxEnvironment:
    """Actual OS/device/model capabilities; only execute() constructs it."""
    kind = 'genuine_subset_active_cpp'
    clock = staticmethod(time.monotonic_ns)

    def startup(self):
        return FG.startup_readback()

    def boot_id(self):
        return FG.BOOT_PATH.read_text().strip()

    def bind(self, capture):
        return topology.bind_ports({port: capture['ports'][port]['path'] for port in PORTS})

    def holders(self, bindings):
        return FG.observed_holders(bindings)

    def verify_sources(self, args, prepared, *, final):
        if final:
            FG.verify_manifest(args.source_root, prepared['manifest'], args.source_count, prepared['pins'])
            recheck_model_dependencies(prepared)
        verify_origins(args.source_root, prepared['manifest'])

    def load_model(self, prepared):
        import torch
        torch.set_num_threads(1)
        if torch.get_num_interop_threads() != 1:
            torch.set_num_interop_threads(1)
        need(torch.get_num_threads() == torch.get_num_interop_threads() == 1, 'Actual Torch one-thread readback failed')
        from singularitydog_hw import policy_active_fk as fk, policy_checked_dispatch as checked
        from singularitydog_hw.policy_observer import StatefulPolicyObserver
        profile, checked_plan = prepared['dependencies']['profile'], prepared['dependencies']['checked_plan']
        original_policy, original_proof = fk.diagnostic_load(profile)
        policy, wrapper, proof = checked.load(profile, original_policy, original_proof, active=False)
        observer = create_type1_observer(prepared['contract']['model_plan'], StatefulPolicyObserver, policy=policy,
            duration_s=prepared['admitted'].duration_s, torch_module=torch, checked_dispatch_wrapper=wrapper)
        def warm(run, **counts):
            value = FG.selected_warmup(run, policy, wrapper, torch, profile['h_hypothesis'], **counts)
            sources = checked_plan['source_sha256']
            value.update(real_model_verified=True,
                model_artifact_sha256=checked_plan['references']['checked_model']['sha256'],
                model_source_sha256=hashlib.sha256(json.dumps(sources, sort_keys=True,
                    separators=(',', ':')).encode()).hexdigest(), model_source_sha256_by_path=dict(sources))
            return value
        # The runner's body-limit check uses its own copy, as OM loads its own (OM:106-119).
        kwargs = prepared['contract']['model_plan']['observer_kwargs']
        correction = None
        if kwargs['accel_input_hypothesis'] is not None:
            from singularitydog_hw.imu_accel_input_hypothesis import load_accel_input_hypothesis
            correction = load_accel_input_hypothesis(kwargs['accel_input_hypothesis'],
                                                     R.imu_frame(prepared['contract']['model_plan'])['rotation'])
        return {'observer': observer, 'model_setup': warm, 'model_source': proof, 'accel_correction': correction}

    def load_libraries(self, args):
        from experiments.four_bus_diagnostic import native_current_guard
        library = T.load_library(args.type1_library, expected_sha256=args.type1_library_sha256,
            ordinary_source_sha256=args.ordinary_source_sha256,
            subset_stop_source_sha256=args.subset_stop_source_sha256,
            extension_source_sha256=args.type1_extension_source_sha256,
            build_record_sha256=args.type1_build_sha256)
        guard = native_current_guard.load_library(args.current_guard_library,
            expected_sha256=args.current_guard_library_sha256, source_sha256=args.current_guard_source_sha256,
            build_record_sha256=args.current_guard_build_sha256)
        encoder = None
        if args.encoder_binary is not None:
            from singularitydog_hw import native_policy_batch_encode
            encoder = native_policy_batch_encode.load_verified_module(args.encoder_binary,
                expected_binary_sha256=args.encoder_binary_sha256)
        return library, guard, encoder

    def locks(self, stack, bindings):
        from singularitydog_hw import can_timing_probe, dual_can_pipeline_benchmark
        stack.enter_context(can_timing_probe.ownership_locks())
        for port in PORTS:
            stack.enter_context(dual_can_pipeline_benchmark.port_lock(bindings[port]['resolved']))
        topology.check_bindings(bindings)

    def open_ports(self, stack, bindings, check):
        import serial
        descriptors, boot_fds = {}, {}
        for port in PORTS:
            check()
            serial_port = serial.Serial(port=None, baudrate=921600, timeout=0, write_timeout=.1, exclusive=True)
            stack.callback(serial_port.close)
            serial_port.dtr = False; serial_port.rts = False
            serial_port.port = bindings[port]['path']; serial_port.open()
            info = os.fstat(serial_port.fileno())
            need(stat.S_ISCHR(info.st_mode) and info.st_rdev == bindings[port]['st_rdev'],
                 'Opened CAN descriptor differs from current topology')
            descriptors[port] = serial_port.fileno()
            boot_fds[port] = stack.enter_context(FG.BOOT_PATH.open('rb')).fileno()
        return descriptors, boot_fds

    def current_guard(self, library, bindings, descriptors, boot_fd, boot_id, stop):
        from experiments.four_bus_diagnostic import native_current_guard
        return native_current_guard.NativeCurrentGuard(library, bindings, descriptors, boot_fd, boot_id, stop)

    def imu(self):
        from singularitydog_hw import imu
        return imu.ICM20948()

    def main_scope(self):
        return FG.MainScope(True, True)

    def worker_scope(self, port, mask):
        return pipeline.LinuxWorkerScope(port, mask)

    def release_wait(self, library, cancel_fd, observer):
        return FG.make_release_wait(library, cancel_fd, observer)

    def command_wait(self, library, cancel_fd):
        return make_command_wait(library, cancel_fd)

    def timing_evidence(self):
        from .timing_evidence import TimingEvidence
        return TimingEvidence()

    def announcer(self, audio, check):
        def announce():
            check()
            raw = Path(audio['path']).read_bytes()
            need(hashlib.sha256(raw).hexdigest() == audio['sha256'], 'Pinned announcement audio changed')
            with tempfile.TemporaryFile() as clip:
                clip.write(raw); clip.flush(); clip.seek(0)
                subprocess.run(['aplay', '-D', audio['device']], check=True, timeout=audio['duration_s']+1.,
                               stdin=clip, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            check()
        return announce


def terminal_stop(measured, groups):
    """All-12 summary of the runner's per-port repeated subset STOP results."""
    results = measured.get('stop_results') or {}
    confirmed, ambiguous, faults = set(), set(), {}
    for group in groups:
        row = results.get(group.port)
        if not isinstance(row, dict):
            continue
        members = set(group.ids)
        confirmed |= set(row.get('confirmed_ids') or ()) & members
        ambiguous |= set(row.get('ambiguous_ids') or ()) & members
        for mid in group.ids:
            faults[str(mid)] = (row.get('faults') or {}).get(str(mid))
    confirmed -= ambiguous
    fault_by_id = {str(mid): faults.get(str(mid)) for mid in IDS}
    opened = measured.get('stop_confirmed') is not None
    complete = (measured.get('stop_confirmed') is True and confirmed == set(IDS) and not ambiguous and
                all(type(value) is int and value == 0 for value in fault_by_id.values()))
    return {'stop_confirmed': complete if opened else None, 'confirmed_ids': sorted(confirmed),
            'unconfirmed_ids': sorted(set(IDS)-confirmed), 'ambiguous_ids': sorted(ambiguous),
            'fault_by_id': fault_by_id, 'physical_cutoff_required': opened and not complete,
            'finished_monotonic_ns': measured.get('all_workers_joined_monotonic_ns'),
            'policy': 'repeated_subset_stop_each_own_owner_concurrent_shared_1250ms'}


def summarize(report, measured, transports, groups):
    """Top-level REPORT_REQUIRED_KEYS from the runner report and actual transport attempts."""
    for key in ATTEMPTS:
        report[key] = (measured.get(key) is True or
                       any(getattr(t, 'attempts', {}).get(key) is True for t in transports.values()))
    cycles = measured.get('cycles') or []
    report['completed_cycles'] = measured.get('completed_cycles', 0)
    report['first_release_monotonic_ns'] = cycles[0].get('release_ns') if cycles else None
    report['all_cycles_passed'] = (str(measured.get('status')).startswith('COMPLETE_FOUR_BUS_TYPE1_') and
        measured.get('normal_ramp_completed') is True and measured.get('failure_retained') is False and
        len(cycles) == report['completed_cycles'] > 0 and all(row.get('completed') is True for row in cycles))
    report['terminal_stop'] = terminal_stop(measured, groups)
    report['physical_cutoff_required'] = (measured.get('physical_cutoff_required') is not False or
                                          report['terminal_stop']['physical_cutoff_required'] is True)
    report['runner_status'] = measured.get('status')
    report['stop_reason'] = measured.get('stop_reason')
    report['runner_errors'] = list(measured.get('errors') or ())
    report['max_iteration_ms'] = measured.get('max_iteration_ms')
    report['transport_attempts'] = {port: dict(getattr(t, 'attempts', {})) for port, t in transports.items()}
    report['transport_failures'] = {port: list(getattr(t, 'failures', ())) for port, t in transports.items()}


def execute(args, prepared, environment=None):
    """Explicit boxed live Type1. Only the human/root operator calls this."""
    env = LinuxEnvironment() if environment is None else environment
    output = prepared['output']
    output.mkdir(mode=0o700)
    admitted, contract, capture = prepared['admitted'], prepared['contract'], prepared['capture']
    groups = tuple(Group(port, tuple(contract['topology_by_port'][port])) for port in PORTS)
    options = prepared['pacing_options']
    evidence = getattr(args, 'timing_evidence', False)
    report = {'schema': P.REPORT_SCHEMA, 'status': 'STARTING', **FALSE_FLAGS, **admitted.report_binding(),
              **dict.fromkeys(ATTEMPTS, False), 'errors': [], 'failure_retained': False, 'completed_cycles': 0,
              'all_cycles_passed': False, 'first_release_monotonic_ns': None, 'terminal_stop': None,
              'physical_cutoff_required': False, 'restoration_complete': False, 'backend_kind': env.kind,
              'input_sha256': dict(prepared['pins'].values), 'settings_restoration': {},
              'plan_summary': {'runner_plan': prepared['plan']['runner_plan'],
                               'firmware_by_id': prepared['plan']['firmware_by_id'],
                               'observer_max_ticks': prepared['plan']['observer_max_ticks'],
                               'model_load': prepared['plan']['model_load'],
                               'transport_native_gain_caps': prepared['plan']['transport_native_gain_caps']}}
    if options:
        report['pacing_options'] = dict(options)
    if evidence:
        report['timing_evidence_selected'] = True
    primary = measured = observer = device = scope = current_guard = None
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
        P.verify_admitted(admitted)
        need(env.boot_id() == capture['boot_before'] == contract['boot_id'] == admitted.boot_id,
             'Current boot differs from the admitted capture')
        bindings = env.bind(capture)
        need(bindings == capture['ports'], 'Current four device identities differ from capture')
        report['holders'] = env.holders(bindings)
        if prepared['holder_receipt'] is not None:
            FG.validate_holder_receipt(prepared['holder_receipt'], capture)
            need(prepared['holder_receipt']['checked_monotonic_ns'] <= env.clock(),
                 'Root holder observation is from a future clock')
            report['root_holder_observation'] = prepared['holder_receipt']
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
                    bounds, kp, kd = transport_limits(contract, group, admitted.mode)
                    value = T.Type1Transport.create(library, descriptors[group.port], group=group, cancel_fd=cr,
                        boot_fd=boot_fds[group.port], boot_id=capture['boot_before'], axis_raw_bounds=bounds,
                        kp_cap_by_id=kp, kd_cap_by_id=kd, cancel_all=cancel_from(group.port+'_transport_failure'),
                        decode_once=options.get('decode_once') is True,
                        prearmed_hold='prearmed_hold_lead_us' in options)
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
                    timing_evidence=env.timing_evidence() if evidence else None)
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
            not cancellation.errors)
        status = measured['status'] if measured is not None else 'ABORTED'
        if report['physical_cutoff_required']:
            status = STOP_UNCONFIRMED
        elif primary is not None and status not in (STOP_UNCONFIRMED, 'ABORTED'):
            status = 'ABORTED'
        report['status'] = status
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
    for name in ('source-manifest', 'source-binding', 'original-model-profile', 'profile', 'conditions',
                 'topology', 'events', 'type1-library', 'current-guard-library', 'announcement-audio'):
        result.add_argument('--'+name, required=True)
        result.add_argument('--'+name+'-sha256', required=True)
    result.add_argument('--source-root', required=True)
    result.add_argument('--source-count', type=int, required=True)
    for name in ('type1-build', 'ordinary-source', 'subset-stop-source', 'type1-extension-source',
                 'current-guard-build', 'current-guard-source'):
        result.add_argument('--'+name+'-sha256', required=True)
    result.add_argument('--mode', choices=P.MODES, required=True)
    result.add_argument('--duration', type=int, choices=P.DURATIONS, required=True)
    result.add_argument('--power-epoch', required=True)
    result.add_argument('--audio-device', required=True)
    result.add_argument('--encoder-binary', help='Optional pinned native batch encoder (VerifiedBatchModule)')
    result.add_argument('--encoder-binary-sha256')
    result.add_argument('--holder-receipt')
    result.add_argument('--holder-receipt-sha256')
    result.add_argument('--timing-evidence', action='store_true',
        help='Diagnostic only (not contract): per-cycle submit/owner-entry stamps and per-thread schedstat/'
             'rusage deltas at cycle end, run-level vmstat thp/compact and interrupts deltas')
    result.add_argument('--output', required=True)
    result.add_argument('--execute', action='store_true',
        help='Explicit boxed Type1 run of the admitted mode/duration; needs the current condition record')
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    prepared = prepare(args)
    if not args.execute:
        print(json.dumps(prepared['plan'], indent=2, allow_nan=False))
        return 0
    report = execute(args, prepared)
    print(json.dumps({'status': report['status'], 'report': str(prepared['output']/'report.json'),
                      'physical_cutoff_required': report['physical_cutoff_required'], **FALSE_FLAGS}))
    return 0 if report['status'] in P.COMPLETE_STATUS.values() and not report['failure_retained'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
