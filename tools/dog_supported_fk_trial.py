#!/usr/bin/env python3
"""Foreground entry for the separately reviewed boxed FK 2 -> 10 -> 20 s probes.

The default is a file-only plan. This tool never creates reviews, changes a
duration, infers a motor-power epoch, or skips the complete profile loader.
Use the existing CPU performance scope outside this entry when measuring it.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REQUEST_GAP_US = 900
DEFAULT_RELEASE_SPIN_US = 200
if not sys.dont_write_bytecode:
    raise RuntimeError('Launch this file-only entry with python -B (or PYTHONDONTWRITEBYTECODE=1)')
sys.path.insert(0, str(ROOT / 'runtime'))
from singularitydog_hw import policy_active_fk as fk
from singularitydog_hw import policy_live_profile as live
from analyze_stationary_velocity_probe import exact, need, read_file
from python_thread_switch_scope import run as run_switched_module


def pin(path, expected, label):
    need(type(expected) is str and len(expected) == 64 and
         all(c in '0123456789abcdef' for c in expected), label + ' SHA256 required')
    raw, ref = read_file(path, 64 * 1024 * 1024)
    exact(ref['sha256'], expected, label + ' SHA256')
    return raw, ref


def private_output_root(path):
    root = Path(path).expanduser().absolute()
    need(root.is_dir() and not any(p.is_symlink() for p in (root, *root.parents)) and
         not any((p / '.git').exists() for p in (root, *root.parents)),
         'Existing private non-symlink output root outside Git required')
    return root


def request_gap_us(value):
    # Same range as policy_live_profile.add_transport_arguments, without
    # introducing a request-window override to this fixed-window entry.
    try:
        number = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('request-gap-us must be an integer 600..5000') from error
    if not 600 <= number <= 5000:
        raise argparse.ArgumentTypeError('request-gap-us must be an integer 600..5000')
    return number


def timing_selection(args):
    changed = (args.request_gap_us != DEFAULT_REQUEST_GAP_US or
               args.release_spin_us != DEFAULT_RELEASE_SPIN_US)
    reason = args.timing_selection_reason
    need(reason is not None or not changed,
         'Non-default timing requires --timing-selection-reason')
    if reason is not None:
        need(type(reason) is str and 0 < len(reason.strip()) <= 1024 and reason.isprintable(),
             'Timing selection reason must be 1..1024 printable characters')
        reason = reason.strip()
    else:
        reason = 'Existing foreground entry defaults: request gap 900 us and release spin 200 us.'
    return {
        'request_gap_us': args.request_gap_us, 'request_window': 3,
        'period_ms': 20, 'release_spin_us': args.release_spin_us,
        'reason': reason,
        'reason_source': 'explicit_cli' if args.timing_selection_reason is not None else 'existing_default',
        'differs_from_foreground_defaults': changed,
        'foreground_defaults': {'request_gap_us': DEFAULT_REQUEST_GAP_US,
                                'release_spin_us': DEFAULT_RELEASE_SPIN_US},
    }


def build_command(args, profile, output):
    switch = ROOT / 'tools/python_thread_switch_scope.py'
    command = [sys.executable, '-B', str(switch), '--module',
        'singularitydog_hw.policy_output', '--interval-us', '100', '--',
        '--profile', str(Path(args.profile).absolute()), '--execute-supported',
        '--profile-sha256', args.profile_sha256, '--library-sha256', args.library_sha256,
        '--support-in-place', '--cutoff-ready',
        '--front-port', args.front_port, '--rear-port', args.rear_port,
        '--library', str(Path(args.library).absolute()), '--output', str(output),
        '--audio', str(Path(args.audio).absolute()), '--audio-sha256', args.audio_sha256,
        '--audio-device', args.audio_device, '--power-epoch', args.power_epoch,
        '--request-gap-us', str(args.request_gap_us), '--request-window', '3',
        '--pre-cycle-policy-warmup-calls', '10', '--main-thread-cpu', '4',
        '--exclude-policy-cpu-from-workers', '--post-pin-policy-prime-calls', '10',
        '--defer-gc-during-cycles', '--absolute-epoch-cadence',
        '--release-spin-us', str(args.release_spin_us), '--active-timer-slack-ns', '1000',
        '--single-thread-math']
    if profile.get('prepare_voltage_before_feedback_publication') is True:
        command.append('--prepare-voltage-before-feedback-publication')
    if profile.get('native_feedback_batch_decode') is True:
        command.append('--native-feedback-batch-decode')
    if profile.get('unpaired_output_future_notifications') is True:
        command.append('--unpaired-output-future-notifications')
    if profile.get('native_checked_policy_dispatch') is True:
        selected=profile['artifacts']['checked_model_manifest']
        command += ['--checked-model-manifest',selected['path'],
                    '--checked-model-manifest-sha256',selected['sha256']]
    if profile.get('native_phase_pair') is True:
        command.append('--native-phase-pair')
    return command


def prepare(args):
    pin(__file__, args.launcher_sha256, 'Foreground launcher')
    _, profile_pin = pin(args.profile, args.profile_sha256, 'Profile')
    profile = live.load_profile(args.profile, require_approved=args.execute_supported)
    exact(profile['profile_sha256'], profile_pin['sha256'], 'Loaded profile SHA256')
    need(fk.selected(profile), 'This entry requires explicit bounded FK selection')
    exact(profile['motor_power_epoch'], args.power_epoch, 'Explicit motor-power epoch')
    exact(profile['request_gap_us'], args.request_gap_us, 'Request gap')
    exact(profile['request_window'], 3, 'Request window')
    exact(profile['period_ms'], 20, 'Control period')
    timing = timing_selection(args)
    paired_native = profile.get('native_phase_pair', False)
    need(type(paired_native) is bool, 'Native phase pair must be an explicit boolean')
    if paired_native:
        exact(args.release_spin_us, 500, 'Native phase pair release spin')
    if args.execute_supported:
        exact(live.native_phase_pair_settings(profile), paired_native,
              'Loaded native phase pair selection')
    # Even an unapproved draft must authenticate its model candidate. No native
    # library, policy, audio, serial port or /proc file is opened by this plan.
    candidate = fk.plan(profile)
    checked_plan=None
    if profile.get('native_checked_policy_dispatch') is True:
        from singularitydog_hw import policy_checked_dispatch
        checked_plan=policy_checked_dispatch.plan(profile)
    _, library = pin(args.library, args.library_sha256, 'Active transport library')
    _, audio = pin(args.audio, args.audio_sha256, 'Start audio')
    wrappers = []
    for name in ('tools/dog_supported_fk_trial.py', 'tools/python_thread_switch_scope.py',
                 'tools/jetson_cpu_performance_scope.py'):
        wrappers.append(read_file(ROOT / name, 1024 * 1024)[1])
    for label, port in (('front', args.front_port), ('rear', args.rear_port)):
        need(type(port) is str and Path(port).is_absolute() and
             port.startswith('/dev/serial/by-path/') and '..' not in Path(port).parts,
             'Explicit stable USB ' + label + ' port required')
    need(args.front_port != args.rear_port, 'Distinct front/rear port names required')
    need(type(args.audio_device) is str and 0 < len(args.audio_device) <= 128 and
         args.audio_device.isprintable(), 'Explicit audio device required')
    output = private_output_root(args.output_root) / ('boxed-fk-' + str(uuid.uuid4()))
    need(not output.exists(), 'Fresh attempt directory required')
    return profile, {
        'schema': 'singularitydog.boxed-fk-foreground-plan.v1',
        'status': 'PLAN_ONLY', 'output_allowed': False,
        'profile_reviewed': profile['output_allowed'], 'blockers': profile['blockers'],
        'profile': profile_pin, 'target_fk_plan': candidate,
        'active_library': library, 'audio': audio, 'launcher_sources': wrappers,
        'scope': profile['scope'], 'duration_s': profile['duration_s'],
        'timing_selection': timing,
        'native_phase_pair': paired_native,
        'native_checked_policy_dispatch': profile.get('native_checked_policy_dispatch',False),
        'checked_model_plan': checked_plan,
        'expected_boot_id': profile['boot_id'], 'motor_power_epoch': args.power_epoch,
        'motor_power_epoch_inferred': False, 'physical_state_confirmed_by_plan': False,
        'hardware_opened': False, 'child_started': False,
        'output_directory': str(output), 'command': build_command(args, profile, output),
        'cpu_frequency_scope_applied_by_entry': False,
        'fresh_2_10_20_success_chain_required': True,
        'library_or_model_loaded_by_plan': False,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('launcher-sha256', 'profile', 'profile-sha256', 'front-port', 'rear-port', 'library',
                 'library-sha256', 'audio', 'audio-sha256', 'audio-device',
                 'output-root', 'power-epoch'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--execute-supported', action='store_true')
    parser.add_argument('--support-in-place', action='store_true')
    parser.add_argument('--cutoff-ready', action='store_true')
    parser.add_argument('--request-gap-us', type=request_gap_us, default=DEFAULT_REQUEST_GAP_US,
                        help='600..5000 us; must exactly match the selected profile (default: 900)')
    parser.add_argument('--release-spin-us', type=int, choices=(200, 500), default=DEFAULT_RELEASE_SPIN_US,
                        help='Pinned cancellation-aware active release wait (default: 200)')
    parser.add_argument('--timing-selection-reason',
                        help='Record why these timing values were selected; required for non-default values')
    args = parser.parse_args(argv)
    try:
        need(not (args.support_in_place or args.cutoff_ready) or args.execute_supported,
             'Physical execution flags belong only to --execute-supported')
        if args.execute_supported:
            need(args.support_in_place and args.cutoff_ready,
                 'Current box support and immediate cutoff confirmations required')
        profile, plan = prepare(args)
        print(json.dumps(plan, indent=2, ensure_ascii=False, allow_nan=False), flush=True)
        if not args.execute_supported:
            return 0
        exact(Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
              profile['boot_id'], 'Current Jetson boot')
        # Repeat all file/approval guards just before handing off. The existing
        # child repeats boot, profile, model, port ownership, feedback and STOP
        # checks; this wrapper never substitutes for those checks.
        profile_again, again = prepare(args)
        exact(profile_again['profile_sha256'], profile['profile_sha256'], 'Profile changed before child')
        for name in ('target_fk_plan', 'active_library', 'audio', 'launcher_sources',
                     'timing_selection', 'native_phase_pair',
                     'native_checked_policy_dispatch','checked_model_plan'):
            exact(again[name], plan[name], 'Binding changed before child: ' + name)
        print('箱を残します。学習目標を0.5%混ぜ、現在位置から最大1度の短い試験です。'
              '音声の開始合図を待ち、終了まで箱を抜かないでください。', flush=True)
        environment = dict(os.environ, PYTHONPATH=str(ROOT / 'runtime'), PYTHONDONTWRITEBYTECODE='1')
        for key in ('PYTHONHOME', 'PYTHONOPTIMIZE', 'PYTHONSTARTUP', 'PYTHONINSPECT', 'LD_PRELOAD'):
            environment.pop(key, None)
        # Keep the real terminal's stdin/stdout/stderr. No shell evaluation and
        # no /dev/tty open, so a CPU scope's foreground child also works.
        # Run in the existing foreground process, rather than making a second
        # process group that could escape an outer CPU-scope supervisor. The
        # policy runner retains its own signal handling and finally STOP.
        previous_environment = dict(os.environ)
        previous_hup = signal.getsignal(signal.SIGHUP)
        def hangup(number, frame):
            # The active runner's TERM handler requests cancellation and STOP.
            # Before that handler exists no motors have been enabled, and an
            # interrupt still unwinds its normal setup/finally path.
            handler = signal.getsignal(signal.SIGTERM)
            if callable(handler):
                handler(signal.SIGTERM, frame)
            else:
                raise KeyboardInterrupt('Terminal hangup before active cancellation setup')
        try:
            os.environ.clear(); os.environ.update(environment)
            signal.signal(signal.SIGHUP, hangup)
            run_switched_module('singularitydog_hw.policy_output',
                                plan['command'][8:], interval_us=100)
        finally:
            signal.signal(signal.SIGHUP, previous_hup)
            os.environ.clear(); os.environ.update(previous_environment)
        return 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    raise SystemExit(main())
