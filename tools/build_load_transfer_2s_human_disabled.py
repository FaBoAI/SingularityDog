"""Freeze an offline, disabled 2 s check for a future stand-removed capture.

The reviewed r8 motor runtime is copied byte for byte. This tool never opens a
UART, transfers files, or runs the generated package. The known center-stand
r1/r2 captures are ineligible even if a physical report labels them otherwise.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
from types import ModuleType

from human_supported_capture_guard import require, sha, validate_capture
from load_transfer_2s_preflight_wrapper import (
    BOOT, EVIDENCE as PRIOR_EVIDENCE, SOURCES,
)


ROOT = Path(__file__).resolve().parents[1]
GUARD = Path(__file__).resolve().with_name('human_supported_capture_guard.py')
CAPTURE_SOURCE = ROOT / 'runtime/singularitydog_hw/fixed_stance_readonly_capture.py'
REVIEWED_PACKAGE_MANIFEST_SHA256 = '2cf7b37ac82fc4f28e63d75a10eef4af9ecb76d4defc7ba138caef51dcd59167'
REVIEWED_WRAPPER_SHA256 = 'ee01d3a4131dd99313bc3f9346a6e5b153f2b9121311db6f2e2fb8751757d9d2'
KNOWN_CENTER_STAND_R2_SHA256 = 'b43b279ee44d3d13f8b962863b77a8b44567d267f95fe7c08bc727e4665ccf60'
NEW_EVIDENCE = (
    'human-capture-summary.json', 'human-capture-events.jsonl',
    'human-capture-draft.json', 'human-capture-source.py',
    'human-physical-review.json',
)


def replace_one(source, before, after):
    require(source.count(before) == 1, 'Reviewed disabled wrapper anchor differs: ' + before[:72])
    return source.replace(before, after)


def render_wrapper(source, remote_package_dir):
    """Change only file/capture/operator gates in the exact reviewed wrapper."""
    require(hashlib.sha256(source.encode()).hexdigest() == REVIEWED_WRAPPER_SHA256,
            'Disabled r8 wrapper is not the exact reviewed source')
    remote = Path(remote_package_dir)
    require(remote.is_absolute()
            and remote.parent == Path('/home/jetson/singularitydog-tests')
            and remote.name.startswith('load-transfer-2s-human-disabled-')
            and all(char.isalnum() or char in '-_' for char in remote.name),
            'Use a new fixed Jetson human-disabled package path')
    source = replace_one(source,
        '"""Frozen two-USB 80 ms workload preflight; every motor output remains disabled.\n\n'
        'Only STOP, parameter reads, volatile watchdog writes and zero-gain frames may\n'
        'reach either UART. This wrapper has no active branch. Its source directory and\n'
        'current-boot evidence must be SHA-pinned before placing it on the Jetson.\n"""',
        '"""Two-second, twelve-axis disabled check at a captured human-supported pose.\n\n'
        'Two people fully support the torso while the stand is absent. Only STOP,\n'
        'parameter reads, volatile watchdog writes and zero-gain frames may reach\n'
        'either UART. This wrapper has no Enable or active branch.\n"""')
    source = replace_one(source,
        "BASE = Path('/home/jetson/singularitydog-tests/load-transfer-2s-preflight-r8')",
        'BASE = Path(' + repr(str(remote)) + ')')
    source = replace_one(source,
        "            'operator-rehearsal.json')",
        "            'operator-rehearsal.json',\n"
        "            'human-capture-summary.json', 'human-capture-events.jsonl',\n"
        "            'human-capture-draft.json', 'human-capture-source.py',\n"
        "            'human-physical-review.json')")
    source = replace_one(source,
        "{'prepared_load_transfer.py', 'preflight-review.json'}",
        "{'prepared_load_transfer.py', 'preflight-review.json', "
        "'human_supported_capture_guard.py'}")
    source = replace_one(source,
        "    return review\n\n\ndef main(argv=None):",
        "    require(review.get('source_disabled_manifest_sha256') ==\n"
        "            '" + REVIEWED_PACKAGE_MANIFEST_SHA256 + "'\n"
        "            and review.get('supported_stance_passed') is False\n"
        "            and review.get('human_full_support_required') is True\n"
        "            and review.get('stand_absent_required') is True,\n"
        "            'Human-supported disabled review gate differs')\n"
        "    guard_path = base / 'human_supported_capture_guard.py'\n"
        "    guard = type(sys)('frozen_human_capture_guard')\n"
        "    guard.__file__ = str(guard_path)\n"
        "    exec(compile(guard_path.read_bytes(), str(guard_path), 'exec'),\n"
        "         guard.__dict__)\n"
        "    guard.verify_package_capture(base, review, manifest, BOOT)\n"
        "    return review\n\n\ndef main(argv=None):")
    source = replace_one(source,
        "    parser.add_argument('--supported', action='store_true', required=True)",
        "    parser.add_argument('--human-supported', action='store_true', required=True)\n"
        "    parser.add_argument('--stand-absent', action='store_true', required=True)\n"
        "    parser.add_argument('--two-operators-full-support', action='store_true', required=True)\n"
        "    parser.add_argument('--paws-floor', action='store_true', required=True)\n"
        "    parser.add_argument('--cutoff-operator-ready', action='store_true', required=True)")
    source = replace_one(source,
        "    if (not args.preflight_only or not args.supported or not args.output.is_absolute()",
        "    if (not all((args.preflight_only, args.human_supported, args.stand_absent,\n"
        "                 args.two_operators_full_support, args.paws_floor,\n"
        "                 args.cutoff_operator_ready)) or not args.output.is_absolute()")
    source = replace_one(source,
        "        parser.error('Use disabled supported preflight and a fresh pinned log directory')",
        "        parser.error('Use disabled human-supported preflight and a fresh pinned log directory')")
    source = replace_one(source,
        "              'motor_enable_sent': False, 'motion_gain_sent': False,",
        "              'human_supported': True, 'stand_absent': True,\n"
        "              'motor_enable_sent': False, 'motion_gain_sent': False,")
    return source


def _load_frozen_wrapper(path):
    module = ModuleType('frozen_human_disabled_preflight')
    module.__file__ = str(path)
    exec(compile(path.read_bytes(), str(path), 'exec'), module.__dict__)
    return module


def build(source_package, capture_summary, capture_events, capture_draft,
          capture_source, physical_review, output, remote_package_dir):
    (source_package, capture_summary, capture_events, capture_draft,
     capture_source, physical_review, output) = map(Path, (
         source_package, capture_summary, capture_events, capture_draft,
         capture_source, physical_review, output))
    require(not output.exists() and not output.is_symlink() and output.parent.is_dir()
            and not output.parent.is_symlink(), 'Use a fresh private package directory')
    require(not any((parent / '.git').exists() for parent in (output, *output.parents)),
            'Raw pose and motor identities must stay outside Git')
    require(sha(source_package / 'manifest.json') == REVIEWED_PACKAGE_MANIFEST_SHA256
            and sha(source_package / 'prepared_load_transfer.py') == REVIEWED_WRAPPER_SHA256,
            'Disabled source package is not the reviewed r8 release')
    # The old bundle is independently checked before any new review can be made.
    original = _load_frozen_wrapper(source_package / 'prepared_load_transfer.py')
    old_review = original.verify_files(source_package, REVIEWED_PACKAGE_MANIFEST_SHA256)
    require(sha(capture_summary) != KNOWN_CENTER_STAND_R2_SHA256,
            'Center-stand r2 is not a human-supported capture')
    require(sha(capture_source) == sha(CAPTURE_SOURCE),
            'Capture source differs from reviewed local collector')
    source_manifest = json.loads((source_package / 'manifest.json').read_text())
    raw = validate_capture(
        capture_summary, capture_events, capture_draft, capture_source,
        physical_review, boot=BOOT, uids=old_review['motor_uids'],
        can_readonly_sha256=source_manifest['singularitydog_hw/can_readonly.py'])
    wrapper = render_wrapper(
        (source_package / 'prepared_load_transfer.py').read_text(), remote_package_dir)
    review = dict(old_review)
    review.update(
        sha256=sha(capture_summary),
        supported_floor_start_raw_rad_by_id=raw,
        supported_stance_passed=False,
        human_full_support_required=True,
        stand_absent_required=True,
        human_capture_summary_sha256=sha(capture_summary),
        human_capture_events_sha256=sha(capture_events),
        human_capture_draft_sha256=sha(capture_draft),
        human_capture_source_sha256=sha(capture_source),
        human_physical_review_sha256=sha(physical_review),
        source_disabled_manifest_sha256=REVIEWED_PACKAGE_MANIFEST_SHA256,
        review_note=('Disabled 2 s, 12-axis preflight at a fresh stand-removed, '
                     'fully human-supported pose. The r8 runtime is unchanged. '
                     'This is an electrical/feedback/STOP check, not a hold, '
                     'load-transfer or self-standing result.'))
    os.umask(0o077)
    output.mkdir(mode=0o700)
    (output / 'singularitydog_hw').mkdir(mode=0o700)
    (output / 'evidence').mkdir(mode=0o700)
    for name in SOURCES:
        shutil.copyfile(source_package / 'singularitydog_hw' / name,
                        output / 'singularitydog_hw' / name)
    for name in PRIOR_EVIDENCE:
        shutil.copyfile(source_package / 'evidence' / name, output / 'evidence' / name)
    evidence_paths = dict(zip(NEW_EVIDENCE, (capture_summary, capture_events,
                                            capture_draft, capture_source,
                                            physical_review)))
    for name, path in evidence_paths.items():
        shutil.copyfile(path, output / 'evidence' / name)
    shutil.copyfile(GUARD, output / 'human_supported_capture_guard.py')
    (output / 'prepared_load_transfer.py').write_text(wrapper)
    (output / 'preflight-review.json').write_text(
        json.dumps(review, indent=2, allow_nan=False) + '\n')
    manifest = {str(path.relative_to(output)): sha(path)
                for path in sorted(output.rglob('*')) if path.is_file()}
    (output / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    for path in output.rglob('*'):
        path.chmod(0o700 if path.is_dir() else 0o600)
    digest = sha(output / 'manifest.json')
    frozen = _load_frozen_wrapper(output / 'prepared_load_transfer.py')
    frozen.verify_files(output, digest)
    return {'status': 'HUMAN_DISABLED_PACKAGE_FROZEN_NOT_RUN',
            'package': str(output), 'manifest_sha256': digest,
            'boot_id': BOOT, 'duration_s': 2., 'motor_output_sent': False,
            'capture_summary_sha256': sha(capture_summary)}


def validate_disabled_run(package, summary, events, manifest_sha256):
    """Check the future Jetson result before an active package can use it."""
    package, summary, events = map(Path, (package, summary, events))
    require(package.is_dir() and not package.is_symlink()
            and (package / 'manifest.json').is_file()
            and not (package / 'manifest.json').is_symlink()
            and sha(package / 'manifest.json') == manifest_sha256,
            'Disabled package manifest differs from the separately pinned SHA')
    manifest = json.loads((package / 'manifest.json').read_text())
    require(sha(package / 'prepared_load_transfer.py') ==
            manifest.get('prepared_load_transfer.py'),
            'Disabled wrapper differs from the pinned package manifest')
    require(summary.is_file() and not summary.is_symlink()
            and events.is_file() and not events.is_symlink(),
            'Disabled trial logs are missing or symlinked')
    frozen = _load_frozen_wrapper(package / 'prepared_load_transfer.py')
    review = frozen.verify_files(package, manifest_sha256)
    result = json.loads(summary.read_text())
    trial = result.get('result') or {}
    require(result.get('boot_id') == BOOT
            and result.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and result.get('preflight_only') is True
            and result.get('human_supported') is True
            and result.get('stand_absent') is True
            and result.get('errors') == [] and result.get('signals') == []
            and result.get('motor_enable_sent') is False
            and result.get('motion_gain_sent') is False
            and result.get('trial_device_closed') is True
            and result.get('locks_released') is True
            and result.get('wrapper_sha256') == manifest['prepared_load_transfer.py']
            and result.get('events_sha256') == sha(events)
            and trial.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and trial.get('preflight_only') is True
            and trial.get('preflight_completed') is True
            and trial.get('motion_completed') is False
            and trial.get('live_output') is False
            and trial.get('duration_s') == 2.
            and trial.get('requested_ticks') == 25
            and trial.get('stop_confirmed') is True
            and trial.get('errors') == []
            and trial.get('review') == review,
            'Disabled trial result is incomplete or differs from the frozen capture')
    workers = trial.get('workers') or {}
    require(set(workers) == {'front', 'rear'}, 'Disabled trial needs both bus reports')
    leg_ids = {'FR': {'1', '2', '3'}, 'FL': {'4', '5', '6'},
               'RR': {'7', '8', '9'}, 'RL': {'10', '11', '12'}}
    for bus, legs in (('front', {'FR', 'FL'}), ('rear', {'RR', 'RL'})):
        worker = workers[bus]
        windows = worker.get('settled_windows') or {}
        stops = worker.get('stop_reports') or {}
        initial_stops = worker.get('initial_stop') or {}
        electrical = worker.get('electrical_samples') or []
        ids = set().union(*(leg_ids[leg] for leg in legs))
        centers = worker.get('centers') or {}
        require(set(centers) == ids
                and all(type(center) in (int, float) and math.isfinite(center)
                        and (abs(center - review['supported_floor_start_raw_rad_by_id'][mid])
                             <= math.radians(3.) or
                             (mid in {'3', '9'}
                              and abs(abs(center - review['supported_floor_start_raw_rad_by_id'][mid])
                                      - 2. * math.pi) <= math.radians(3.)))
                        for mid, center in centers.items()),
                f'Disabled {bus} centers differ from the frozen capture')
        require(worker.get('completed') is True and worker.get('errors') == []
                and worker.get('cycle_count') == 25
                and len(electrical) == 25
                and all(type(row.get('voltage_v')) in (int, float)
                        and math.isfinite(row['voltage_v'])
                        and 35. <= row['voltage_v'] <= 43.
                        and row.get('watchdog_ticks') == 4000 for row in electrical)
                and set(windows) == legs
                and all(set(window.get('motors') or {}) == leg_ids[leg]
                        and window.get('passed') is True
                        and window.get('errors') == []
                        and window.get('sample_count_required') == 21
                        and all(motor.get('sample_count') == 21
                                and len(motor.get('samples') or []) == 21
                                and motor.get('errors') == []
                                for motor in window['motors'].values())
                        for leg, window in windows.items())
                and set(initial_stops) == ids
                and all(stop.get('confirmed') is True
                        for stop in initial_stops.values())
                and set(stops) == ids
                and all(stop.get('confirmed') is True for stop in stops.values()),
                f'Disabled {bus} cycle, static-window, electrical or STOP gate failed')
    counts = Counter()
    for line in events.read_text().splitlines():
        row = json.loads(line)
        if row.get('kind') != 'can_tx':
            continue
        wire = bytes.fromhex(row['hex'])
        require(len(wire) == 17 and wire[:2] == b'AT' and wire[-2:] == b'\r\n',
                'Malformed disabled CAN transmit event')
        kind = (int.from_bytes(wire[2:6], 'big') >> 27) & 31
        require(kind in (0, 1, 4, 17, 18)
                and (kind != 1 or wire[11:15] == bytes(4)),
                'Disabled trial emitted Enable, nonzero gain or unexpected CAN')
        counts[str(kind)] += 1
    require(dict(counts) == result.get('typed_tx_counts')
            and counts['4'] >= 12 and counts['1'] > 0,
            'Disabled CAN transmit counts or STOP/zero-gain events differ')
    return {'status': 'HUMAN_DISABLED_PREFLIGHT_VERIFIED',
            'boot_id': BOOT, 'package_manifest_sha256': manifest_sha256,
            'capture_summary_sha256': review['human_capture_summary_sha256'],
            'physical_review_sha256': review['human_physical_review_sha256'],
            'summary_sha256': sha(summary), 'events_sha256': sha(events)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source-package', 'capture-summary', 'capture-events',
                 'capture-draft', 'capture-source', 'physical-review', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--remote-package-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(build(args.source_package, args.capture_summary,
                           args.capture_events, args.capture_draft,
                           args.capture_source, args.physical_review,
                           args.output, args.remote_package_dir), indent=2), flush=True)


if __name__ == '__main__':
    main()
