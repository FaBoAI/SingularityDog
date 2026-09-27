"""Freeze, offline, a 2 s fully human-supported current-position hold package.

The source is the exact reviewed supported-active r2 package. This builder
never opens hardware, uploads files, or modifies any prior package. A failed
read-only capture is ineligible even if the position spans look small.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

from human_supported_capture_guard import (read, require, sha,
                                           validate_capture, verify_disabled_result)
from load_transfer_2s_active_wrapper import (BOOT, EVIDENCE as PRIOR_EVIDENCE,
                                              SOURCES, verify_files as verify_prior)


ROOT = Path(__file__).resolve().parents[1]
WRAPPER = Path(__file__).resolve().with_name('load_transfer_2s_active_wrapper.py')
GUARD = Path(__file__).resolve().with_name('human_supported_capture_guard.py')
REVIEWED_SOURCE_MANIFEST_SHA256 = 'd07e596a6782f8c2d7a360c1e4744734fa92fb76eb523e43550ebe2c6b971ffc'
REVIEWED_WRAPPER_SHA256 = '3bbc8323c03fb96acca0ed6e783f97cf0aecca8701fb184d94d71e86cbff6ff5'
NEW_EVIDENCE = (
    'human-capture-summary.json', 'human-capture-events.jsonl',
    'human-capture-draft.json', 'human-capture-source.py',
    'human-physical-review.json',
    'human-disabled-summary.json', 'human-disabled-events.jsonl',
    'human-disabled-manifest.json',
)


def validate_disabled_package(package, expected_manifest_sha256, summary_path,
                              events_path, *, boot, uids, raw, capture_sha256,
                              physical_sha256, capture_end_ns):
    """Require a separate new-pose disabled 25-cycle/all12-STOP result."""
    from build_load_transfer_2s_human_disabled import validate_disabled_run
    package = Path(package)
    result = validate_disabled_run(package, summary_path, events_path,
                                   expected_manifest_sha256)
    require(result.get('status') == 'HUMAN_DISABLED_PREFLIGHT_VERIFIED'
            and result.get('boot_id') == boot
            and result.get('capture_summary_sha256') == capture_sha256
            and result.get('physical_review_sha256') == physical_sha256,
            'Disabled run differs from new capture or physical review')
    disabled_manifest = read(package / 'manifest.json')
    disabled_review = read(package / 'preflight-review.json')
    require(disabled_review.get('boot_id') == boot
            and disabled_review.get('duration_s') == 2.
            and disabled_review.get('motor_uids') == uids
            and disabled_review.get('supported_floor_start_raw_rad_by_id') == raw
            and disabled_review.get('human_capture_summary_sha256') == capture_sha256
            and disabled_review.get('human_physical_review_sha256') == physical_sha256
            and disabled_review.get('human_full_support_required') is True
            and disabled_review.get('stand_absent_required') is True
            and disabled_review.get('supported_stance_passed') is False,
            'Disabled package is not bound to the newly reviewed human-supported pose')
    require(sha(package / 'manifest.json') == expected_manifest_sha256,
            'Disabled package manifest differs from explicit pin')
    # The capture and disabled events use the same monotonic clock on this boot.
    events_path = Path(events_path)
    latest_ns = capture_end_ns
    for line in events_path.read_text().splitlines():
        require(bool(line.strip()), 'Blank disabled event')
        event = json.loads(line)
        event_ns = event.get('monotonic_ns')
        require(type(event_ns) is int and event_ns > capture_end_ns,
                'Disabled result predates the new capture')
        latest_ns = max(latest_ns, event_ns)
    require(latest_ns > capture_end_ns, 'Disabled event log is empty')
    return disabled_manifest


def replace_one(source, before, after):
    require(source.count(before) == 1, 'Reviewed wrapper patch anchor differs: ' + before[:70])
    return source.replace(before, after)


def render_wrapper(source, remote_package_dir):
    """Keep the active r2 wire/runner code, changing only package/evidence gates."""
    require(hashlib.sha256(source.encode()).hexdigest() == REVIEWED_WRAPPER_SHA256,
            'Supported-active wrapper is not the exact reviewed source')
    remote = Path(remote_package_dir)
    require(remote.is_absolute() and remote.parent == Path('/home/jetson/singularitydog-tests')
            and remote.name.startswith('load-transfer-2s-human-supported-')
            and all(c.isalnum() or c in '-_' for c in remote.name),
            'Remote package path must be a new human-supported test directory')
    source = replace_one(source,
        "BASE = Path('/home/jetson/singularitydog-tests/load-transfer-2s-supported-active-r2')",
        'BASE = Path(' + repr(str(remote)) + ')')
    source = replace_one(source,
        "            'supported-r1-manifest.json')",
        "            'supported-r1-manifest.json',\n"
        "            'human-capture-summary.json', 'human-capture-events.jsonl',\n"
        "            'human-capture-draft.json', 'human-capture-source.py',\n"
        "            'human-physical-review.json',\n"
        "            'human-disabled-summary.json', 'human-disabled-events.jsonl',\n"
        "            'human-disabled-manifest.json')")
    source = replace_one(source,
        "{'prepared_load_transfer.py', 'review.json'}",
        "{'prepared_load_transfer.py', 'review.json', 'human_supported_capture_guard.py'}")
    source = replace_one(source,
        "            and review.get('stand_fully_supporting_required') is True\n"
        "            and review.get('partial_load_allowed') is False",
        "            and review.get('stand_fully_supporting_required') is False\n"
        "            and review.get('partial_load_allowed') is False\n"
        "            and review.get('stand_removed_only_with_40v_off') is True\n"
        "            and review.get('two_operators_full_support_required') is True\n"
        "            and review.get('no_load_easing_required') is True\n"
        "            and review.get('full_support_through_stop_required') is True")
    source = replace_one(source,
        "    return review\n\n\ndef main(argv=None):",
        "    from human_supported_capture_guard import verify_package_capture\n"
        "    verify_package_capture(base, review, manifest, BOOT)\n"
        "    from human_supported_capture_guard import verify_disabled_result\n"
        "    verify_disabled_result(base, review, manifest, BOOT)\n"
        "    return review\n\n\ndef main(argv=None):")
    source = replace_one(source,
        "    parser.add_argument('--stand-fully-supporting', action='store_true', required=True)",
        "    parser.add_argument('--stand-removed-under-40v-off', action='store_true', required=True)\n"
        "    parser.add_argument('--two-operators-full-support', action='store_true', required=True)\n"
        "    parser.add_argument('--no-load-easing', action='store_true', required=True)\n"
        "    parser.add_argument('--support-through-stop', action='store_true', required=True)")
    source = replace_one(source,
        "    if (not all((args.active, args.stand_fully_supporting, args.paws_floor,",
        "    if (not all((args.active, args.stand_removed_under_40v_off,\n"
        "                 args.two_operators_full_support, args.no_load_easing,\n"
        "                 args.support_through_stop, args.paws_floor,")
    source = replace_one(source,
        "              'supported_diagnostic_only': True, 'partial_load_allowed': False,",
        "              'supported_diagnostic_only': True, 'partial_load_allowed': False,\n"
        "              'stand_removed_under_40v_off': True,\n"
        "              'two_operators_full_support_required': True,\n"
        "              'no_load_easing_required': True,\n"
        "              'full_support_through_stop_required': True,")
    source = replace_one(source,
        "The fixed stand must fully support the torso throughout. Two operators keep\n"
        "continuous catch and a physical 40 V cutoff. An external I2S wrapper must\n"
        "finish its Japanese announcement first. No stand removal or load shift is\n"
        "allowed in this run. The program never retries and never claims stance.",
        "The stand is removed only while 40 V is Off. Two operators fully support\n"
        "the torso through the complete hold and all12 STOP; no easing of support.\n"
        "A physical cutoff remains ready. This never claims self-standing.")
    return source


def build(source_package, capture_summary, capture_events, capture_draft,
          capture_source, physical_review, disabled_package,
          disabled_manifest_sha256, disabled_summary, disabled_events,
          output, remote_package_dir):
    paths = list(map(Path, (source_package, capture_summary, capture_events,
                            capture_draft, capture_source, physical_review,
                            disabled_package, disabled_summary, disabled_events,
                            output)))
    (source_package, capture_summary, capture_events, capture_draft,
     capture_source, physical_review, disabled_package, disabled_summary,
     disabled_events, output) = paths
    require(not output.exists() and not output.is_symlink() and output.parent.is_dir()
            and not output.parent.is_symlink(), 'Use a fresh private package directory')
    require(not any((parent / '.git').exists() for parent in (output, *output.parents)),
            'Raw pose and motor identities must stay outside Git')
    require(sha(source_package / 'manifest.json') == REVIEWED_SOURCE_MANIFEST_SHA256,
            'Supported-active source package manifest is not the reviewed r2 release')
    old_review = verify_prior(source_package, REVIEWED_SOURCE_MANIFEST_SHA256)
    require(sha(WRAPPER) == REVIEWED_WRAPPER_SHA256
            and sha(source_package / 'prepared_load_transfer.py') == REVIEWED_WRAPPER_SHA256,
            'Reviewed wrapper source changed')
    require(sha(capture_source) == sha(ROOT / 'runtime/singularitydog_hw/fixed_stance_readonly_capture.py'),
            'Capture source differs from reviewed local collector')
    source_manifest = json.loads((source_package / 'manifest.json').read_text())
    raw = validate_capture(
        capture_summary, capture_events, capture_draft, capture_source,
        physical_review, boot=BOOT, uids=old_review['motor_uids'],
        can_readonly_sha256=source_manifest['singularitydog_hw/can_readonly.py'])
    disabled_manifest = validate_disabled_package(
        disabled_package, disabled_manifest_sha256, disabled_summary,
        disabled_events, boot=BOOT, uids=old_review['motor_uids'], raw=raw,
        capture_sha256=sha(capture_summary), physical_sha256=sha(physical_review),
        capture_end_ns=read(capture_summary)['pose']['ended_monotonic_ns'])
    active_source = (source_package / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text()
    require(active_source.count('LIVE_OUTPUT_ENABLED = True') == 1
            and sha(disabled_package / 'singularitydog_hw/rs05_load_transfer_hold.py') ==
                hashlib.sha256(active_source.replace('LIVE_OUTPUT_ENABLED = True',
                                                       'LIVE_OUTPUT_ENABLED = False').encode()).hexdigest()
            and all(source_manifest['singularitydog_hw/' + name] ==
                    disabled_manifest['singularitydog_hw/' + name]
                    for name in SOURCES if name != 'rs05_load_transfer_hold.py'),
            'Disabled and active runtimes differ beyond the exact source gate')
    wrapper = render_wrapper(WRAPPER.read_text(), remote_package_dir)
    review = dict(old_review)
    review.update(
        sha256=sha(capture_summary),
        supported_floor_start_raw_rad_by_id=raw,
        stand_fully_supporting_required=False,
        partial_load_allowed=False,
        continuous_human_support_required=True,
        stand_removed_only_with_40v_off=True,
        two_operators_full_support_required=True,
        no_load_easing_required=True,
        full_support_through_stop_required=True,
        human_capture_summary_sha256=sha(capture_summary),
        human_capture_events_sha256=sha(capture_events),
        human_capture_draft_sha256=sha(capture_draft),
        human_capture_source_sha256=sha(capture_source),
        human_physical_review_sha256=sha(physical_review),
        human_disabled_manifest_sha256=disabled_manifest_sha256,
        human_disabled_summary_sha256=sha(disabled_summary),
        human_disabled_events_sha256=sha(disabled_events),
        source_supported_active_manifest_sha256=REVIEWED_SOURCE_MANIFEST_SHA256,
        review_note=('Two-second current-position hold at a newly captured same-boot '
                     'human-supported pose. Two operators fully support the torso '
                     'through all12 STOP, without easing or shifting load. The earlier '
                     'supported-active r2 hold validates this exact runtime and '
                     'monitor set, not this new pose. A separate same-boot, '
                     'new-pose disabled 2 s trial passed all12 STOP before freezing. '
                     'Fresh 21-sample pre-Enable stability, ±3-degree raw start, Type2 feedback, '
                     'fault/torque/voltage/watchdog guards, and all12 STOP remain '
                     'mandatory at execution. No autonomous standing is claimed.'))
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
                                            physical_review, disabled_summary,
                                            disabled_events,
                                            disabled_package / 'manifest.json')))
    for name, path in evidence_paths.items():
        shutil.copyfile(path, output / 'evidence' / name)
    shutil.copyfile(GUARD, output / 'human_supported_capture_guard.py')
    (output / 'prepared_load_transfer.py').write_text(wrapper)
    (output / 'review.json').write_text(json.dumps(review, indent=2, allow_nan=False) + '\n')
    manifest = {str(path.relative_to(output)): sha(path)
                for path in sorted(output.rglob('*')) if path.is_file()}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    for path in output.rglob('*'):
        path.chmod(0o700 if path.is_dir() else 0o600)
    digest = sha(output / 'manifest.json')
    # Importing the generated wrapper is safe: its main() has a standard guard.
    import importlib.util
    spec = importlib.util.spec_from_file_location('frozen_human_supported',
                                                output / 'prepared_load_transfer.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.verify_files(output, digest)
    return {'status': 'HUMAN_SUPPORTED_PACKAGE_FROZEN_NOT_RUN',
            'package': str(output), 'manifest_sha256': digest,
            'boot_id': BOOT, 'duration_s': 2., 'motor_output_sent': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source-package', 'capture-summary', 'capture-events',
                 'capture-draft', 'capture-source', 'physical-review',
                 'disabled-package', 'disabled-summary', 'disabled-events', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--disabled-manifest-sha256', required=True)
    parser.add_argument('--remote-package-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    result = build(args.source_package, args.capture_summary, args.capture_events,
                   args.capture_draft, args.capture_source, args.physical_review,
                   args.disabled_package, args.disabled_manifest_sha256,
                   args.disabled_summary, args.disabled_events,
                   args.output, args.remote_package_dir)
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
