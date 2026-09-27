"""Frozen two-USB, two-second supervised partial-load diagnostic.

Remove the stand only while 40 V is Off. Two operators fully support the torso
through preparation, Enable, and the confirmed first active cycle. Only after
the printed cue may support be eased slightly for at most 0.5 s; restore
full support before the scheduled STOP and catch through STOP. The cutoff
operator remains able to turn off 40 V. This never claims self-standing.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

BASE = Path('/home/jetson/singularitydog-tests/load-transfer-2s-partial-r5')
CURRENT = Path('/home/jetson/singularitydog-tests/fullbody-active-20260927-r1')
BOOT = '5662ee00-b2f5-4913-9bfd-33ae39642427'
SOURCES = ('__init__.py', 'bounded_pose_plan.py', 'can_readonly.py',
           'current_hold_review.py', 'position_response_evidence.py',
           'rs05_bus_transport.py', 'rs05_joint_trial.py', 'rs05_leg_trial.py',
           'rs05_load_transfer_hold.py', 'rs05_trial_protocol.py')
EVIDENCE = ('floor-summary.json', 'floor-events.jsonl', 'readonly-summary.json',
            'operator-rehearsal.json', 'disabled-r8-summary.json',
            'disabled-r8-events.jsonl', 'disabled-r8-manifest.json',
            'supported-r1-summary.json', 'supported-r1-events.jsonl',
            'supported-r1-manifest.json', 'supported-r2-summary.json',
            'supported-r2-events.jsonl', 'supported-r2-manifest.json',
            'supported-r2-runtime.py', 'partial-r3-summary.json',
            'partial-r3-events.jsonl', 'partial-r3-manifest.json',
            'partial-r4-summary.json', 'partial-r4-events.jsonl',
            'partial-r4-manifest.json', 'box-baseline-summary.json',
            'box-baseline-events.jsonl', 'box-baseline-draft.json',
            'box-baseline-source.py', 'box-baseline-operator-report.json',
            'box-supported-summary.json', 'box-supported-events.jsonl',
            'box-supported-manifest.json')

# Reversible, one-file delta from the successful supported-r2 runtime. No
# shared stationarity profile, active feedback guard, or final-hold gate changes.
SOURCE_HELPER_ANCHOR = '\ndef static_hold_error_limit('
SOURCE_CALL_ANCHOR = "                report['settled_windows'][leg] = evaluation\n"
SOURCE_CALL_PATCH = (
    "                evaluation = reviewed_human_supported_id3_window(evaluation, review, leg)\n"
    + SOURCE_CALL_ANCHOR)
SOURCE_HELPER = '''

def reviewed_human_supported_id3_window(evaluation, review, leg):
    """One pre-Enable full-window range exception; retain every other gate."""
    if (leg != 'FR' or evaluation.get('profile') != FULLBODY_POSITION_PROFILE
            or review.get('partial_load_allowed') is not True
            or review.get('stand_removed_only_with_40v_off') is not True
            or review.get('full_support_through_tick0') is not True
            or review.get('partial_pre_enable_id3_full_window_range_rad') != .003
            or evaluation.get('limits', {}).get('position_range_rad') != .001):
        return evaluation
    entry = evaluation['motors'][3]
    evaluation['partial_pre_enable_id3_full_window_range_limit_rad'] = .003
    evaluation['limits_by_motor'] = {
        mid: {**evaluation['limits'], 'position_range_rad': .003 if mid == 3 else .001}
        for mid in evaluation['motors']}
    error = 'position_range_rad exceeds 0.001'
    position_range = entry.get('position_range_rad')
    if (type(position_range) in (int, float) and math.isfinite(position_range)
            and .001 < position_range <= .003 and error in entry['errors']
            and 'ID3: ' + error in evaluation['errors']):
        entry['errors'].remove(error)
        evaluation['errors'].remove('ID3: ' + error)
        evaluation['passed'] = not evaluation['errors']
        evaluation['partial_pre_enable_id3_range_exception_applied'] = True
    return evaluation
'''


def patch_partial_source(source):
    require(source.count(SOURCE_HELPER_ANCHOR) == 1
            and source.count(SOURCE_CALL_ANCHOR) == 1
            and 'reviewed_human_supported_id3_window' not in source,
            'Supported source patch anchors differ')
    return (source.replace(SOURCE_HELPER_ANCHOR,
                           SOURCE_HELPER + SOURCE_HELPER_ANCHOR)
            .replace(SOURCE_CALL_ANCHOR, SOURCE_CALL_PATCH))


def unpatch_partial_source(source):
    require(source.count(SOURCE_HELPER + SOURCE_HELPER_ANCHOR) == 1
            and source.count(SOURCE_CALL_PATCH) == 1,
            'Partial runtime has an unreviewed source delta')
    return (source.replace(SOURCE_CALL_PATCH, SOURCE_CALL_ANCHOR)
            .replace(SOURCE_HELPER + SOURCE_HELPER_ANCHOR, SOURCE_HELPER_ANCHOR))


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def partial_cue_deadline(cycle_events, now):
    """Return an absolute re-support time only for a fresh dual-bus tick0."""
    by_bus = {row['bus']: row for row in cycle_events
              if row.get('kind') == 'load_transfer_cycle' and row.get('tick') == 0
              and row.get('bus') in ('front', 'rear')}
    if set(by_bus) != {'front', 'rear'}:
        return None
    starts = [by_bus[bus]['due_monotonic_s'] for bus in ('front', 'rear')]
    completed = [by_bus[bus]['completed_monotonic_s'] for bus in ('front', 'rear')]
    start = min(starts)
    if (max(starts) - start > .001 or now < max(completed)
            or now - max(completed) > .1 or now > start + .2):
        return None
    return min(now + .5, start + .7)


def announce_partial_window(first_cycle_confirmed, runner_finished,
                            active_stop_started, cue_opened,
                            events, emit_lock, cue_lock, *, clock=time.monotonic,
                            writer=print):
    """Cue a short ease only after timely tick0 evidence; cue re-support on exit."""
    while True:
        if runner_finished.is_set() or active_stop_started.is_set():
            writer('NO_PARTIAL_LOAD RE_SUPPORT_NOW', flush=True)
            return
        if first_cycle_confirmed.wait(.01):
            with emit_lock:
                cycle0 = tuple(row for row in events
                               if row.get('kind') == 'load_transfer_cycle'
                               and row.get('tick') == 0)
            with cue_lock:
                deadline = partial_cue_deadline(cycle0, clock())
                if (runner_finished.is_set() or active_stop_started.is_set()
                        or deadline is None):
                    writer('NO_PARTIAL_LOAD RE_SUPPORT_NOW', flush=True)
                    return
                writer('FIRST_HOLD_CYCLE_CONFIRMED PARTIAL_LOAD_WINDOW_OPEN', flush=True)
                cue_opened.set()
            while (clock() < deadline and not runner_finished.is_set()
                   and not active_stop_started.is_set()):
                runner_finished.wait(min(.01, max(0., deadline - clock())))
            writer('RE_SUPPORT_NOW', flush=True)
            return


class ExactWirePort:
    """Final pre-write gate for this fixed current-position diagnostic."""

    def __init__(self, raw, ids, review, *, parser_type, read_request, protocol,
                 active_stop_started):
        self.raw, self.ids, self.review = raw, tuple(ids), review
        self.parser_type, self.read_request, self.protocol = parser_type, read_request, protocol
        self.active_stop_started = active_stop_started
        self.enable_seen = False
        self.active_targets = {}
        self.center_target_bytes = {}

    def _floor_equivalent(self, raw, mid):
        reference = self.review['supported_floor_start_raw_rad_by_id'][str(mid)]
        delta = raw - reference
        if abs(delta) <= math.radians(3.):
            return True
        return (mid in (3, 9)
                and self.review.get('wrap_equivalence_motor_ids') == [3, 9]
                and self.review.get('physical_full_turn_excluded') is True
                and abs(abs(delta) - 2. * math.pi) <= math.radians(3.))

    def set_centers(self, centers):
        require(not self.center_target_bytes and set(centers) == set(self.ids),
                'Exact-wire gate requires one complete fresh bus center set')
        for mid in self.ids:
            center = centers[mid]
            require(type(center) in (int, float) and math.isfinite(center)
                    and self._floor_equivalent(center, mid),
                    'Fresh exact-wire center differs from supported floor envelope')
            phase = (self.protocol.TrialPhase.POSITION_STEP5_KP4 if mid in (4, 10)
                     else self.protocol.TrialPhase.POSITION_STEP5)
            self.center_target_bytes[mid] = self.protocol.motion_request(
                phase=phase, center_rad=center, motor_id=mid)[7:9]

    @property
    def in_waiting(self):
        return self.raw.in_waiting

    @property
    def port(self):
        return self.raw.port

    def read(self, count):
        return self.raw.read(count)

    def write(self, wire):
        parser = self.parser_type()
        frames = parser.feed(wire) if type(wire) is bytes else []
        require(len(frames) == 1 and not parser.buffer and not parser.discarded_bytes,
                'Exact-wire gate rejects malformed or compound frame')
        frame = frames[0]
        mid, kind = frame.destination, frame.kind
        require(mid in self.ids and frame.flags == 4 and len(frame.data) == 8,
                'Exact-wire gate rejects bus/ID/header')
        P = self.protocol
        if kind == 0:
            allowed = wire == self.read_request(mid)
        elif kind == 3:
            allowed = wire == P.enable_request(phase=P.TrialPhase.ENABLE, motor_id=mid)
        elif kind == 4:
            allowed = wire == P.stop_request(phase=P.TrialPhase.STOP, motor_id=mid)
        elif kind == 18:
            allowed = wire == P.watchdog_setup_request(
                phase=P.TrialPhase.WATCHDOG_SETUP, motor_id=mid)
        elif kind == 17:
            allowed = any(wire == self.read_request(mid, name) for name in
                          ('run_mode', 'position', 'current', 'velocity', 'voltage', 'can_timeout'))
        elif kind == 1:
            phase = (P.TrialPhase.POSITION_STEP5_KP4 if mid in (4, 10)
                     else P.TrialPhase.POSITION_STEP5) if self.enable_seen else P.TrialPhase.ZERO_GAIN
            canonical = P.motion_request(phase=phase, center_rad=0., motor_id=mid)
            allowed = wire[:7] == canonical[:7] and wire[9:] == canonical[9:]
            raw_target = int.from_bytes(wire[7:9], 'big')
            target_rad = P.POSITION_MIN + raw_target * (P.POSITION_MAX - P.POSITION_MIN) / 65535.
            allowed = (allowed and self._floor_equivalent(target_rad, mid))
            if self.enable_seen and allowed:
                allowed = wire[7:9] == self.center_target_bytes.get(mid)
                previous = self.active_targets.setdefault(mid, raw_target)
                allowed = allowed and raw_target == previous
        else:
            allowed = False
        require(allowed, 'Exact-wire gate rejected nonreviewed command')
        if kind == 4 and self.enable_seen:
            self.active_stop_started.set()
        written = self.raw.write(wire)
        if kind == 3 and written == len(wire):
            self.enable_seen = True
        return written


def verify_files(base, expected_manifest_sha256):
    require(base.is_dir() and not base.is_symlink(), 'Frozen active directory missing')
    manifest_path = base / 'manifest.json'
    require(manifest_path.is_file() and not manifest_path.is_symlink()
            and sha(manifest_path) == expected_manifest_sha256,
            'Trusted active manifest SHA mismatch')
    manifest = json.loads(manifest_path.read_text())
    expected = ({'prepared_load_transfer.py', 'review.json'}
                | {f'singularitydog_hw/{name}' for name in SOURCES}
                | {f'evidence/{name}' for name in EVIDENCE})
    require(type(manifest) is dict and set(manifest) == expected,
            'Active manifest file set differs')
    actual = {str(path.relative_to(base)) for path in base.rglob('*') if path.is_file()}
    require(actual == expected | {'manifest.json'}, 'Active bundle has extra/missing file')
    for name, digest in manifest.items():
        path = base / name
        require(type(digest) is str and len(digest) == 64
                and all(c in '0123456789abcdef' for c in digest)
                and path.is_file() and not path.is_symlink() and sha(path) == digest,
                'Active pin mismatch: ' + name)
    review = json.loads((base / 'review.json').read_text())
    require(review.get('boot_id') == BOOT and review.get('duration_s') == 2.
            and review.get('gain_profile') == 'id4-id10-kp4'
            and review.get('wrap_equivalence_motor_ids') == [3, 9]
            and review.get('physical_full_turn_excluded') is True
            and review.get('review_complete') is True
            and review.get('load_transfer_hold_authorized') is True
            and review.get('learned_policy_allowed') is False
            and review.get('standing_allowed') is False
            and review.get('automatic_retry_allowed') is False
            and review.get('continuous_human_support_required') is True
            and review.get('stand_fully_supporting_required') is False
            and review.get('partial_load_allowed') is True
            and review.get('stand_removed_only_with_40v_off') is True
            and review.get('full_support_through_tick0') is True
            and review.get('slight_ease_max_duration_s') == .5
            and review.get('full_resupport_before_stop') is True
            and review.get('side_view_video_available') is True
            and review.get('partial_pre_enable_id3_full_window_range_rad') == .003
            and review.get('box_baseline_correlated_with_human_held_r4') is True
            and review.get('human_held_pre_enable_requalification_required') is True
            and review.get('self_supported_stance_proven') is False
            and review.get('timed_stop_catch_demonstrated') is False
            and all(review.get(flag) is True for flag in (
                'supported_stance_passed', 'physical_catch_reviewed',
                'power_cutoff_operator_reviewed', 'off_power_transfer_rehearsal_reviewed',
                'load_specific_limits_reviewed', 'serial_write_timeout_verified',
                'two_usb_80ms_workload_verified')),
            'Frozen active review is incomplete or exceeds brief partial-load scope')
    require('LIVE_OUTPUT_ENABLED = True' in
            (base / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text(),
            'Frozen runner live gate is not explicitly open')
    disabled_manifest = json.loads((base / 'evidence/disabled-r8-manifest.json').read_text())
    live_source = (base / 'singularitydog_hw/rs05_load_transfer_hold.py').read_text()
    require(live_source.count('LIVE_OUTPUT_ENABLED = True') == 1,
            'Active source gate must occur exactly once')
    supported_source = unpatch_partial_source(live_source)
    disabled_source = supported_source.replace('LIVE_OUTPUT_ENABLED = True',
                                               'LIVE_OUTPUT_ENABLED = False')
    require(hashlib.sha256(disabled_source.encode()).hexdigest() ==
            disabled_manifest['singularitydog_hw/rs05_load_transfer_hold.py']
            and all(manifest['singularitydog_hw/' + name] ==
                    disabled_manifest['singularitydog_hw/' + name]
                    for name in SOURCES if name != 'rs05_load_transfer_hold.py'),
            'Active runtime differs from disabled r8 beyond the one-line source gate')
    floor = json.loads((base / 'evidence/floor-summary.json').read_text())
    require(floor.get('boot_id') == BOOT
            and review.get('floor_hold_summary_sha256') == manifest['evidence/floor-summary.json']
            and floor.get('events_sha256') == manifest['evidence/floor-events.jsonl']
            and review.get('readonly_summary_sha256') == manifest['evidence/readonly-summary.json']
            and review.get('operator_rehearsal_sha256') == manifest['evidence/operator-rehearsal.json']
            and review.get('motor_uids') == floor.get('result', {}).get('review', {}).get('motor_uids'),
            'Active review differs from same-boot floor evidence')
    held = json.loads((base / 'evidence/box-baseline-summary.json').read_text())
    draft = json.loads((base / 'evidence/box-baseline-draft.json').read_text())
    operator = json.loads((base / 'evidence/box-baseline-operator-report.json').read_text())
    pose = held.get('pose') or {}
    raw = pose.get('raw_rad_by_id') or {}
    identities = held.get('identities') or {}
    require(held.get('schema') == 'singularitydog.fixed-stance-readonly-capture.v1'
            and held.get('status') == 'RECORDED_REVIEW_REQUIRED'
            and held.get('boot_id') == BOOT and held.get('errors') == []
            and held.get('output_allowed') is False
            and held.get('approved_for_runtime') is False
            and held.get('stop_state') == 'UNVERIFIED_BY_READ_ONLY_PROTOCOL'
            and held.get('source_sha256', {}).get('fixed_stance_readonly_capture.py') ==
                manifest['evidence/box-baseline-source.py']
            and held.get('source_sha256', {}).get('can_readonly.py') ==
                manifest['singularitydog_hw/can_readonly.py']
            and held.get('plan', {}).get('allowed_can_types') == [0, 17]
            and held.get('plan', {}).get('motor_output_available') is False
            and held.get('plan', {}).get('stop_command_available') is False
            and set(identities) == {str(i) for i in range(1, 13)}
            and {mid: row.get('mcu_uid_hex') for mid, row in identities.items()} == review['motor_uids']
            and set(raw) == {str(i) for i in range(1, 13)}
            and all(type(value) in (int, float) and math.isfinite(value)
                    and -12.57 <= value <= 12.57 for value in raw.values())
            and pose.get('sampling_issues') == []
            and pose.get('sampling_stability_heuristic_passed') is True
            and pose.get('stationarity_verified') is False
            and review.get('supported_floor_start_raw_rad_by_id') == raw
            and review.get('sha256') == manifest['evidence/box-baseline-summary.json']
            and draft.get('stance_capture_sha256') == manifest['evidence/box-baseline-summary.json']
            and draft.get('raw_rad_by_id') == raw
            and draft.get('output_allowed') is False
            and operator.get('boot_id') == BOOT
            and operator.get('baseline_summary_sha256') == manifest['evidence/box-baseline-summary.json']
            and all(operator.get(flag) is True for flag in (
                'box_supporting_torso', 'four_paws_floor',
                'no_slip_or_clamp_contact'))
            and operator.get('stand_removed_during_capture') is False
            and review.get('box_baseline_summary_sha256') == manifest['evidence/box-baseline-summary.json']
            and review.get('box_baseline_events_sha256') == manifest['evidence/box-baseline-events.jsonl']
            and review.get('box_baseline_draft_sha256') == manifest['evidence/box-baseline-draft.json']
            and review.get('box_baseline_operator_report_sha256') ==
                manifest['evidence/box-baseline-operator-report.json'],
            'Fresh same-boot box-supported four-paw baseline is incomplete')
    disabled = json.loads((base / 'evidence/disabled-r8-summary.json').read_text())
    require(disabled.get('boot_id') == BOOT
            and disabled.get('status') == 'PREFLIGHT_PASSED_RESET_CONFIRMED'
            and disabled.get('errors') == [] and disabled.get('signals') == []
            and disabled.get('motor_enable_sent') is False
            and disabled.get('motion_gain_sent') is False
            and disabled.get('trial_device_closed') is True
            and disabled.get('locks_released') is True
            and disabled.get('wrapper_sha256') == disabled_manifest['prepared_load_transfer.py']
            and disabled.get('events_sha256') == manifest['evidence/disabled-r8-events.jsonl']
            and disabled.get('result', {}).get('stop_confirmed') is True
            and all(disabled['result']['workers'][bus]['cycle_count'] == 25
                    and len(disabled['result']['workers'][bus]['electrical_samples']) == 25
                    and all(row['confirmed'] is True for row in
                            disabled['result']['workers'][bus]['stop_reports'].values())
                    for bus in ('front', 'rear'))
            and review.get('disabled_r8_summary_sha256') == manifest['evidence/disabled-r8-summary.json']
            and review.get('disabled_r8_events_sha256') == manifest['evidence/disabled-r8-events.jsonl']
            and review.get('disabled_r8_manifest_sha256') == manifest['evidence/disabled-r8-manifest.json'],
            'Exact same-boot disabled 80 ms trial evidence is incomplete')
    prior_manifest = json.loads((base / 'evidence/supported-r1-manifest.json').read_text())
    prior = json.loads((base / 'evidence/supported-r1-summary.json').read_text())
    require(prior.get('boot_id') == BOOT and prior.get('status') == 'ABORTED'
            and prior.get('events_sha256') == manifest['evidence/supported-r1-events.jsonl']
            and prior.get('wrapper_sha256') == prior_manifest['prepared_load_transfer.py']
            and prior.get('motor_enable_sent') is True
            and prior.get('motion_gain_sent') is True
            and prior.get('trial_device_closed') is True
            and prior.get('locks_released') is True
            and prior.get('result', {}).get('stop_confirmed') is True
            and any('Stale feedback' in error for error in prior['result']['errors'])
            and all(prior['result']['workers'][bus]['cycle_count'] == 1
                    for bus in ('front', 'rear'))
            and review.get('supported_r1_summary_sha256') == manifest['evidence/supported-r1-summary.json']
            and review.get('supported_r1_events_sha256') == manifest['evidence/supported-r1-events.jsonl']
            and review.get('supported_r1_manifest_sha256') == manifest['evidence/supported-r1-manifest.json'],
            'Prior one-cycle abort/STOP evidence is incomplete')
    supported_manifest = json.loads((base / 'evidence/supported-r2-manifest.json').read_text())
    supported = json.loads((base / 'evidence/supported-r2-summary.json').read_text())
    require(sha(base / 'evidence/supported-r2-runtime.py') ==
                supported_manifest['singularitydog_hw/rs05_load_transfer_hold.py']
            and (base / 'evidence/supported-r2-runtime.py').read_text() == supported_source
            and all(manifest['singularitydog_hw/' + name] ==
                    supported_manifest['singularitydog_hw/' + name]
                    for name in SOURCES if name != 'rs05_load_transfer_hold.py'),
            'Partial runtime differs from supported r2 beyond exact ID3 pre-Enable delta')
    require(supported.get('boot_id') == BOOT
            and supported.get('status') == 'SUPPORTED_HOLD_COMPLETED_RESET_CONFIRMED'
            and supported.get('errors') == [] and supported.get('signals') == []
            and supported.get('events_sha256') == manifest['evidence/supported-r2-events.jsonl']
            and supported.get('wrapper_sha256') == supported_manifest['prepared_load_transfer.py']
            and supported.get('motor_enable_sent') is True
            and supported.get('motion_gain_sent') is True
            and supported.get('trial_device_closed') is True
            and supported.get('locks_released') is True
            and supported.get('result', {}).get('status') == 'LOAD_TRANSFER_HOLD_COMPLETED_RESET_CONFIRMED'
            and supported['result'].get('stop_confirmed') is True
            and all(supported['result']['workers'][bus]['cycle_count'] == 25
                    and len(supported['result']['workers'][bus]['electrical_samples']) == 25
                    and all(row['confirmed'] is True for row in
                            supported['result']['workers'][bus]['stop_reports'].values())
                    for bus in ('front', 'rear'))
            and review.get('supported_r2_summary_sha256') == manifest['evidence/supported-r2-summary.json']
            and review.get('supported_r2_events_sha256') == manifest['evidence/supported-r2-events.jsonl']
            and review.get('supported_r2_manifest_sha256') == manifest['evidence/supported-r2-manifest.json'],
            'Complete stand-supported two-second prerequisite is missing')
    prior_partial_manifest = json.loads((base / 'evidence/partial-r3-manifest.json').read_text())
    prior_partial = json.loads((base / 'evidence/partial-r3-summary.json').read_text())
    front = prior_partial.get('result', {}).get('workers', {}).get('front', {})
    rejected = front.get('settled_windows', {}).get('FR', {})
    id3 = rejected.get('motors', {}).get('3', rejected.get('motors', {}).get(3, {}))
    require(prior_partial.get('boot_id') == BOOT
            and prior_partial.get('status') == 'ABORTED'
            and prior_partial.get('motor_enable_sent') is False
            and prior_partial.get('motion_gain_sent') is False
            and prior_partial.get('trial_device_closed') is True
            and prior_partial.get('locks_released') is True
            and prior_partial.get('events_sha256') == manifest['evidence/partial-r3-events.jsonl']
            and prior_partial.get('wrapper_sha256') == prior_partial_manifest['prepared_load_transfer.py']
            and prior_partial.get('result', {}).get('stop_confirmed') is True
            and rejected.get('errors') == ['ID3: position_range_rad exceeds 0.001']
            and .001 < id3.get('position_range_rad', 0) < .003
            and id3.get('errors') == ['position_range_rad exceeds 0.001']
            and all(len(prior_partial['result']['workers'][bus]['stop_reports']) == 6
                    and all(row['confirmed'] is True for row in
                            prior_partial['result']['workers'][bus]['stop_reports'].values())
                    for bus in ('front', 'rear'))
            and review.get('partial_r3_summary_sha256') == manifest['evidence/partial-r3-summary.json']
            and review.get('partial_r3_events_sha256') == manifest['evidence/partial-r3-events.jsonl']
            and review.get('partial_r3_manifest_sha256') == manifest['evidence/partial-r3-manifest.json'],
            'ID3-only pre-Enable r3 abort/STOP evidence is missing')
    latest_partial_manifest = json.loads((base / 'evidence/partial-r4-manifest.json').read_text())
    latest_partial = json.loads((base / 'evidence/partial-r4-summary.json').read_text())
    require(latest_partial.get('boot_id') == BOOT
            and latest_partial.get('status') == 'ABORTED'
            and latest_partial.get('motor_enable_sent') is False
            and latest_partial.get('motion_gain_sent') is False
            and latest_partial.get('trial_device_closed') is True
            and latest_partial.get('locks_released') is True
            and latest_partial.get('events_sha256') == manifest['evidence/partial-r4-events.jsonl']
            and latest_partial.get('wrapper_sha256') == latest_partial_manifest['prepared_load_transfer.py']
            and latest_partial.get('result', {}).get('stop_confirmed') is True
            and any('Exact-wire gate rejected' in error for error in
                    latest_partial.get('result', {}).get('errors', []))
            and review.get('partial_r4_summary_sha256') == manifest['evidence/partial-r4-summary.json']
            and review.get('partial_r4_events_sha256') == manifest['evidence/partial-r4-events.jsonl']
            and review.get('partial_r4_manifest_sha256') == manifest['evidence/partial-r4-manifest.json'],
            'Human-held pose mismatch r4 abort/STOP evidence is missing')
    prior_raw = {str(mid): latest_partial['result']['workers'][
        'front' if mid <= 6 else 'rear']['motors'][str(mid)]['position']
        for mid in range(1, 13)}
    require(all(abs(raw[mid] - prior_raw[mid]) <= math.radians(.1) for mid in raw),
            'Box capture differs from prior human-held r4 raw pose')
    box_manifest = json.loads((base / 'evidence/box-supported-manifest.json').read_text())
    box = json.loads((base / 'evidence/box-supported-summary.json').read_text())
    require(hashlib.sha256(supported_source.encode()).hexdigest() ==
                box_manifest['singularitydog_hw/rs05_load_transfer_hold.py']
            and all(manifest['singularitydog_hw/' + name] ==
                    box_manifest['singularitydog_hw/' + name]
                    for name in SOURCES if name != 'rs05_load_transfer_hold.py'),
            'Partial runtime differs from successful box-supported trial beyond ID3 pre-Enable delta')
    require(box.get('boot_id') == BOOT
            and box.get('status') == 'SUPPORTED_HOLD_COMPLETED_RESET_CONFIRMED'
            and box.get('errors') == [] and box.get('signals') == []
            and box.get('events_sha256') == manifest['evidence/box-supported-events.jsonl']
            and box.get('wrapper_sha256') == box_manifest['prepared_load_transfer.py']
            and box.get('motor_enable_sent') is True
            and box.get('motion_gain_sent') is True
            and box.get('first_hold_cycle_confirmed') is True
            and box.get('trial_device_closed') is True
            and box.get('locks_released') is True
            and box.get('result', {}).get('status') == 'LOAD_TRANSFER_HOLD_COMPLETED_RESET_CONFIRMED'
            and box['result'].get('stop_confirmed') is True
            and box['result'].get('review', {}).get('motor_uids') == review['motor_uids']
            and box['result'].get('review', {}).get('supported_floor_start_raw_rad_by_id') == raw
            and all(box['result']['workers'][bus]['cycle_count'] == 25
                    and len(box['result']['workers'][bus]['electrical_samples']) == 25
                    and len(box['result']['workers'][bus]['stop_reports']) == 6
                    and all(row['confirmed'] for row in
                            box['result']['workers'][bus]['stop_reports'].values())
                    for bus in ('front', 'rear'))
            and review.get('box_supported_summary_sha256') == manifest['evidence/box-supported-summary.json']
            and review.get('box_supported_events_sha256') == manifest['evidence/box-supported-events.jsonl']
            and review.get('box_supported_manifest_sha256') == manifest['evidence/box-supported-manifest.json'],
            'Fresh box-pose supported active r3 prerequisite has not passed')
    return review


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--active', action='store_true', required=True)
    parser.add_argument('--stand-removed-under-40v-off', action='store_true', required=True)
    parser.add_argument('--two-operators-full-support', action='store_true', required=True)
    parser.add_argument('--slight-ease-only', action='store_true', required=True)
    parser.add_argument('--resupport-before-stop', action='store_true', required=True)
    parser.add_argument('--side-view-video-ready', action='store_true', required=True)
    parser.add_argument('--paws-floor', action='store_true', required=True)
    parser.add_argument('--continuous-catch', action='store_true', required=True)
    parser.add_argument('--cutoff-ready', action='store_true', required=True)
    parser.add_argument('--off-power-rehearsal', action='store_true', required=True)
    parser.add_argument('--audio-announced', action='store_true', required=True)
    parser.add_argument('--trial-authorized', action='store_true', required=True)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if (not all((args.active, args.stand_removed_under_40v_off,
                 args.two_operators_full_support, args.slight_ease_only,
                 args.resupport_before_stop, args.side_view_video_ready,
                 args.paws_floor,
                 args.continuous_catch, args.cutoff_ready,
                 args.off_power_rehearsal, args.audio_announced,
                 args.trial_authorized)) or not args.output.is_absolute()
            or args.output.parent != Path('/home/jetson/singularitydog-logs')
            or args.output.exists() or args.output.is_symlink()):
        parser.error('All reviewed physical, audio and authorization flags plus a fresh log path are required')
    os.umask(0o077)
    sys.dont_write_bytecode = True
    args.output.mkdir(mode=0o700)
    report = {'status': 'INCOMPLETE', 'boot_id': BOOT, 'preflight_only': False,
              'supported_diagnostic_only': False, 'partial_load_allowed': True,
              'full_support_through_tick0': True,
              'slight_ease_max_duration_s': .5,
              'full_resupport_before_stop': True,
              'side_view_video_available': True,
              'continuous_human_support_required': True,
              'load_transfer_proven': False, 'self_supported_stance_proven': False,
              'partial_load_physically_verified': False,
              'side_view_video_reviewed': False,
              'external_audio_operator_confirmed': True,
              'motor_enable_sent': False, 'motion_gain_sent': False,
              'errors': [], 'signals': [], 'typed_tx_counts': {}, 'captures': {},
              'trial_device_closed': None, 'locks_released': False,
              'wrapper_sha256': sha(__file__), 'started_wall_time_ns': time.time_ns()}
    events, ports, transports, handlers = [], {}, {}, {}
    emit_lock = threading.RLock()
    deadline = time.monotonic() + 45.
    bindings = None
    legacy = None

    def check():
        require(not report['signals'] and time.monotonic() < deadline,
                'Signal or 45 s finite trial deadline')

    def emit(event):
        row = {'wall_time_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(), **event}
        with emit_lock:
            require(len(events) < 50000, 'Bounded event log full')
            if row.get('kind') == 'load_transfer_bus_ready':
                transports[row['bus']].serial.set_centers(row['centers'])
            if row.get('kind') == 'can_tx':
                wire = bytes.fromhex(row['hex'])
                kind = ((int.from_bytes(wire[2:6], 'big') >> 3) >> 24) & 31
                counts = report['typed_tx_counts']
                counts[str(kind)] = counts.get(str(kind), 0) + 1
                if kind == 3:
                    report['motor_enable_sent'] = True
                if kind == 1 and wire[11:15] != bytes(4):
                    report['motion_gain_sent'] = True
            events.append(row)

    try:
        require(Path(__file__).resolve().parent == BASE, 'Use pinned remote active path')
        review = verify_files(BASE, args.manifest_sha256)
        require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == BOOT,
                'Jetson boot differs from frozen two-second review')
        current_spec = importlib.util.spec_from_file_location('current_fullbody', CURRENT / 'prepared_fullbody.py')
        current_wrapper = importlib.util.module_from_spec(current_spec)
        current_spec.loader.exec_module(current_wrapper)
        current_wrapper.verify_files(CURRENT)
        legacy_spec = importlib.util.spec_from_file_location('legacy_ownership', CURRENT / 'legacy/prepared_transaction.py')
        legacy = importlib.util.module_from_spec(legacy_spec)
        legacy_spec.loader.exec_module(legacy)
        legacy.verify_package(CURRENT / 'legacy')
        expected = legacy.validate_uids(json.loads((CURRENT / 'legacy/expected-uids.json').read_text()))
        require(review['motor_uids'] == expected, 'Current physical 12 UID binding differs')
        sys.path.insert(0, str(BASE))
        from singularitydog_hw.can_readonly import ATParser, read_request
        from singularitydog_hw import rs05_trial_protocol as protocol
        from singularitydog_hw.rs05_bus_transport import BusTrialTransport
        from singularitydog_hw.rs05_load_transfer_hold import _review, run_load_transfer_hold
        import serial
        for name in SOURCES:
            if name == '__init__.py':
                continue
            module = sys.modules.get('singularitydog_hw.' + name.removesuffix('.py'))
            if module is not None:
                require(Path(module.__file__).resolve() == BASE / 'singularitydog_hw' / name,
                        'Unexpected imported runtime module: ' + name)
        _review({int(k): v for k, v in expected.items()}, (5, 6, 8), review,
                False, 'id4-id10-kp4', 2., True)
        bindings = legacy.validate_bindings()
        report['bindings'] = bindings
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.signal(sig, lambda n, _: report['signals'].append(n))
        check()
        with legacy.ownership(bindings, report):
            report['trial_device_closed'] = False
            try:
                active_stop_started = threading.Event()
                for bus in ('front', 'rear'):
                    p = serial.Serial(port=None, baudrate=921600, bytesize=8,
                                      parity='N', stopbits=1, timeout=.002,
                                      write_timeout=.02, exclusive=True,
                                      rtscts=False, dsrdtr=False, xonxoff=False)
                    ports[bus] = p
                    p.dtr = p.rts = False
                    p.port = bindings[bus]['path']
                    p.open()
                    legacy.check_binding(bindings[bus], p)
                    transports[bus] = BusTrialTransport(
                        ExactWirePort(p, legacy.BUS_IDS[bus], review,
                                      parser_type=ATParser, read_request=read_request,
                                      protocol=protocol,
                                      active_stop_started=active_stop_started),
                        lambda event, b=bus: emit({'bus': b, **event}), check,
                        ids=legacy.BUS_IDS[bus], interleave_feedback=True)
                require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == BOOT,
                        'Jetson boot changed before active trial')
                first_cycle_confirmed = threading.Event()
                runner_finished = threading.Event()
                cue_opened = threading.Event()
                cue_lock = threading.Lock()
                cue_trace = {}

                def cue_writer(message, *, flush=True):
                    cue_trace[message] = time.monotonic_ns()
                    print(message, flush=flush)

                cue_thread = threading.Thread(target=announce_partial_window,
                    args=(first_cycle_confirmed, runner_finished,
                          active_stop_started, cue_opened,
                          events, emit_lock, cue_lock),
                    kwargs={'writer': cue_writer},
                    name='partial-load-cue', daemon=True)
                cue_thread.start()
                try:
                    report['result'] = run_load_transfer_hold(
                        transports, {int(k): v for k, v in expected.items()}, check, emit,
                        validated_review=review, preflight_only=False, live_output=True,
                        duration_s=2., gain_profile='id4-id10-kp4',
                        active_start_signal=first_cycle_confirmed)
                finally:
                    with cue_lock:
                        runner_finished.set()
                    cue_thread.join(timeout=.2)
                report['first_hold_cycle_confirmed'] = first_cycle_confirmed.is_set()
                report['partial_load_window_opened'] = cue_opened.is_set()
                report['cue_monotonic_ns'] = cue_trace
                report['resupport_cue_sent'] = 'RE_SUPPORT_NOW' in cue_trace
                require(report['result'].get('stop_confirmed') is True,
                        'All12 STOP not confirmed: CUT40V_POWER')
            finally:
                closed = {}
                for bus, p in ports.items():
                    try:
                        p.close()
                        closed[bus] = not p.is_open
                    except BaseException as error:
                        closed[bus] = False
                        report['errors'].append(repr(error))
                report['port_closes'] = closed
                report['trial_device_closed'] = all(closed.values())
        require(report['result']['status'] == 'LOAD_TRANSFER_HOLD_COMPLETED_RESET_CONFIRMED'
                and report['result']['errors'] == []
                and report['result']['workers']['front']['cycle_count'] == 25
                and report['result']['workers']['rear']['cycle_count'] == 25
                and all(len(report['result']['workers'][bus]['electrical_samples']) == 25
                        for bus in ('front', 'rear'))
                and report['trial_device_closed'] is True
                and report['locks_released'] is True
                and report.get('first_hold_cycle_confirmed') is True
                and report.get('partial_load_window_opened') is True
                and report.get('resupport_cue_sent') is True
                and report['motor_enable_sent'] is True
                and report['motion_gain_sent'] is True,
                'Partial-load two-second current-position hold did not pass')
        report['status'] = 'PARTIAL_LOAD_WINDOW_COMPLETED_RESET_CONFIRMED_UNVERIFIED'
    except BaseException as error:
        report['errors'].append(repr(error))
        report['status'] = 'ABORTED'
    finally:
        for sig, prior in handlers.items():
            signal.signal(sig, prior)
        report['completed_wall_time_ns'] = time.time_ns()
        events_path = args.output / 'events.jsonl'
        with events_path.open('x') as stream:
            for event in events:
                stream.write(json.dumps(event, allow_nan=False) + '\n')
            stream.flush(); os.fsync(stream.fileno())
        report['events_sha256'] = sha(events_path)
        with (args.output / 'summary.json').open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.flush(); os.fsync(stream.fileno())
    print(json.dumps({key: report[key] for key in
        ('status', 'errors', 'motor_enable_sent', 'motion_gain_sent', 'locks_released')}), flush=True)
    return 0 if report['status'] == 'PARTIAL_LOAD_WINDOW_COMPLETED_RESET_CONFIRMED_UNVERIFIED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
