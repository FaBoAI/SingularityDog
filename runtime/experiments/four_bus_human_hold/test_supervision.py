"""New30 terminal/timer fault tests using synthetic bindings only; no output qualification.

Legacy fault cases are retained on the independent copied supervisor. The
production active-proof gate is patched only in these memory/PTY tests.
"""
import math
from contextlib import contextmanager
import os
import pty
import threading
import unittest
from unittest.mock import patch

from . import profile as profile_gate
from .profile import SCOPE, SETTINGS, ACCEPTANCE, CAPS
from singularitydog_hw.policy_clock import control_clock_payload
from .supervision import HumanSupported30Execution as _ProductionSupervisor, linux_timer_scope


@contextmanager
def synthetic_timer_scope():
    record={'native_tid':threading.get_native_id(),'cpu_mask':[0,1,2,3],
            'file_only_mock_readback':True,'restored':False}
    try: yield record
    finally: record['restored']=True


class HumanSupportedHoldExecution(_ProductionSupervisor):
    """Inject only synthetic wire timestamps; never a production proof."""
    def __init__(self,*args,**kwargs):
        kwargs.setdefault('timer_scope',synthetic_timer_scope)
        super().__init__(*args,**kwargs)

    def after_cycle_validated(self, begun_ns, ended_ns, phase, **kwargs):
        kwargs.setdefault('oldest_input_ns', begun_ns)
        kwargs.setdefault('final_write_ns', begun_ns + 17_000_000)
        kwargs.setdefault('last_reply_ns', ended_ns)
        return super().after_cycle_validated(begun_ns, ended_ns, phase, **kwargs)



def profile():
    axis = {'kp': 6., 'kd': .15, 'max_displacement_from_start_rad': math.radians(1),
            'max_measured_velocity_rad_s': .25, 'max_estimated_pd_torque_nm': .2,
            'max_command_velocity_rad_s': math.radians(1),
            'max_command_acceleration_rad_s2': math.radians(5)}
    axis.update(CAPS)
    return {'scope': SCOPE, 'diagnostic_timing_acceptance': ACCEPTANCE,
            'control_clock': control_clock_payload(30), 'inference_tx_budget_ns': 20_000_000,
            'cycle_budget_ns': 33_333_334, 'reply_budget_ns': 33_333_334, 'schema': 'singularitydog.four-can-human-hold30-runtime.v1',
            'human_supported_hold': dict(SETTINGS), 'duration_s': 8.,
            'startup_duration_s': 1., 'startup_damping_duration_s': 1., 'stop_duration_s': .4, 'policy_weight': 0.,
            'hard_cycle_ms': 1000/30, 'max_sample_age_ms': 20.,
            'axes': {str(i): dict(axis) for i in range(1, 13)}, 'output_allowed': False}


class ParkedThread:
    """Tests drive deadlines directly using a deterministic monotonic clock."""
    def __init__(self, **kwargs): self.kwargs = kwargs
    def start(self): pass
    def join(self, timeout=None): pass
    def is_alive(self): return False


def audio_manifest():
    # Synthetic playback metadata only; the loader's real PCM/SHA tests are separate.
    return {'clips': {name: {'duration_s': duration} for name, duration in
                      (('brief', .4), ('prepare_ease', .2), ('go', .02),
                       ('resupport', .2), ('abort', .2))}}


class FakeAudio:
    def __init__(self, clock):
        self.clock = clock; self.pending = {}; self.cancelled = False; self.closed = False
        self.requests = []
    def start(self, stage, on_complete, on_failure, on_started=None):
        self.requests.append(stage)
        if self.closed or self.cancelled and stage not in ('resupport', 'abort'):
            on_failure(stage, 'cancelled'); return
        if self.pending:
            on_failure(stage, 'overlap'); return
        started = self.clock()
        self.pending[stage] = started, on_complete, on_failure
        if on_started: on_started(stage, started)
    def finish(self, stage):
        started, complete, _ = self.pending.pop(stage)
        complete(stage, started, self.clock())
    def cancel(self): self.cancelled = True; self.pending.clear()
    def close(self): self.closed = True; self.pending.clear()


class HumanSupportedExecutionTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.addCleanup(os.close, self.master); self.addCleanup(os.close, self.slave)
        self.now = 1_000_000_000
        proof = patch('experiments.four_bus_human_hold.supervision.supervisor_view', return_value=(profile(), audio_manifest()))
        proof.start(); self.addCleanup(proof.stop)
        self.audio = FakeAudio(lambda: self.now)
        self.execution = HumanSupportedHoldExecution(
            self.slave, clock=lambda: self.now, thread_factory=ParkedThread, stage_audio=self.audio)
        self.addCleanup(self.execution.close)
        self.cancelled = []
        self.execution.connect_cancel(lambda: self.cancelled.append(True))
        self.execution.bind_profile(profile(), active=True)
        self.execution.on_start(self.now)

    def cycle(self, seconds, *, phase='active', full_gain=True, stop=False):
        self.now = 1_000_000_000 + round(seconds * 1e9)
        self.execution.after_cycle_validated(
            self.now - 19_000_000, self.now, phase,
            full_gain=full_gain, stop_requested=stop)

    def finish(self, stage, execution=None):
        e = execution or self.execution
        self.now += round(audio_manifest()['clips'][stage]['duration_s'] * 1e9)
        e.stage_audio.finish(stage)

    def open(self):
        self.cycle(1.2)
        self.assertIsNone(self.execution.cue_ns)
        self.finish('prepare_ease')
        self.cycle(1.44)
        self.finish('go')

    def resupport(self):
        self.now = self.execution.ease_deadline_ns
        self.execution.before_cycle(self.now)
        self.finish('resupport')

    def ack(self):
        os.write(self.master, b'\n')
        self.now += 20_000_000
        return self.execution.before_cycle(self.now)

    def test_only_fresh_active_all_axis_full_gain_cycle_opens_once(self):
        self.cycle(.9, phase='starting')
        self.cycle(1.19)
        self.cycle(1.21, full_gain=False)
        self.assertIsNone(self.execution.cue_ns)
        self.cycle(1.24)
        self.assertIsNone(self.execution.cue_ns)
        self.finish('prepare_ease')
        self.cycle(1.48)
        self.assertEqual(self.execution.cue_ns, self.now)
        self.assertEqual(self.execution.ease_deadline_ns, self.now + 500_000_000)
        self.finish('go'); self.cycle(1.52)
        self.assertEqual(sum(e['key'] == 'SLIGHT_EASE_WINDOW_OPEN'
                             for e in self.execution.events), 1)

    def test_full_support_ack_after_cue_allows_normal_gain_down(self):
        self.open(); self.resupport()
        self.assertIsNone(self.execution.ack_ns)
        self.assertTrue(self.ack())
        self.assertGreater(self.execution.ack_ns, self.execution.resupport_cue_ns)
        self.execution.before_stop()
        self.assertEqual(self.cancelled, [])
        self.assertEqual(self.execution.events[-1]['key'], 'FULL_SUPPORT_BEFORE_NORMAL_STOP')

    def test_buffered_enter_before_ease_or_before_resupport_never_confirms(self):
        os.write(self.master, b'\n')
        self.open()
        os.write(self.master, b'\n')
        self.resupport()
        self.now += 20_000_000
        self.assertFalse(self.execution.before_cycle(self.now))
        self.assertIsNone(self.execution.ack_ns)
        self.assertTrue(self.ack())

    def test_missing_or_exactly_late_ack_cancels_without_delaying_stop(self):
        self.open(); self.resupport()
        self.now = self.execution.ack_deadline_ns
        with self.assertRaisesRegex(RuntimeError, 'Full support'):
            self.execution.before_cycle(self.now)
        self.assertIsNone(self.execution.ack_ns)
        self.assertTrue(self.cancelled)
        self.execution.before_stop(emergency=True)

    def test_enter_exactly_at_ack_deadline_never_extends_window(self):
        self.open(); self.resupport(); os.write(self.master, b'\n')
        self.now = self.execution.ack_deadline_ns
        with self.assertRaises(RuntimeError): self.execution.before_cycle(self.now)
        self.assertIsNone(self.execution.ack_ns)

    def test_late_enter_is_not_accepted(self):
        self.open(); self.resupport()
        os.write(self.master, b'\n')
        self.now = self.execution.ack_deadline_ns + 1
        with self.assertRaises(RuntimeError): self.execution.before_cycle(self.now)
        self.assertIsNone(self.execution.ack_ns)
        self.assertTrue(self.cancelled)

    def test_normal_stop_before_ack_is_forbidden(self):
        self.open()
        with self.assertRaisesRegex(RuntimeError, 'Fresh full-support'):
            self.execution.before_stop()
        self.assertTrue(self.cancelled)
        self.assertIsNotNone(self.execution.resupport_cue_ns)

    def test_early_cancel_never_opens_easing_and_announces_no_partial_load(self):
        self.execution.on_abort('SIMULATED_PRE_CYCLE_CAN_LOSS')
        with self.assertRaises(RuntimeError): self.cycle(1.2)
        self.assertIsNone(self.execution.cue_ns)
        self.assertIsNotNone(self.execution.resupport_cue_ns)
        self.assertTrue(self.cancelled)

    def test_cancel_during_window_requests_full_support_immediately(self):
        self.open()
        self.now += 20_000_000
        self.execution.on_abort('SIMULATED_IMU_FAULT')
        self.assertEqual(self.execution.resupport_cue_ns, self.now)
        self.assertTrue(self.cancelled)
        self.execution.before_stop(emergency=True)

    def test_graceful_request_before_cue_skips_easing(self):
        self.now += 100_000_000
        self.assertTrue(self.execution.before_cycle(self.now, stop_requested=True))
        self.cycle(1.2)
        self.assertTrue(self.execution.skipped)
        self.assertIsNone(self.execution.cue_ns)
        self.execution.before_stop()
        self.assertEqual(self.cancelled, [])

    def test_graceful_request_during_ease_waits_for_fresh_resupport_ack(self):
        self.open(); self.now += 20_000_000
        self.assertFalse(self.execution.before_cycle(self.now, stop_requested=True))
        self.assertEqual(self.execution.resupport_cue_ns, self.now)
        self.finish('resupport')
        self.assertTrue(self.ack())
        self.execution.before_stop()

    def test_cue_too_late_for_complete_window_is_skipped(self):
        self.cycle(6.4)
        self.assertIsNone(self.execution.cue_ns)
        self.assertTrue(self.execution.skipped)
        self.now += 20_000_000
        self.assertTrue(self.execution.before_cycle(self.now))

    def test_unvalidated_late_noncausal_or_repeated_cycle_never_opens_cue(self):
        for label, begin, end, now in (('slow', 2_166_000_000, 2_200_000_000, 2_200_000_000),
                                      ('old', 2_181_000_000, 2_200_000_000, 2_221_000_000),
                                      ('future', 2_181_000_000, 2_200_000_001, 2_200_000_000)):
            with self.subTest(label=label):
                e = self.new_execution(); self.now = now
                with self.assertRaises(RuntimeError):
                    e.after_cycle_validated(begin, end, 'active', full_gain=True)
                self.assertIsNone(e.cue_ns)
        self.cycle(.5, phase='starting')
        with self.assertRaises(RuntimeError): self.cycle(.5, phase='starting')

    def new_execution(self, **kwargs):
        e = HumanSupportedHoldExecution(self.slave, clock=lambda: self.now,
                                       thread_factory=kwargs.pop('thread_factory', ParkedThread),
                                       stage_audio=FakeAudio(lambda: self.now), **kwargs)
        self.addCleanup(e.close); e.connect_cancel(lambda: self.cancelled.append(True))
        e.bind_profile(profile(), active=True)
        e.on_start(1_000_000_000)
        return e

    def test_operator_q_and_terminal_read_error_cancel(self):
        self.open(); os.write(self.master, b'q\n')
        with self.assertRaises(InterruptedError): self.execution.before_cycle(self.now)
        self.assertTrue(self.cancelled)
        e = self.new_execution()
        with patch('experiments.four_bus_human_hold.supervision.os.read', side_effect=OSError('lost')):
            with self.assertRaises(OSError): e.before_cycle(self.now)
        self.assertIsNotNone(e.failed)

    def test_output_backpressure_and_partial_write_prohibit_easing(self):
        for result in ('blocked', 'partial'):
            with self.subTest(result=result):
                e = self.new_execution(); self.now = 2_200_000_000
                e.after_cycle_validated(self.now - 19_000_000, self.now, 'active', full_gain=True)
                self.finish('prepare_ease', e); self.now += 40_000_000
                kwargs = {'side_effect': BlockingIOError('full')} if result == 'blocked' else {'return_value': 1}
                with patch('experiments.four_bus_human_hold.supervision.os.write', **kwargs):
                    # An asynchronous Go-start callback cannot throw into the
                    # completed control cycle. It must cancel immediately and
                    # latch failure before any following output cycle.
                    e.after_cycle_validated(self.now - 19_000_000, self.now, 'active', full_gain=True)
                self.assertIsNotNone(e.failed)
                self.assertTrue(self.cancelled)
                with self.assertRaises(RuntimeError): e.before_cycle(self.now)

    def test_independent_timer_requests_resupport_without_any_next_cycle(self):
        self.open()
        self.now = self.execution.ease_deadline_ns
        # Exactly the timer's target runs with a simulated clock; no runtime hook follows.
        self.execution._timer_main()
        self.assertEqual(self.execution.resupport_cue_ns, self.now)
        self.assertIsNone(self.execution.ack_ns)

    def test_independent_timer_write_failure_cancels(self):
        self.open(); self.now = self.execution.ease_deadline_ns
        with patch('experiments.four_bus_human_hold.supervision.os.write', side_effect=BlockingIOError('full')):
            self.execution._timer_main()
        self.assertTrue(self.cancelled)
        self.assertEqual(self.execution.failed, 'INDEPENDENT_RESUPPORT_TIMER_FAILED')

    def test_production_timer_thread_needs_no_further_runtime_hook(self):
        e = self.new_execution(thread_factory=threading.Thread)
        self.now = 2_200_000_000
        e.after_cycle_validated(self.now - 19_000_000, self.now, 'active', full_gain=True)
        self.finish('prepare_ease', e); self.now += 40_000_000
        e.after_cycle_validated(self.now - 19_000_000, self.now, 'active', full_gain=True)
        self.finish('go', e)
        self.now = e.ease_deadline_ns
        e._timer.join(timeout=1.)
        self.assertFalse(e._timer.is_alive())
        self.assertEqual(e.resupport_cue_ns, self.now)

    def test_active_review_missing_or_plan_binding_cannot_start(self):
        e = HumanSupportedHoldExecution(self.slave, thread_factory=ParkedThread,
                                       stage_audio=FakeAudio(lambda: self.now))
        self.addCleanup(e.close)
        p = profile(); p['output_allowed'] = True
        with patch('experiments.four_bus_human_hold.supervision.supervisor_view',
                   side_effect=profile_gate.supervisor_view):
            with self.assertRaisesRegex(ValueError, 'Genuine independently'): e.bind_profile(p, active=True)
        e.bind_profile(profile(), active=False); e.connect_cancel(lambda: None)
        with self.assertRaisesRegex(RuntimeError, 'Reviewed active'): e.on_start(1)

    def test_report_does_not_claim_measured_weight_support_or_standing(self):
        self.open(); self.resupport(); self.ack()
        report = self.execution.decorate_report({})['human_supported_trial']
        self.assertFalse(report['actual_full_support_or_weight_transfer_measured'])
        self.assertFalse(report['standing_or_walking_authorized'])
        self.assertTrue(report['contact_and_stance_operator_review_required'])
        self.assertEqual(report['full_support_ack_ns'], self.execution.ack_ns)

    def test_close_open_window_cancels_and_restores_descriptor_state(self):
        self.open(); self.execution.close()
        self.assertTrue(self.cancelled)
        self.assertTrue(os.get_blocking(self.slave))
        self.assertTrue(self.execution.closed)
        self.assertIsNotNone(self.execution.resupport_cue_ns)
        self.execution.close()
        with self.assertRaises(RuntimeError): self.execution.on_start(self.now)
        with self.assertRaises(RuntimeError): self.execution.bind_profile(profile(), active=True)
        with self.assertRaises(RuntimeError): self.execution.before_cycle(self.now)

    def test_constructor_requires_same_visible_terminal_and_restores_partial_setup(self):
        read, write = os.pipe()
        try:
            with self.assertRaises(ValueError): HumanSupportedHoldExecution(read)
            with self.assertRaises(ValueError): HumanSupportedHoldExecution(self.slave, write_fd=write)
        finally: os.close(read); os.close(write)
        writer = os.open(os.ttyname(self.slave), os.O_WRONLY | os.O_NOCTTY)
        self.addCleanup(os.close, writer)
        before = os.get_blocking(self.slave), os.get_blocking(writer)
        original = os.set_blocking
        def fail(fd, blocking):
            original(fd, blocking)
            if fd == writer and not blocking: raise OSError('write-then-raise')
        with patch('experiments.four_bus_human_hold.supervision.os.set_blocking', side_effect=fail):
            with self.assertRaises(OSError): HumanSupportedHoldExecution(self.slave, write_fd=writer)
        self.assertEqual((os.get_blocking(self.slave), os.get_blocking(writer)), before)

    def test_timer_start_failure_is_fail_closed(self):
        class BadThread(ParkedThread):
            def start(self): raise OSError('cannot spawn timer')
        e = HumanSupportedHoldExecution(self.slave, clock=lambda: self.now, thread_factory=BadThread,
                                       stage_audio=FakeAudio(lambda: self.now))
        self.addCleanup(e.close); e.bind_profile(profile(), active=True)
        e.connect_cancel(lambda: self.cancelled.append(True))
        with self.assertRaises(OSError): e.on_start(self.now)
        self.assertTrue(self.cancelled)
        self.assertIsNone(e.cue_ns)

    def test_invalid_timer_scope_is_rejected_before_modifying_terminal_flags(self):
        before = os.get_blocking(self.slave)
        with patch('experiments.four_bus_human_hold.supervision.os.set_blocking') as changed:
            with self.assertRaisesRegex(ValueError, 'timer operating scope'):
                HumanSupportedHoldExecution(self.slave, timer_scope=None)
        changed.assert_not_called()
        self.assertIs(os.get_blocking(self.slave), before)

    def test_actual_thread_start_error_does_not_prevent_nonblocking_restoration(self):
        e = HumanSupportedHoldExecution(self.slave, clock=lambda: self.now,
                                       stage_audio=FakeAudio(lambda: self.now))
        self.addCleanup(e.close); e.bind_profile(profile(), active=True)
        e.connect_cancel(lambda: self.cancelled.append(True))
        with patch('experiments.four_bus_human_hold.supervision.threading.Thread.start',
                   side_effect=RuntimeError('thread start failure')):
            with self.assertRaises(RuntimeError): e.on_start(self.now)
        e.close()
        self.assertIsNotNone(e.failed)
        self.assertFalse(os.get_blocking(self.slave))  # Original supervisor owns this fd.

    def test_preparatory_voice_completion_requires_a_new_full_gain_cycle(self):
        self.cycle(1.2)
        self.assertEqual(self.audio.requests, ['prepare_ease'])
        self.cycle(1.3)
        self.assertIsNone(self.execution.cue_ns)
        self.now = 2_400_000_000; self.audio.finish('prepare_ease')
        self.cycle(1.41)  # Began before completion; this input is still pre-voice.
        self.assertNotIn('go', self.audio.requests)
        self.cycle(1.44, full_gain=False)
        self.assertNotIn('go', self.audio.requests)
        self.cycle(1.48)
        self.assertIn('go', self.audio.requests)
        self.assertEqual(self.execution.cue_ns, self.now)

    def test_missing_prepare_completion_never_opens_window(self):
        self.cycle(1.2)
        self.now += 450_000_000
        with self.assertRaisesRegex(RuntimeError, 'Owned audio'):
            self.execution.before_cycle(self.now)
        self.assertIsNone(self.execution.cue_ns)
        self.assertTrue(self.audio.cancelled)
        self.assertIn('abort', self.audio.requests)

    def test_audio_failure_overlap_and_noncausal_completion_cancel(self):
        self.cycle(1.2)
        self.audio.pending['prepare_ease'][2]('prepare_ease', 'ALSA failure')
        self.assertTrue(self.cancelled)
        self.assertIsNone(self.execution.cue_ns)
        e = self.new_execution()
        self.now = 2_200_000_000
        e.after_cycle_validated(self.now - 19_000_000, self.now, 'active', full_gain=True)
        e._audio_complete('prepare_ease', self.now, self.now + 1)
        self.assertIsNotNone(e.failed)
        self.assertIsNone(e.cue_ns)

    def test_go_start_after_runtime_abort_cannot_open_a_window(self):
        class DelayedGo(FakeAudio):
            def start(audio, stage, complete, failure, on_started=None):
                if stage == 'go':
                    audio.late_started = on_started
                    audio.requests.append(stage)
                else: super().start(stage, complete, failure, on_started)
        audio = DelayedGo(lambda: self.now)
        e = HumanSupportedHoldExecution(self.slave, clock=lambda: self.now,
                                       stage_audio=audio, thread_factory=ParkedThread)
        self.addCleanup(e.close); e.bind_profile(profile(), active=True)
        e.connect_cancel(lambda: self.cancelled.append(True)); e.on_start(1_000_000_000)
        self.now = 2_200_000_000
        e.after_cycle_validated(self.now - 19_000_000, self.now, 'active', full_gain=True)
        self.finish('prepare_ease', e); self.now += 40_000_000
        e.after_cycle_validated(self.now - 19_000_000, self.now, 'active', full_gain=True)
        self.assertIsNone(e.cue_ns)
        e.on_abort('CAN_REPLY_FAULT_BEFORE_QUEUED_GO')
        self.now += 1
        audio.late_started('go', self.now)
        self.assertIsNone(e.cue_ns)
        self.assertTrue(audio.cancelled)

    def test_go_launch_with_stale_full_gain_cycle_fails_closed(self):
        self.cycle(1.2); self.finish('prepare_ease')
        self.now += 40_000_000
        self.execution.last_full_gain_ns = self.now - 21_000_000
        self.execution.go_request_ns = self.now
        self.execution.audio_requests['go'] = self.now
        self.execution._audio_started('go', self.now)
        self.assertIsNone(self.execution.cue_ns)
        self.assertTrue(self.cancelled)

    def test_enter_during_resupport_voice_is_stale(self):
        self.open(); self.now = self.execution.ease_deadline_ns
        self.execution.before_cycle(self.now)
        os.write(self.master, b'\n')
        self.finish('resupport')
        self.now += 20_000_000
        self.assertFalse(self.execution.before_cycle(self.now))
        self.assertIsNone(self.execution.ack_ns)
        self.assertTrue(self.ack())

    def test_real_stage_durations_fit_but_zero_or_long_go_do_not(self):
        measured = audio_manifest()
        for name, value in (('brief', 23.704979), ('prepare_ease', 2.894771),
                            ('go', .1), ('resupport', 1.280208), ('abort', 2.588021)):
            measured['clips'][name]['duration_s'] = value
        e = HumanSupportedHoldExecution(self.slave, stage_audio=FakeAudio(lambda: self.now))
        self.addCleanup(e.close)
        with patch('experiments.four_bus_human_hold.supervision.supervisor_view', return_value=(profile(), measured)):
            e.bind_profile(profile(), active=True)
        for stage, duration in (('prepare_ease', 0.), ('go', .121), ('prepare_ease', 3.5)):
            with self.subTest(stage=stage, duration=duration):
                bad = audio_manifest(); bad['clips'][stage]['duration_s'] = duration
                if stage == 'prepare_ease' and duration > 1:
                    bad['clips']['resupport']['duration_s'] = 3.
                with patch('experiments.four_bus_human_hold.supervision.supervisor_view', return_value=(profile(), bad)):
                    with self.assertRaises(ValueError): e.bind_profile(profile(), active=True)


    def test_whole22ms_reply22ms_with_write18ms_is_accepted_at30hz(self):
        self.now = 2_200_000_000
        self.execution.after_cycle_validated(self.now-22_000_000, self.now, 'active',
            full_gain=True, oldest_input_ns=self.now-22_000_000,
            final_write_ns=self.now-4_000_000, last_reply_ns=self.now-1_000_000)
        self.assertIsNone(self.execution.failed)
        self.assertIn('prepare_ease', self.audio.requests)
        self.assertIsNone(self.execution.cue_ns)

    def test_exact20ms_write_deadline_cancels_before_easing(self):
        self.now = 2_200_000_000
        with self.assertRaisesRegex(RuntimeError, '20ms'):
            self.execution.after_cycle_validated(self.now-22_000_000, self.now, 'active',
                full_gain=True, oldest_input_ns=self.now-22_000_000,
                final_write_ns=self.now-2_000_000, last_reply_ns=self.now)
        self.assertTrue(self.cancelled)
        self.assertIsNone(self.execution.cue_ns)

    def test_reply_at30hz_deadline_and_noncausal_timestamps_reject(self):
        for label, input_delta, write_delta, reply_delta in (
                ('reply_deadline', 0, 18_000_000, 33_333_334),
                ('input_before_begin', -1, 18_000_000, 22_000_000),
                ('reply_before_write', 0, 18_000_000, 17_000_000)):
            with self.subTest(label=label):
                e=self.new_execution(); begin=2_170_000_000
                self.now=begin+max(reply_delta, 33_333_334)
                with self.assertRaises(RuntimeError):
                    e.after_cycle_validated(begin, self.now, 'active', full_gain=True,
                        oldest_input_ns=begin+input_delta, final_write_ns=begin+write_delta,
                        last_reply_ns=begin+reply_delta)
                self.assertIsNone(e.cue_ns)
                self.assertIsNotNone(e.failed)

    def test_audio_go_needs20ms_recent_full_gain_even_at30hz(self):
        self.cycle(1.2); self.finish('prepare_ease')
        self.cycle(1.44)
        self.assertIsNotNone(self.execution.cue_ns)
        # Failure callback and cancel remain latched even if a cycle budget is33ms.
        e=self.new_execution(); self.now=2_200_000_000
        e.last_full_gain_ns=self.now-20_000_001
        e.prepare_finished_ns=self.now-25_000_000
        e.audio_requests['go']=self.now
        e._audio_started('go',self.now)
        self.assertIsNone(e.cue_ns)
        self.assertIsNotNone(e.failed)


    def test_close_preserves_terminal_restore_failure_instead_of_closed_flag_success(self):
        self.open();self.resupport();self.ack()
        original=os.set_blocking
        def fail_restore(fd,state):
            if fd==self.slave and state is True:
                raise OSError('synthetic blocking restore failed')
            return original(fd,state)
        with patch('experiments.four_bus_human_hold.supervision.os.set_blocking',side_effect=fail_restore):
            report=self.execution.close()
        self.assertTrue(self.execution.closed)
        self.assertFalse(self.execution.restored)
        self.assertFalse(report['complete'])
        self.assertFalse(report['terminal'][str(self.slave)]['restored'])
        self.assertTrue(any(row['stage']=='terminal' for row in report['errors']))
        self.assertIs(self.execution.close(),report)
        decorated=self.execution.decorate_report({})['human_supported_trial']
        self.assertFalse(decorated['restoration_complete'])
        self.assertIsNotNone(decorated['failure'])

    def test_audio_close_error_does_not_skip_terminal_restore_or_hide_first_failure(self):
        self.open();self.resupport();self.ack()
        self.execution.failed='ORIGINAL_CONTROL_FAILURE'
        with patch.object(self.audio,'close',side_effect=RuntimeError('owned audio join failed')):
            report=self.execution.close()
        self.assertEqual(self.execution.failed,'ORIGINAL_CONTROL_FAILURE')
        self.assertFalse(self.execution.restored)
        self.assertFalse(report['audio']['closed'])
        self.assertTrue(all(row['restored'] for row in report['terminal'].values()))

    def test_successful_close_verifies_real_terminal_flags_and_reports_after_cleanup(self):
        self.open();self.resupport();self.ack()
        report=self.execution.close()
        self.assertTrue(self.execution.restored)
        self.assertTrue(report['complete'])
        self.assertTrue(report['audio']['closed'])
        self.assertTrue(all(row['restored'] for row in report['terminal'].values()))
        self.assertTrue(self.execution.decorate_report({})['human_supported_trial']['restoration_complete'])


class RecoveryTimerPlacementTests(unittest.TestCase):
    def test_linux_scope_reads_cpu0to3_and_restores_original_thread_mask(self):
        reads=iter(({4},{0,1,2,3},{4})); writes=[]
        with patch('experiments.four_bus_human_hold.supervision.os.sched_getaffinity',
                   side_effect=lambda _:next(reads),create=True), \
             patch('experiments.four_bus_human_hold.supervision.os.sched_setaffinity',
                   side_effect=lambda _,mask:writes.append(set(mask)),create=True):
            with linux_timer_scope() as record:
                self.assertEqual(record['cpu_mask'],[0,1,2,3])
                self.assertFalse(record['file_only_mock_readback'])
            self.assertTrue(record['restored'])
        self.assertEqual(writes,[{0,1,2,3},{4}])

    def test_failed_readback_still_restores_original_thread_mask(self):
        reads=iter(({4},{4},{4})); writes=[]
        with patch('experiments.four_bus_human_hold.supervision.os.sched_getaffinity',
                   side_effect=lambda _:next(reads),create=True), \
             patch('experiments.four_bus_human_hold.supervision.os.sched_setaffinity',
                   side_effect=lambda _,mask:writes.append(set(mask)),create=True):
            with self.assertRaisesRegex(RuntimeError,'readback'):
                with linux_timer_scope(): pass
        self.assertEqual(writes,[{0,1,2,3},{4}])

    def test_genuine_primary_proof_cannot_select_synthetic_timer_scope(self):
        master,slave=pty.openpty()
        try:
            e=HumanSupportedHoldExecution(slave,stage_audio=FakeAudio(lambda:1_000_000_000))
            try:
                p=profile();p['_genuine_primary_execution_proof']=True
                with patch('experiments.four_bus_human_hold.supervision.supervisor_view',
                           return_value=(p,audio_manifest())):
                    with self.assertRaisesRegex(ValueError,'source-owned Linux'):
                        e.bind_profile(p,active=True)
                self.assertFalse(e._active_bound)
            finally: e.close()
        finally: os.close(master);os.close(slave)


if __name__ == '__main__': unittest.main()
