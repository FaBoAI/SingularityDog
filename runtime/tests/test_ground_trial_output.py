"""Ground runner orchestration with synthetic plans and no physical I/O.

CLI tests mock both artifact loading and the shared output entry point. Direct
context fixtures exercise the timeline interface and never physical approval.
"""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import ground_trial_output as output
from singularitydog_hw.ground_trial_plan import template_ground_plan
from singularitydog_hw.policy_live_profile import SCHEMA_V1, SCHEMA_V2


def synthetic_profile():
    return {'schema':SCHEMA_V1,'profile_sha256': 'a'*64, 'assembly_id': 'SIMULATED_TEST_ONLY',
            'boot_id': 'SIMULATED_BOOT', 'motor_power_epoch': 'SIMULATED_EPOCH',
            'output_allowed': True, 'duration_s': 6., 'startup_duration_s': .5,
            'policy_ramp_s': .5, 'stop_duration_s': .4,
            'axes': {str(i): {'max_command_velocity_rad_s': .1,
                             'max_command_acceleration_rad_s2': 1.}
                     for i in range(1, 13)}}


def synthetic_plan(stage='stand'):
    plan = template_ground_plan(stage, synthetic_profile())
    # Test-double gate only. No reviewed physical artifact is written by these tests.
    plan.update(execution_plan_validated=True, output_allowed=False,
                physical_arming_required=True)
    return plan


class FakeModel:
    def __init__(self):
        self.calls = 0
        self.provenance = {'kind': 'SIMULATION_ONLY'}
        self.last_validation = {'valid': True}
        self.received = []
    def validate_inputs(self, *args):
        self.received.append(('validation', args))
        return 'synthetic_validation'
    def __call__(self, sample, imu, now_ns, *, command_override):
        self.calls += 1
        self.received.append((sample, imu, now_ns, command_override))
        return tuple(command_override)


class GroundTrialExecutionTests(unittest.TestCase):
    start_ns = 1_000_000_000

    def execution(self, stage='stand', **kwargs):
        self.messages = []
        self.cancelled = []
        self.clock_ns = self.start_ns
        context = output.GroundTrialExecution(synthetic_plan(stage), 'b'*64,
                                              execution_kind='simulation', emit=self.messages.append,
                                              clock=lambda: self.clock_ns,
                                              **kwargs)
        context.connect_cancel(lambda: self.cancelled.append(True))
        context.on_start(self.start_ns)
        return context

    def now(self, t):
        value = self.start_ns + round(t*1e9)
        self.clock_ns = max(self.clock_ns, value)
        return value

    def test_current_resupport_event_allows_scheduled_normal_shutdown(self):
        context = self.execution()
        self.assertFalse(context.before_cycle(self.now(2.5)))
        self.assertTrue(any('resupport_window_open' in row for row in self.messages))
        context.acknowledge_resupport(self.now(2.6))
        self.assertFalse(context.before_cycle(self.now(2.6)))
        self.assertTrue(context.before_cycle(self.now(5.46)))
        self.assertEqual(context.timeline.resupport_ack_s, 2.6)
        self.assertEqual(self.cancelled, [])

    def test_old_resupport_event_does_not_release_load_shutdown(self):
        context = self.execution()
        context.acknowledge_resupport(self.now(.5))
        context.before_cycle(self.now(2.5))
        with self.assertRaisesRegex(RuntimeError, 'resupport_not_confirmed'):
            context.before_cycle(self.now(5.46))
        self.assertEqual(self.cancelled, [True])

    def test_missing_resupport_event_immediately_cancels_at_deadline(self):
        context = self.execution('walk')
        context.before_cycle(self.now(2.5))
        with self.assertRaisesRegex(RuntimeError, 'resupport_not_confirmed'):
            context.before_cycle(self.now(5.46))
        self.assertEqual(self.cancelled, [True])
        self.assertEqual(context.timeline.command_at(5.46), (0., 0., 0.))

    def test_ack_racing_future_cycle_is_consumed_only_when_causal(self):
        context = self.execution()
        context.before_cycle(self.now(2.5))
        context.acknowledge_resupport(self.now(2.7))
        context.before_cycle(self.now(2.69))
        self.assertIsNone(context.timeline.resupport_ack_s)
        context.before_cycle(self.now(2.7))
        self.assertEqual(context.timeline.resupport_ack_s, 2.7)

    def test_future_ack_after_deadline_cannot_preapprove_recovery(self):
        context = self.execution()
        context.before_cycle(self.now(2.5))
        context.acknowledge_resupport(self.now(5.47))
        with self.assertRaises(RuntimeError):
            context.before_cycle(self.now(5.46))
        self.assertEqual(self.cancelled, [True])

    def test_scheduled_time_without_displayed_cue_never_authorizes_shutdown(self):
        context = self.execution()
        context.acknowledge_resupport(self.now(2.6))
        # Skip ahead: the scheduled window elapsed, but no request was displayed.
        with self.assertRaisesRegex(RuntimeError, 'resupport_not_confirmed'):
            context.before_cycle(self.now(5.46))
        self.assertIsNone(context.timeline.resupport_ack_s)
        self.assertFalse(any(e['key'] == 'OPERATOR_RESUPPORT_ACK' for e in context.events))

    def test_enter_during_slow_cue_output_is_not_fresh_confirmation(self):
        context = self.execution()
        def delayed_emit(text):
            self.messages.append(text)
            if text.startswith('resupport_window_open'):
                context.acknowledge_resupport(self.now(2.6))
                self.now(2.8)  # Output has only now returned to the runtime.
        context.emit = delayed_emit
        context.before_cycle(self.now(2.5))
        event = next(e for e in context.events if e['key'] == 'resupport_window_open')
        self.assertEqual(event['emitted_ns'], self.now(2.8))
        context.before_cycle(self.now(2.9))
        self.assertIsNone(context.timeline.resupport_ack_s)
        with self.assertRaises(RuntimeError):
            context.before_cycle(self.now(5.46))

    def test_only_event_strictly_after_visible_cue_and_later_step_is_accepted(self):
        context = self.execution()
        context.before_cycle(self.now(2.5))
        context.acknowledge_resupport(self.now(2.5))
        context.before_cycle(self.now(2.52))
        self.assertIsNone(context.timeline.resupport_ack_s)
        context.acknowledge_resupport(self.now(2.53))
        context.before_cycle(self.now(2.54))
        accepted = next(e for e in context.events if e['key'] == 'OPERATOR_RESUPPORT_ACK')
        self.assertGreater(accepted['monotonic_ns'], accepted['cue_emitted_ns'])
        self.assertGreater(accepted['accepted_step'], accepted['cue_step'])
        self.assertEqual(accepted['accepted_ns'], self.now(2.54))

    def test_replanned_same_open_time_reannounces_and_discards_old_ack(self):
        context = self.execution()
        context.before_cycle(self.now(2.5))
        context.acknowledge_resupport(self.now(2.51))
        context.before_cycle(self.now(2.5), stop_requested=True)
        # The shortened deadline is3.5, same opening2.5: a new visible cue is required.
        windows = [e for e in context.events if e['key'] == 'resupport_window_open']
        self.assertEqual(len(windows), 2)
        self.assertIsNone(context.timeline.resupport_ack_s)
        with self.assertRaises(RuntimeError):
            context.before_cycle(self.now(3.5))

    def test_emergency_cancellation_precedes_any_deadline_output(self):
        context = self.execution()
        context.before_cycle(self.now(2.5))
        def inspect_cancel(text):
            if text.startswith('normal_stop_deadline'):
                self.assertEqual(self.cancelled, [True])
        context.emit = inspect_cancel
        with self.assertRaises(RuntimeError):
            context.before_cycle(self.now(5.46))

    def test_new_visible_window_flushes_terminal_queue_and_python_buffer(self):
        context = self.execution()
        context._input_fd = 77  # Synthetic fd; every operation is mocked.
        context._input_buffer = b'resupported\n'
        context.acknowledge_resupport(self.now(2.4))
        with patch.object(output.termios, 'tcflush') as flush, \
             patch.object(output.os, 'read', side_effect=BlockingIOError):
            context.before_cycle(self.now(2.5))
        flush.assert_called_once_with(77, output.termios.TCIFLUSH)
        self.assertEqual(context._input_buffer, b'')
        self.assertIsNone(context.ack_ns)

    def test_failed_terminal_flush_immediately_cancels(self):
        context = self.execution()
        context._input_fd = 77
        with patch.object(output.termios, 'tcflush', side_effect=OSError('synthetic failure')), \
             patch.object(output.os, 'read', side_effect=BlockingIOError):
            with self.assertRaisesRegex(RuntimeError, 'fresh operator-input boundary'):
                context.before_cycle(self.now(2.5))
        self.assertEqual(self.cancelled, [True])
        self.assertEqual(context._reader_failed, 'TERMINAL_FLUSH_FAILED')

    def test_cue_flush_discards_old_enter_but_preserves_queued_emergency(self):
        for payload in (b'q\n', b'\nstop\n'):
            context = self.execution()
            context._input_fd = 77
            with patch.object(output.os, 'read', side_effect=[payload, BlockingIOError()]), \
                 patch.object(output.termios, 'tcflush') as flush:
                with self.subTest(payload=payload), self.assertRaises(RuntimeError):
                    context.before_cycle(self.now(2.5))
            self.assertEqual(self.cancelled, [True])
            self.assertEqual(context._reader_failed, 'OPERATOR_EMERGENCY')
            flush.assert_not_called()

    def test_reader_fileno_failure_is_reported_and_cancels_instead_of_dying_silently(self):
        class BrokenStream:
            def fileno(self):
                raise ValueError('synthetic closed stream')
        with self.assertRaisesRegex(RuntimeError, 'reader could not start'):
            self.execution(input_stream=BrokenStream())
        self.assertEqual(self.cancelled, [True])

    def test_reader_thread_start_failure_cancels_and_restores_terminal_mode(self):
        class SyntheticStream:
            def fileno(self):return 77
        with patch.object(output.os, 'get_blocking', return_value=True), \
             patch.object(output.os, 'set_blocking') as mode, \
             patch.object(output.threading, 'Thread') as thread:
            thread.return_value.start.side_effect = RuntimeError('synthetic thread failure')
            with self.assertRaisesRegex(RuntimeError, 'reader could not start'):
                self.execution(input_stream=SyntheticStream())
        self.assertEqual(self.cancelled, [True])
        self.assertEqual(mode.call_args_list[-1].args, (77, True))

    def test_reader_select_exception_eof_and_oversized_input_cancel(self):
        for kind in ('select_failure', 'eof', 'oversized'):
            context = self.execution()
            context._input_fd = 77
            error = OSError('synthetic select failure') if kind == 'select_failure' else None
            chunk = b'' if kind == 'eof' else b'x'*4097
            with patch.object(output.select, 'select', return_value=([77], [], []), side_effect=error), \
                 patch.object(output.os, 'read', return_value=chunk):
                context._input()
            with self.subTest(kind=kind):
                self.assertEqual(self.cancelled, [True])
                self.assertIsNotNone(context._reader_failed)
                with self.assertRaises(RuntimeError):
                    context.before_cycle(self.now(.5))

    def test_raw_terminal_q_cancels_and_received_enter_is_not_an_accepted_ack(self):
        context = self.execution()
        context._input_fd = 77
        with patch.object(output.select, 'select', return_value=([77], [], [])), \
             patch.object(output.os, 'read', return_value=b'\nq\n'):
            context._input()
        self.assertEqual(self.cancelled, [True])
        self.assertTrue(any(e['key'] == 'OPERATOR_RESUPPORT_ACK_RECEIVED' for e in context.events))
        self.assertFalse(any(e['key'] == 'OPERATOR_RESUPPORT_ACK' for e in context.events))

    def test_terminal_blocking_state_is_restored_after_report(self):
        context = self.execution()
        context._input_fd, context._input_was_blocking = 77, True
        with patch.object(output.os, 'set_blocking') as restore:
            context.decorate_report({'status': 'COMPLETE_SUPPORTED_OUTPUT'})
        restore.assert_called_once_with(77, True)

    def test_supported_stance_retains_support_and_needs_no_new_ack(self):
        context = self.execution('supported_stance')
        self.assertTrue(context.before_cycle(self.now(5.46)))
        self.assertEqual(self.cancelled, [])

    def test_early_normal_stop_skips_activation_and_still_demands_fresh_support(self):
        context = self.execution('walk')
        self.assertFalse(context.before_cycle(self.now(.5), stop_requested=True))
        self.assertEqual(context.timeline.command_at(1.25), (0., 0., 0.))
        context.before_cycle(self.now(1.5))
        context.acknowledge_resupport(self.now(1.6))
        context.before_cycle(self.now(1.6))
        self.assertTrue(context.before_cycle(self.now(2.5)))
        self.assertTrue(context._normal_requested)

    def test_live_model_receives_bounded_forward_only_command_and_original_inputs(self):
        context = self.execution('walk')
        model = FakeModel()
        self.assertIs(context.wrap_model(model), context)
        sample, imu = object(), object()
        observed = []
        for t in [i/100 for i in range(301)]:
            command = context(sample, imu, self.now(t))
            self.assertEqual(command[1:], (0., 0.))
            self.assertGreaterEqual(command[0], 0.)
            self.assertLessEqual(command[0], .02)
            observed.append(command[0])
        self.assertGreater(max(observed), 0)
        self.assertEqual(observed[0], 0)
        self.assertEqual(observed[-1], 0)
        self.assertIs(model.received[-1][0], sample)
        self.assertIs(model.received[-1][1], imu)
        self.assertEqual(context.calls, 301)
        self.assertEqual(context.provenance, model.provenance)
        self.assertEqual(context.validate_inputs(sample, imu, self.now(3.)), 'synthetic_validation')

    def test_simulation_report_cannot_become_physical_pass(self):
        context = self.execution()
        source = {'status': 'COMPLETE_SUPPORTED_OUTPUT', 'errors': [],
                  'motor_enable_sent': True, 'learned_targets_sent': True,
                  'stop_confirmed': True, 'deadline20ms_misses': 0}
        report = context.decorate_report(source)
        self.assertEqual(report['status'], 'COMPLETE_BOUNDED_GROUND_TRIAL')
        self.assertTrue(report['simulation_only'])
        self.assertEqual(report['execution_kind'], 'simulation')
        self.assertFalse(report['dependency_eligible'])
        self.assertEqual(report['physical_result'], 'UNREVIEWED')
        self.assertEqual(report['runtime_report']['status'], 'COMPLETE_SUPPORTED_OUTPUT')
        self.assertNotIn('scope', source)

    def test_success_and_abort_decoration_preserve_outer_and_nested_transport_settings(self):
        for schema,gap,window in ((SCHEMA_V1,600,3),(SCHEMA_V2,800,2)):
            for status in ('COMPLETE_SUPPORTED_OUTPUT','ABORTED'):
                with self.subTest(schema=schema,status=status):
                    context=self.execution()
                    pacing={'request_gap_us':gap,'request_window':window,'source_profile_schema':schema,
                            'emergency_stop_uses_same_gap':True}
                    errors=[] if status=='COMPLETE_SUPPORTED_OUTPUT' else ['synthetic runtime abort']
                    source={'status':status,'errors':errors,'transport_settings':pacing,
                            'actual_policy_output_20ms_verified':False,'stop_confirmed':status=='COMPLETE_SUPPORTED_OUTPUT'}
                    report=context.decorate_report(source)
                    expected='COMPLETE_BOUNDED_GROUND_TRIAL' if status=='COMPLETE_SUPPORTED_OUTPUT' else status
                    self.assertEqual(report['status'],expected)
                    self.assertEqual(report['runtime_report']['status'],status)
                    self.assertEqual(report['transport_settings'],pacing)
                    self.assertEqual(report['runtime_report']['transport_settings'],pacing)
                    self.assertFalse(report['actual_policy_output_20ms_verified'])
                    self.assertFalse(report['runtime_report']['actual_policy_output_20ms_verified'])
                    self.assertEqual(report['errors'],errors)
                    self.assertFalse(report['dependency_eligible'])
                    self.assertNotIn('scope',source)

    def test_fault_and_stop_ambiguity_are_preserved(self):
        context = self.execution()
        source = {'status': 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED',
                  'errors': ['synthetic missing STOP'], 'stop_confirmed': False}
        report = context.decorate_report(source)
        self.assertEqual(report['status'], source['status'])
        self.assertEqual(report['errors'], source['errors'])
        self.assertFalse(report['stop_confirmed'])
        self.assertFalse(report['dependency_eligible'])

    def test_base_bindings_and_duration_rechecked(self):
        for key, value in [('profile_sha256', 'c'*64), ('assembly_id', 'other'),
                           ('boot_id', 'reboot'), ('motor_power_epoch', 'replaced'),
                           ('duration_s', 7.), ('output_allowed', False)]:
            context = self.execution()
            profile = synthetic_profile()
            profile[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                context.bind_profile(profile, active=True)

    def test_context_cannot_restart_or_accept_implicit_execution_kind(self):
        context = self.execution()
        with self.assertRaises(RuntimeError):
            context.on_start(self.start_ns)
        for kind in ('', 'real', 'replay'):
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                output.GroundTrialExecution(synthetic_plan(), 'b'*64, execution_kind=kind)
        with self.assertRaises(ValueError):
            output.GroundTrialExecution({'execution_plan_validated': False}, 'b'*64)


class GroundTrialCliTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root/'synthetic-plan.json'
        # Raw file is unapproved; validator is mocked for CLI ownership tests.
        self.path.write_text(json.dumps(template_ground_plan('stand', synthetic_profile())))
        self.args = ['--ground-plan', str(self.path), '--profile', str(self.root/'synthetic-profile.json')]

    def invoke(self, flags=(), *, isatty=True, validated=True,profile_data=None):
        valid = synthetic_plan()
        valid['execution_plan_validated'] = validated
        stdout = io.StringIO()
        with patch.object(output, 'load_profile', return_value=profile_data or synthetic_profile()) as load, \
             patch('singularitydog_hw.ground_trial_plan.validate_ground_plan', return_value=valid) as validate, \
             patch('singularitydog_hw.policy_output.main', return_value=17) as execute, \
             patch.object(output.sys.stdin, 'isatty', return_value=isatty), \
             contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(io.StringIO()):
            result = output.main(self.args+list(flags))
        return result, stdout.getvalue(), load, validate, execute

    def test_default_plan_opens_no_physical_output_even_with_validated_plan(self):
        result, text, load, validate, execute = self.invoke()
        self.assertEqual(result, 0)
        payload = json.loads(text)
        self.assertEqual(payload['status'], 'PLAN_ONLY')
        self.assertFalse(payload['output_allowed'])
        self.assertFalse(payload['hardware_opened'])
        execute.assert_not_called()
        self.assertFalse(load.call_args.kwargs['require_approved'])
        self.assertFalse(validate.call_args.kwargs['require_approved'])

    def pacing_profile(self,schema,*,gap_us=600,window=3):
        value=synthetic_profile();value['schema']=schema
        if schema==SCHEMA_V2:value.update(request_gap_us=gap_us,request_window=window)
        return value

    def expected_transport(self,schema,*,gap_us=600,window=3):
        return {'request_gap_us':gap_us,'request_window':window,'source_profile_schema':schema,
                'emergency_stop_uses_same_gap':True}

    def test_profile_bound_pacing_is_reported_in_ground_plan_without_execution(self):
        for schema,gap,window in ((SCHEMA_V1,600,3),(SCHEMA_V2,800,2)):
            profile=self.pacing_profile(schema,gap_us=gap,window=window)
            for flags in ((),('--request-gap-us',str(gap),'--request-window',str(window))):
                with self.subTest(schema=schema,flags=flags):
                    result,text,load,validate,execute=self.invoke(flags,profile_data=profile)
                    payload=json.loads(text)
                    self.assertEqual(result,0);self.assertEqual(payload['status'],'PLAN_ONLY')
                    self.assertEqual(payload['transport_settings'],self.expected_transport(schema,gap_us=gap,window=window))
                    self.assertFalse(payload['hardware_opened']);self.assertFalse(payload['output_allowed'])
                    execute.assert_not_called()
                    self.assertFalse(load.call_args.kwargs['require_approved'])
                    self.assertFalse(validate.call_args.kwargs['require_approved'])

    def test_matching_v1_and_v2_transport_flags_are_forwarded_once_as_strings(self):
        for schema,gap,window in ((SCHEMA_V1,600,3),(SCHEMA_V2,800,2)):
            with self.subTest(schema=schema):
                profile=self.pacing_profile(schema,gap_us=gap,window=window)
                result,_,load,validate,execute=self.invoke(
                    ['--execute-ground','--catch-ready','--video-ready','--roles-ready',
                     '--support-in-place','--cutoff-ready','--request-gap-us',str(gap),
                     '--request-window',str(window)],profile_data=profile)
                self.assertEqual(result,17)
                load.assert_called_once();validate.assert_called_once();execute.assert_called_once()
                self.assertTrue(load.call_args.kwargs['require_approved'])
                self.assertTrue(validate.call_args.kwargs['require_approved'])
                forwarded=execute.call_args.args[0]
                for flag,value in (('--request-gap-us',gap),('--request-window',window)):
                    self.assertEqual(forwarded.count(flag),1)
                    self.assertEqual(forwarded[forwarded.index(flag)+1],str(value))
                self.assertTrue(all(type(value) is str for value in forwarded))
                self.assertNotIn('--execute-ground',forwarded)
                self.assertIn('--execute-supported',forwarded)
                self.assertEqual(execute.call_args.kwargs['execution'].execution_kind,'hardware')

    def test_single_thread_math_is_forwarded_only_when_selected(self):
        selected=['--execute-ground','--catch-ready','--video-ready','--roles-ready',
                  '--support-in-place','--cutoff-ready','--single-thread-math']
        result,_,_,_,execute=self.invoke(selected)
        self.assertEqual(result,17)
        self.assertEqual(execute.call_args.args[0].count('--single-thread-math'),1)

        result,_,_,_,execute=self.invoke(selected[:-1])
        self.assertEqual(result,17)
        self.assertNotIn('--single-thread-math',execute.call_args.args[0])

    def test_invalid_pacing_is_rejected_before_ground_document_profile_or_shared_runner(self):
        invalid=(('--request-gap-us','599'),('--request-gap-us','5001'),('--request-gap-us','not-an-int'),
                 ('--request-window','0'),('--request-window','4'),('--request-window','not-an-int'))
        for execute in (False,True):
            for flag,value in invalid:
                with self.subTest(execute=execute,flag=flag,value=value), \
                     patch.object(output,'read_document') as read, \
                     patch.object(output,'load_profile') as load, \
                     patch('singularitydog_hw.ground_trial_plan.validate_ground_plan') as validate, \
                     patch('singularitydog_hw.policy_output.main') as shared, \
                     contextlib.redirect_stderr(io.StringIO()):
                    flags=['--execute-ground','--catch-ready','--video-ready','--roles-ready'] if execute else []
                    with self.assertRaises(SystemExit) as raised:output.main(self.args+flags+[flag,value])
                    self.assertEqual(raised.exception.code,2)
                for method in (read,load,validate,shared):method.assert_not_called()

    def test_bounded_pacing_mismatch_is_rejected_before_ground_validation_or_shared_runner(self):
        for schema,gap,window in ((SCHEMA_V1,600,3),(SCHEMA_V2,800,2)):
            profile=self.pacing_profile(schema,gap_us=gap,window=window)
            for execute in (False,True):
                for flag,value in (('--request-gap-us','800' if gap==600 else '600'),
                                   ('--request-window','2' if window==3 else '3')):
                    with self.subTest(schema=schema,execute=execute,flag=flag), \
                         patch.object(output,'load_profile',return_value=profile) as load, \
                         patch('singularitydog_hw.ground_trial_plan.validate_ground_plan') as validate, \
                         patch('singularitydog_hw.policy_output.main') as shared, \
                         contextlib.redirect_stderr(io.StringIO()):
                        flags=['--execute-ground','--catch-ready','--video-ready','--roles-ready'] if execute else []
                        with self.assertRaises(SystemExit) as raised:output.main(self.args+flags+[flag,value])
                        self.assertEqual(raised.exception.code,2)
                    load.assert_called_once_with(Path(self.args[self.args.index('--profile')+1]),require_approved=execute)
                    validate.assert_not_called();shared.assert_not_called()

    def test_current_catch_video_roles_are_required_before_shared_output(self):
        complete = ['--execute-ground', '--catch-ready', '--video-ready', '--roles-ready']
        for missing in ('--catch-ready', '--video-ready', '--roles-ready'):
            with self.subTest(missing=missing), \
                 patch('singularitydog_hw.policy_output.main') as execute, \
                 self.assertRaises(SystemExit):
                self.invoke([v for v in complete if v != missing])
            execute.assert_not_called()

    def test_noninteractive_terminal_is_rejected_before_any_output(self):
        with self.assertRaises(SystemExit):
            self.invoke(['--execute-ground', '--catch-ready', '--video-ready', '--roles-ready'],
                        isatty=False)

    def test_explicit_ground_execute_passes_reviewed_context_to_shared_runner(self):
        result, _, load, validate, execute = self.invoke(
            ['--execute-ground', '--catch-ready', '--video-ready', '--roles-ready',
             '--support-in-place', '--cutoff-ready', '--trial-id', 'UNIT_TEST_TRIAL'])
        self.assertEqual(result, 17)
        self.assertTrue(load.call_args.kwargs['require_approved'])
        self.assertTrue(validate.call_args.kwargs['require_approved'])
        context = execute.call_args.kwargs['execution']
        self.assertEqual(context.trial_id, 'UNIT_TEST_TRIAL')
        self.assertEqual(context.execution_kind, 'hardware')
        self.assertIsNotNone(context.input_stream)
        forwarded = execute.call_args.args[0]
        self.assertIn('--execute-supported', forwarded)
        self.assertIn('--support-in-place', forwarded)
        self.assertNotIn('--execute-ground', forwarded)

    def test_supported_entry_flag_cannot_bypass_ground_arming(self):
        with self.assertRaises(SystemExit):
            self.invoke(['--execute-supported'])

    def test_unknown_and_abbreviated_flags_never_reach_shared_entry_point(self):
        for flags in (['--execute-s'], ['--execute-supported=true'], ['--execute-g'],
                      ['--force'], ['--skip-catch'], ['--prof', 'other.json']):
            with self.subTest(flags=flags), self.assertRaises(SystemExit):
                self.invoke(flags)


if __name__ == '__main__':
    unittest.main()
