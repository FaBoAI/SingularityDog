"""Delayed optional reporting never runs while commands or owners are active.

All sessions and timestamps are synthetic in-memory fixtures; these tests do
not qualify physical deadlines, source admission, torque or an actual run.
"""
import threading
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_output_runtime as runtime
from test_policy_output_runtime import (FakeIMU, FakeSession, SimulatedClock,
    encode_motion, profile, causal_semantic_wait)


class DeferredCycleEvidenceTests(unittest.TestCase):
    def run_case(self, *, formatting=None, high_torque=False, budget=False,
                 feedback_delay_ns=0):
        clock=SimulatedClock()
        sessions={'front':FakeSession(1,clock=clock,high_returned_torque=high_torque,
                                     torque_fault_after=12),
                  'rear':FakeSession(7,clock=clock)}
        cancelled=threading.Event();data=profile()
        if budget:
            data.update(schema=live.SCHEMA_V3,scope='supported_characterization_only',
                telemetry_cadence=live.CADENCE_PRE_ENABLE,
                cadence_source_sha256=live.cadence_source_hashes(),
                hard_cycle_ms=20.,max_sample_age_ms=20.,max_sample_gap_ms=21.,
                post_reply_deadline_policy=dict(mode='bounded_post_reply_v1',max_lateness_ms=1.,
                    max_consecutive_misses=1,rolling_window_cycles=100,max_misses_per_window=1),
                _post_reply_validation_token=live._POST_REPLY_VALIDATION_TOKEN)
        original=runtime._DeferredCycleEvidence.materialize
        rows=[];stop_states=[];original_sample=runtime.feedback_sample
        def materialize(row):
            # The diagnostic path starts only after cancellation, final STOP and
            # owner shutdown. An arbitrary expensive formatter cannot alter the
            # original native writes, input ages or cycle end timestamps.
            self.assertTrue(cancelled.is_set())
            self.assertTrue(all(len(s.stop_times)==1 and not s.enabled for s in sessions.values()))
            stop_states.append(tuple(tuple(s.stop_times) for s in sessions.values()))
            rows.append(row)
            if formatting is not None:formatting(row,clock)
            return original(row)
        def feedback_sample(*args,**kwargs):
            value=original_sample(*args,**kwargs)
            previous=kwargs.get('previous') or {}
            if feedback_delay_ns and any(name=='voltage' for _,name in previous):
                clock.advance(feedback_delay_ns)
            return value
        with patch.object(runtime,'wait',side_effect=causal_semantic_wait), \
             patch.object(runtime._DeferredCycleEvidence,'materialize',materialize), \
             patch.object(runtime,'feedback_sample',side_effect=feedback_sample):
            report=runtime.run_supported_policy(data,sessions,FakeIMU(clock=clock),
                lambda *_:(.04,)*12,cancel_io=cancelled.set,encode_motion=encode_motion,
                clock=clock,sleep=clock.sleep)
        return report,sessions,rows,stop_states

    def test_one_second_per_row_optional_formatting_does_not_delay_control(self):
        report,sessions,rows,_=self.run_case(formatting=lambda row,clock:clock.advance(1_000_000_000))
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertTrue(report['normal_ramp_completed']);self.assertTrue(report['stop_confirmed'])
        self.assertGreater(len(rows),10)
        self.assertEqual(len(rows),len(report['cycles']))
        self.assertEqual(report['deadline20ms_misses'],0)
        self.assertTrue(all(r['iteration_ms']<20 for r in report['cycles']))
        self.assertTrue(all(s.positive_gain_writes>0 for s in sessions.values()))

    def test_formatting_exception_occurs_after_final_zero_gain_and_stop(self):
        captured=[]
        def fail(row,clock):
            captured.append(row)
            self.assertEqual(row.phase,'starting')
            raise ValueError('Synthetic optional serialization failure after STOP')
        with self.assertRaisesRegex(ValueError,'optional serialization failure'):
            self.run_case(formatting=fail)
        # The injected formatter itself asserts cancelled, disabled and both
        # owners' STOP before throwing. No formatter is called in active work.
        self.assertEqual(len(captured),1)

    def test_full_float_metrics_and_reply_provenance_match_retained_raw_values(self):
        report,_,rows,_=self.run_case()
        for raw,formatted in zip(rows,report['cycles']):
            self.assertEqual(formatted['iteration_ms'],(raw.end_ns-raw.begin_ns)/1e6)
            self.assertEqual(formatted['post_output_processing_ms'],
                (raw.end_ns-raw.output_exchange_return_ns)/1e6)
            self.assertEqual(formatted['oldest_input_to_final_host_write_ms'],
                (raw.final_write_ns-raw.first_ns)/1e6)
            self.assertEqual(formatted['acquisition_ms'],(raw.acquired_ns-raw.first_ns)/1e6)
            self.assertEqual(formatted['inference_ms'],(raw.computed_ns-raw.acquired_ns)/1e6)
            self.assertEqual(formatted['policy_and_envelope_ms'],(raw.encoded_ns-raw.acquired_ns)/1e6)
            self.assertEqual(formatted['effective_policy_weight'],raw.weight)
            self.assertLessEqual(formatted['output_reply_end_ns'],formatted['output_exchange_return_ns'])
            self.assertLessEqual(formatted['output_exchange_return_ns'],formatted['end_ns'])
            # Find every actual output reply corresponding to this saved tick.
            wires=[record for entry in report['journal']
                if entry['phase'] in {'startup_hold','policy_output','graceful_stop'}
                for record in entry['records']
                if raw.begin_ns<=record['start_ns']<=raw.output_reply_end_ns]
            self.assertEqual(len(wires),12)
            self.assertTrue(all(row['written']==row['received']==17 for row in wires))
            self.assertEqual(max(row['finish_ns'] for row in wires),raw.final_write_ns)
            self.assertEqual(max(row['received_ns'] for row in wires),raw.output_reply_end_ns)

    def test_v1_admission_decision_and_counts_survive_post_stop_formatting(self):
        report,_,rows,_=self.run_case(budget=True,
            formatting=lambda row,clock:clock.advance(2_000_000_000))
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertEqual(report['post_reply_deadline_allowance_uses'],0)
        self.assertEqual(report['post_reply_deadline_rejections'],[])
        for raw,row in zip(rows,report['cycles']):
            self.assertEqual(row['post_reply_deadline'],raw.post_reply_deadline)
            self.assertTrue(row['post_reply_deadline']['accepted'])
            self.assertEqual(row['end_ns'],row['post_reply_deadline']['checked_ns'])

    def test_measured_torque_failure_keeps_stop_and_original_error(self):
        baseline,_,_,_=self.run_case(high_torque=True)
        delayed,_,_,_=self.run_case(high_torque=True,
            formatting=lambda row,clock:clock.advance(1_000_000_000))
        self.assertEqual(baseline['status'],delayed['status'])
        self.assertEqual(baseline['errors'],delayed['errors'])
        self.assertTrue(any('torque' in error for error in baseline['errors']))
        self.assertTrue(delayed['stop_confirmed']);self.assertFalse(delayed['normal_ramp_completed'])

    def test_actual_returned_feedback_validation_delay_is_not_deferred_or_hidden(self):
        report,_,_,_=self.run_case(feedback_delay_ns=30_000_000)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(report['stop_confirmed']);self.assertFalse(report['normal_ramp_completed'])
        self.assertTrue(any('timing misses' in error for error in report['errors']),report['errors'])
        self.assertGreaterEqual(report['cycles'][0]['post_output_processing_ms'],30.)
        self.assertTrue(report['cycles'][0]['deadline20ms_missed'])


if __name__=='__main__':unittest.main()
