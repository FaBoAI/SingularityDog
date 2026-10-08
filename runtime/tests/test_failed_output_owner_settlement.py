"""Offline failure evidence: settled originals never grant timely admission."""
from concurrent.futures import Future, ThreadPoolExecutor
import builtins
import ctypes as C
import threading
import time
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import native_diagnostic_transport as diagnostic
from singularitydog_hw import native_pipeline_benchmark as bench


class FailedOutputOwnerSettlementTests(unittest.TestCase):
    def raw(self, marker):
        rows=(active.Record*6)()
        for index,row in enumerate(rows):
            row.tx[:]=bytes([marker+index])*17
            row.rx[:]=bytes([marker+index+12])*17
            row.start_ns=100+index;row.finish_ns=200+index
            row.read_start_ns=300+index;row.received_ns=400+index
            row.deadline_ns=500;row.written=row.received=17
        stats=active.Stats()
        stats.begin_ns=90;stats.end_ns=410
        stats.bytes=102;stats.writes=6;stats.reads=3;stats.rejected_total=9
        return rows,stats

    def future(self,raw):
        original=Future()
        original.set_result((raw,{},205,405))
        return original

    def settle(self,futures,*,clock=None):
        primary=TimeoutError('Output join coordinator deadline')
        proof={'output_join_deadline_ns':500,
               'output_join_failure_proof':{'reason':type(primary).__name__+': '+str(primary)}}
        record={'output':{}}
        returned=bench._settle_failed_decoded_output_owners(
            futures,record,primary,proof,clock or Mock(side_effect=[600,700]))
        self.assertIs(returned,primary)
        self.assertEqual(proof['output_join_deadline_ns'],500)
        self.assertEqual(str(primary),'Output join coordinator deadline')
        return record,proof

    def assert_no_admission(self,settlement):
        self.assertIs(settlement['cleanup_only'],True)
        self.assertIs(settlement['output_allowed'],False)
        self.assertIs(settlement['timing_admission_eligible'],False)
        self.assertIs(settlement['capture_is_deadline_decision_time'],False)
        self.assertIs(settlement['future_publication_time_inferred'],False)
        self.assertIs(settlement['native_reply_time_inferred'],False)

    def assert_raw(self,retained,original):
        self.assertIs(retained[0],original[0])
        self.assertEqual(bytes(retained[0]),bytes(original[0]))
        self.assertEqual(bytes(retained[1]),bytes(original[1])[:C.sizeof(diagnostic.Stats)])
        self.assertEqual(retained[1].rejected_total,original[1].rejected_total)

    def test_late_successful_original_results_keep_raw_without_deadline_admission(self):
        raw={scope:self.raw(marker) for scope,marker in (('front',1),('rear',30))}
        futures={scope:self.future(value) for scope,value in raw.items()}
        record,proof=self.settle(futures)
        settlement=proof['output_owner_settlement'];self.assert_no_admission(settlement)
        self.assertEqual(settlement['primary_error'],'TimeoutError: Output join coordinator deadline')
        for scope,timestamp in (('front',600),('rear',700)):
            self.assert_raw(record['output'][scope],raw[scope])
            owner=settlement['owners'][scope]
            self.assertEqual(owner['outcome'],'SUCCESS')
            self.assertEqual(owner['settled_monotonic_ns'],timestamp)
            self.assertGreater(timestamp,settlement['original_deadline_ns'])
            self.assertEqual(owner['raw_origin'],'original_future_result')
            self.assertIs(owner['raw_retained'],True)

    def test_decode_exception_attached_raw_is_distinct_and_primary_timeout_unchanged(self):
        raw_front=self.raw(1);raw_rear=self.raw(30)
        error=ValueError('actual owner decode error')
        error.records,error.stats=raw_rear
        rear=Future();rear.set_exception(error)
        record,proof=self.settle({'front':self.future(raw_front),'rear':rear})
        self.assert_raw(record['output']['front'],raw_front)
        self.assert_raw(record['output']['rear'],raw_rear)
        owner=proof['output_owner_settlement']['owners']['rear']
        self.assertEqual(owner['outcome'],'EXCEPTION')
        self.assertEqual(owner['exception_type'],'ValueError')
        self.assertEqual(owner['exception_message'],'actual owner decode error')
        self.assertEqual(owner['raw_origin'],'original_future_exception')
        self.assertIs(owner['raw_retained'],True)
        self.assertIs(rear.exception(),error)
        self.assert_no_admission(proof['output_owner_settlement'])

    def test_pending_original_is_joined_after_expired_deadline_only_for_cleanup(self):
        raw=self.raw(1);started=threading.Event();release=threading.Event()
        primary=TimeoutError('Output join coordinator deadline')
        proof={'output_join_deadline_ns':time.monotonic_ns()}
        record={'output':{}}
        def owner():
            started.set()
            if not release.wait(1):raise RuntimeError('fixture release missing')
            return raw,{},205,405
        with ThreadPoolExecutor(max_workers=1) as pool:
            future=pool.submit(owner)
            self.assertTrue(started.wait(1));self.assertFalse(future.done())
            timer=threading.Timer(.01,release.set);timer.start()
            try:
                self.assertIs(bench._settle_failed_decoded_output_owners(
                    {'front':future},record,primary,proof,time.monotonic_ns),primary)
            finally:
                release.set();timer.join()
        evidence=proof['output_owner_settlement'];self.assert_no_admission(evidence)
        self.assertGreater(evidence['owners']['front']['settled_monotonic_ns'],
                           evidence['original_deadline_ns'])
        self.assertEqual(evidence['owners']['front']['outcome'],'SUCCESS')
        self.assert_raw(record['output']['front'],raw)

    def test_exception_without_current_raw_never_inherits_prior_output(self):
        prior=self.raw(1);failed=Future();failed.set_exception(RuntimeError('before native'))
        current=self.raw(30)
        record,proof=self.settle({'front':failed,'rear':self.future(current)})
        self.assertNotIn('front',record['output'])
        owner=proof['output_owner_settlement']['owners']['front']
        self.assertEqual(owner['outcome'],'EXCEPTION');self.assertIsNone(owner['raw_origin'])
        self.assertIs(owner['raw_retained'],False)
        self.assertNotEqual(bytes(prior[0]),bytes(record['output']['rear'][0]))

    def test_cancelled_original_settles_other_owner_and_retains_primary(self):
        front=Future();self.assertTrue(front.cancel())
        raw=self.raw(30)
        record,proof=self.settle({'front':front,'rear':self.future(raw)})
        owner=proof['output_owner_settlement']['owners']['front']
        self.assertEqual(owner['outcome'],'CANCELLED')
        self.assertEqual(owner['exception_type'],'CancelledError')
        self.assertIs(owner['raw_retained'],False)
        self.assert_raw(record['output']['rear'],raw)

    def test_bad_observation_clock_cannot_hide_timeout_or_skip_other_owner(self):
        front=self.raw(1);rear=self.raw(30)
        record,proof=self.settle({'front':self.future(front),'rear':self.future(rear)},
                                clock=Mock(side_effect=[RuntimeError('clock failure'),700]))
        owner=proof['output_owner_settlement']['owners']['front']
        self.assertIsNone(owner['settled_monotonic_ns'])
        self.assertEqual(owner['capture_error_type'],'RuntimeError')
        self.assert_raw(record['output']['front'],front);self.assert_raw(record['output']['rear'],rear)

    def test_raw_conversion_failure_does_not_relabel_original_success_or_skip_peer(self):
        front=self.raw(1);rear=self.raw(30)
        from singularitydog_hw import private_seven_request_bridge as bridge
        original=bridge.diagnostic_result
        def convert(records,stats):
            if records is front[0]:raise ValueError('raw copy failure')
            return original(records,stats)
        with patch.object(bridge,'diagnostic_result',side_effect=convert):
            record,proof=self.settle({'front':self.future(front),'rear':self.future(rear)})
        owner=proof['output_owner_settlement']['owners']['front']
        self.assertEqual(owner['outcome'],'SUCCESS')
        self.assertEqual(owner['raw_retention_error_type'],'ValueError')
        self.assertIs(owner['raw_retained'],False)
        self.assert_raw(record['output']['rear'],rear)

    def test_submit_failure_reason_remains_primary_with_no_submitted_owners(self):
        primary=RuntimeError('original submission rejected')
        record={'output':{}};proof={'output_join_deadline_ns':500};clock=Mock()
        self.assertIs(bench._settle_failed_decoded_output_owners({},record,primary,proof,clock),primary)
        self.assertEqual(proof['output_owner_settlement']['primary_error'],
                         'RuntimeError: original submission rejected')
        self.assertEqual(proof['output_owner_settlement']['owners'],{})
        clock.assert_not_called()

    def test_converter_import_failure_does_not_skip_either_original_settlement(self):
        futures={scope:self.future(self.raw(marker))
                 for scope,marker in (('front',1),('rear',30))}
        taken=[];original_result=Future.result;original_import=builtins.__import__
        def result(future,*args,**kwargs):
            taken.append(future)
            return original_result(future,*args,**kwargs)
        def load(name,*args,**kwargs):
            if name=='private_seven_request_bridge':raise ImportError('converter unavailable')
            return original_import(name,*args,**kwargs)
        with patch.object(Future,'result',result),patch.object(builtins,'__import__',load):
            record,proof=self.settle(futures)
        self.assertEqual(taken,list(futures.values()))
        self.assertEqual(record['output'],{})
        for owner in proof['output_owner_settlement']['owners'].values():
            self.assertEqual(owner['outcome'],'SUCCESS')
            self.assertIs(owner['raw_retained'],False)
            self.assertEqual(owner['raw_retention_error_type'],'ImportError')


if __name__=='__main__':unittest.main()
