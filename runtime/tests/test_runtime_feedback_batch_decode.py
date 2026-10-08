"""Runtime codec selection and exact legacy fallback; no hardware or native build."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

from singularitydog_hw import native_active_transport as native
from singularitydog_hw import policy_output_runtime as runtime
from test_policy_output_runtime import FakeSession,encode_motion


class RuntimeFeedbackBatchDecodeTests(unittest.TestCase):
    def feedback(self):
        return FakeSession(1).exchange([encode_motion(mid,.1,1.,.1) for mid in range(1,7)])

    def test_selected_batch_returns_same_original_rows_without_python_frame_copies(self):
        result=self.feedback();expected=runtime.decode_records(result)
        decoder=SimpleNamespace(available=True,decode=Mock(return_value=expected))
        with patch.object(runtime,'_fixed_record_frame',side_effect=AssertionError('Legacy frame copy')):
            self.assertIs(runtime.decode_records(result,feedback_decoder=decoder,first_id=1),expected)
        decoder.decode.assert_called_once_with(result[0],1)

    def test_optional_absent_capability_preserves_codec_without_native_result(self):
        result=self.feedback();expected=runtime.decode_records(result)
        decoder=SimpleNamespace(available=False,decode=Mock(return_value=None))
        self.assertEqual(runtime.decode_records(result,feedback_decoder=decoder,first_id=1),expected)
        decoder.decode.assert_called_once_with(result[0],1)

    def test_unsupported_native_batch_falls_back_to_same_legacy_values(self):
        result=self.feedback();expected=runtime.decode_records(result)
        decoder=SimpleNamespace(available=True,decode=Mock(return_value=None))
        actual=runtime.decode_records(result,feedback_decoder=decoder,first_id=1)
        self.assertEqual(actual,expected)
        self.assertEqual({key:(value[1],value[2]) for key,value in actual.items()},
                         {key:(value[1],value[2]) for key,value in expected.items()})

    def test_invalid_batches_keep_exact_legacy_exception_after_native_declines(self):
        for defect in ('noncausal','incomplete','invalid_frame','wrong_source','duplicate'):
            result=self.feedback();records=result[0]
            if defect=='noncausal':records[0].received_ns=records[0].deadline_ns
            if defect=='incomplete':records[0].received=0
            if defect=='invalid_frame':records[0].rx[0]=ord('X')
            if defect=='wrong_source':records[0].rx[4]^=8
            if defect=='duplicate':records[1].tx[:]=records[0].tx;records[1].rx[:]=records[0].rx
            def failure(**kwargs):
                try:runtime.decode_records(result,**kwargs)
                except BaseException as error:return type(error),str(error)
                self.fail('Invalid motor transaction accepted')
            decoder=SimpleNamespace(available=True,decode=Mock(return_value=None))
            with self.subTest(defect=defect):
                self.assertEqual(failure(feedback_decoder=decoder,first_id=1),failure())

    def test_decoder_failure_cannot_be_hidden_as_legacy_success(self):
        error=ValueError('Native codec ABI binding changed')
        decoder=SimpleNamespace(available=True,decode=Mock(side_effect=error))
        with self.assertRaises(ValueError) as caught:
            runtime.decode_records(self.feedback(),feedback_decoder=decoder,first_id=1)
        self.assertIs(caught.exception,error)

    def test_default_and_generic_native_pair_fakes_do_not_select_native_codec(self):
        for selected in (False,True):
            sessions={'front':FakeSession(1),'rear':FakeSession(7)}
            pair=Mock()
            with patch.object(native,'ActivePhasePair',return_value=pair), \
                 patch.object(native,'NativeFeedbackBatchDecoder') as decoder_factory:
                workers=runtime.BusWorkers(sessions,lambda:None,native_phase_pair=selected)
                try:
                    self.assertIsNone(workers.native_feedback_decoders)
                    decoder_factory.assert_not_called()
                finally:workers.close()

    def test_rejected_optional_decoder_closes_new_native_pair_and_executors(self):
        sessions={scope:object.__new__(native.ActiveSession) for scope in ('front','rear')}
        for session in sessions.values():session.lib=object()
        pair=Mock();pools=[Mock(),Mock()]
        error=ValueError('Incomplete native feedback codec ABI')
        with patch.object(native,'ActivePhasePair',return_value=pair), \
             patch.object(native,'NativeFeedbackBatchDecoder',side_effect=error), \
             patch.object(runtime,'ThreadPoolExecutor',side_effect=pools):
            with self.assertRaises(ValueError) as caught:
                runtime.BusWorkers(sessions,lambda:None,native_phase_pair=True)
        self.assertIs(caught.exception,error);pair.close.assert_called_once_with()
        for pool in pools:pool.shutdown.assert_called_once_with(wait=True,cancel_futures=False)


if __name__=='__main__':unittest.main()
