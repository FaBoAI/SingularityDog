"""Genuine socketpair publication checks; no CAN device or target approval."""
import copy
from concurrent.futures import Future, ThreadPoolExecutor
import ctypes as C
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw import native_pipeline_benchmark as benchmark
import test_disabled_native_pair_candidate as fixture


class DisabledPairDirectPublicationTests(unittest.TestCase):
    # Reuse only the socket/placement fixture, not its inherited test suite.
    setUpClass = classmethod(fixture.DisabledNativePairCandidateTests.setUpClass.__func__)
    tearDownClass = classmethod(fixture.DisabledNativePairCandidateTests.tearDownClass.__func__)
    setUp = fixture.DisabledNativePairCandidateTests.setUp
    tearDown = fixture.DisabledNativePairCandidateTests.tearDown
    create = fixture.DisabledNativePairCandidateTests.create
    batches = fixture.DisabledNativePairCandidateTests.batches
    device = fixture.DisabledNativePairCandidateTests.device

    def ready(self):
        candidate = self.create(); candidate.configure_owners()
        for scope in self.peers: self.device(scope, 6)
        return candidate

    def test_normalization_runs_once_on_coordinator_before_original_future_publication(self):
        candidate = self.ready(); calls = []; published = []
        normalizer = benchmark._disabled_diagnostic_result
        original_submit = active.ActivePhasePair.submit
        def normalize(records, stats, **kwargs):
            calls.append((threading.get_ident(), records, stats, bytes(records), bytes(stats)))
            return normalizer(records, stats, **kwargs)
        def submit(pair, *args, **kwargs):
            self.assertTrue(callable(kwargs['result_transform']))
            result = original_submit(pair, *args, **kwargs); published.append(result)
            return result
        with patch.object(benchmark, '_disabled_diagnostic_result', normalize), \
             patch.object(active.ActivePhasePair, 'submit', submit):
            result = candidate.exchange_stop_proxy(self.batches(),
                deadline_ns=time.monotonic_ns()+benchmark.PERIOD_NS)
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(call[0] != threading.get_ident() for call in calls))
        pair = candidate._DisabledNativePairCandidate__pair
        originals = candidate.last_completed_futures()
        for scope, (_, records, stats, record_bytes, stats_bytes) in zip(('front', 'rear'), calls):
            self.assertIs(originals[scope], published[0][scope])
            self.assertIs(originals[scope].result(), result[scope])
            self.assertIs(result[scope][0], records)
            self.assertIs(type(result[scope][1]), native.Stats)
            self.assertIs(pair.last_completed_bus_results[scope][1], stats)
            self.assertIs(type(stats), active.Stats)
            self.assertEqual(bytes(records), record_bytes)
            self.assertEqual(bytes(stats), stats_bytes)
            self.assertEqual(bytes(result[scope][1]), stats_bytes[:C.sizeof(native.Stats)])
        self.assertTrue(pair.publication_complete(dict(originals)))

    def test_publication_fence_waits_even_when_both_original_futures_are_done(self):
        candidate = self.ready(); pair = candidate._DisabledNativePairCandidate__pair
        entered = threading.Event(); release = threading.Event()
        original_signal = pair._signal_published_completion
        def stalled_hint():
            entered.set()
            if not release.wait(.3): raise TimeoutError('Controlled publication stall')
            original_signal()
        with patch.object(pair, '_signal_published_completion', stalled_hint), \
             ThreadPoolExecutor(max_workers=1) as pool:
            call = pool.submit(candidate.exchange_stop_proxy, self.batches(),
                deadline_ns=time.monotonic_ns()+benchmark.PERIOD_NS)
            try:
                self.assertTrue(entered.wait(.2))
                self.assertTrue(all(future.done() for future in pair._current_futures.values()))
                pair.wait_idle()
                self.assertFalse(call.done())
                self.assertFalse(pair.publication_complete(pair._current_futures))
            finally: release.set()
            self.assertEqual(set(call.result(timeout=.35)), {'front', 'rear'})

    def test_late_postpublication_hint_failure_keeps_raw_and_cannot_claim_success(self):
        candidate = self.ready(); pair = candidate._DisabledNativePairCandidate__pair
        with patch.object(pair, '_signal_published_completion',
                          side_effect=OSError('Controlled final hint failure')), \
             self.assertRaisesRegex(OSError, 'final hint failure') as caught:
            candidate.exchange_stop_proxy(self.batches(),
                deadline_ns=time.monotonic_ns()+benchmark.PERIOD_NS)
        self.assertEqual(set(caught.exception.native_pair_bus_results), {'front', 'rear'})
        self.assertTrue(all(row.received == 17 for records, _ in
                            caught.exception.native_pair_bus_results.values() for row in records))
        evidence = candidate.evidence()['journal'][0]
        self.assertTrue(evidence['joined']); self.assertTrue(evidence['errors'])
        with self.assertRaisesRegex(RuntimeError, 'No validated current'):
            candidate.last_completed_futures()

    def test_transform_rejection_preserves_joined_original_raw_diagnostic_failure(self):
        candidate = self.ready(); normalizer = benchmark._disabled_diagnostic_result
        def fail_after_join(records, stats, **kwargs):
            result = normalizer(records, stats, **kwargs)
            if bytes(records[0].tx) == native.stop_wire(7):
                raise native.ExchangeError('Controlled diagnostic transform rejection', *result)
            return result
        with patch.object(benchmark, '_disabled_diagnostic_result', fail_after_join), \
             self.assertRaisesRegex(native.ExchangeError, 'transform rejection') as caught:
            candidate.exchange_stop_proxy(self.batches(),
                deadline_ns=time.monotonic_ns()+benchmark.PERIOD_NS)
        raw = caught.exception.native_pair_bus_results
        self.assertTrue(all(row.received == 17 for records, _ in raw.values() for row in records))
        self.assertEqual(set(raw), {'front', 'rear'})
        self.assertIs(type(raw['rear'][1]), active.Stats)
        self.assertIs(type(caught.exception.stats), native.Stats)
        self.assertEqual(bytes(caught.exception.stats), bytes(raw['rear'][1])[:C.sizeof(native.Stats)])
        self.assertTrue(candidate.evidence()['journal'][0]['errors'])


class DisabledPhaseSnapshotTests(unittest.TestCase):
    def phase(self):
        return dict(generation=1, submitted_ns=2, validated_ns=3, released_ns=4,
            cancel_requested_ns=0, owner_started_ns=[5, 6], owner_finished_ns=[7, 8],
            owner_status=[0, 0])

    def test_fixed_layout_is_exact_and_each_snapshot_is_independent(self):
        original = self.phase(); a = benchmark._disabled_pair_phase_snapshot(original)
        b = benchmark._disabled_pair_phase_snapshot(original)
        self.assertEqual(a, copy.deepcopy(original)); self.assertEqual(b, a)
        a['owner_status'][0] = -1; a['generation'] = 99
        self.assertEqual(original, self.phase()); self.assertEqual(b, self.phase())

    def test_unfamiliar_metadata_retains_general_deepcopy_and_all_fields(self):
        original = self.phase(); original['instrumentation'] = {'nested': [[17]]}
        result = benchmark._disabled_pair_phase_snapshot(original)
        self.assertEqual(result, copy.deepcopy(original))
        result['instrumentation']['nested'][0][0] = 99
        self.assertEqual(original['instrumentation']['nested'], [[17]])

    def test_original_ready_futures_at_deadline_still_fail_without_wait_or_backdating(self):
        futures = {'front': Future(), 'rear': Future()}
        for future in futures.values(): future.set_result('genuine result')
        deadline = 20_000_000
        with self.assertRaisesRegex(TimeoutError, '20 ms hard deadline'):
            benchmark._await_owned_ready(futures, None, phase='Proxy output',
                deadline_ns=deadline, clock=lambda: deadline,
                deadline_wait=lambda _: self.fail('Expired ready result must not wait'))


if __name__ == '__main__': unittest.main()
