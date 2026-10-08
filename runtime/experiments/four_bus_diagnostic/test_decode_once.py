"""Explicit decode-once hot path: same checks, one decode, raw-image takeouts."""
import copy
from dataclasses import replace
import socket
import struct
import threading
import time
from types import MappingProxyType
import unittest

from singularitydog_hw import can_readonly as codec
from singularitydog_hw.can_readonly import read_request
from singularitydog_hw.native_active_transport import Stats
from singularitydog_hw.native_diagnostic_transport import Record, stop_wire
from singularitydog_hw.policy_output_runtime import decode_records
from . import model_bridge as m
from . import pipeline
from . import test_subset_stop as native_fixture
from .test_model_bridge import Fixture, records, wire
from .test_pipeline import SeparateBoundaryIdentityTests
from .transport_adapter import Batch, Group, ThreeAxisTransport, PORTS

BOOT = native_fixture.BOOT


def batches_for(plan, raw, *, proxy):
    result = {}
    for port, values in raw.items():
        stats = Stats(); stats.begin_ns = min(r.start_ns for r in values)
        stats.end_ns = max(r.received_ns for r in values)
        rows = decode_records((values, stats))
        result[port] = Batch(Group(port, tuple(plan['topology_by_port'][port])), 'feedback', values, stats,
                             MappingProxyType(dict(rows)) if proxy else rows,
                             bytes(values), bytes(stats), stats.end_ns)
    return result


class SnapshotDecodeOnceTests(unittest.TestCase):
    def setUp(self):
        self.plan = Fixture().plan()
        self.raw, self.imu, self.tick = records(self.plan)

    def test_selected_builder_is_bit_identical_to_original_projection(self):
        expected = m.snapshot_from_four_records(self.plan, self.raw, self.imu, self.tick)
        legacy = m.batch_snapshot_builder(self.plan)(batches_for(self.plan, self.raw, proxy=False),
                                                    self.imu, self.tick)
        for proxy in (False, True):
            with self.subTest(proxy=proxy):
                actual = m.batch_snapshot_builder(self.plan, decode_once=True)(
                    batches_for(self.plan, self.raw, proxy=proxy), self.imu, self.tick)
                self.assertEqual(actual, expected); self.assertEqual(actual, legacy)
                for got, want in zip(actual['motors'], expected['motors']):
                    self.assertEqual(struct.pack('d', got['value']), struct.pack('d', want['value']))

    def test_frame_wire_matches_hex_frame_and_rejects_noncanonical_images(self):
        for values in self.raw.values():
            for record in values:
                for image in (bytes(record.tx), bytes(record.rx)):
                    self.assertEqual(m._frame_wire(image), m._frame(image.hex()))
        good = bytes(self.raw['port0'][0].rx)
        encoded = int.from_bytes(good[2:6], 'big')
        for bad in (good[:16], good + b'\x00', b'XT' + good[2:], good[:6] + b'\x07' + good[7:],
                    good[:15] + b'\r\r', good[:2] + ((encoded & ~7) | 0).to_bytes(4, 'big') + good[6:],
                    bytearray(good), good.hex()):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                m._frame_wire(bad)

    def test_selected_builder_keeps_every_frame_and_health_rejection(self):
        for mutation in ('tx', 'mode', 'fault', 'bad_id', 'partial', 'deadline', 'cross'):
            raw = {p: type(r).from_buffer_copy(bytes(r)) for p, r in self.raw.items()}
            r = raw['port0'][0]
            if mutation == 'tx': r.tx[7] = 1
            elif mutation == 'partial': r.received = 11
            elif mutation == 'deadline': r.deadline_ns = r.received_ns
            elif mutation == 'cross': raw['port0'], raw['port1'] = raw['port1'], raw['port0']
            else:
                frame = m._frame(bytes(r.rx).hex()); cid = frame.can_id
                if mutation == 'mode': cid |= 2 << 22
                elif mutation == 'fault': cid |= 1 << 16
                else: cid ^= 1 << 8
                r.rx[:] = wire(cid, frame.data)
            build = m.batch_snapshot_builder(self.plan, decode_once=True)
            with self.subTest(mutation=mutation), self.assertRaises((ValueError, RuntimeError)):
                batches = {}
                for port, values in raw.items():
                    stats = Stats()
                    batches[port] = Batch(Group(port, tuple(self.plan['topology_by_port'][port])), 'feedback',
                        values, stats, {}, bytes(values), bytes(stats), self.tick)
                build(batches, self.imu, self.tick)

    def test_raw_image_change_after_publication_is_still_rejected(self):
        build = m.batch_snapshot_builder(self.plan, decode_once=True)
        batches = batches_for(self.plan, self.raw, proxy=True)
        batches['port0'].records[0].rx[7] ^= 1
        with self.assertRaisesRegex(ValueError, 'changed'):
            build(batches, self.imu, self.tick)

    def test_selection_must_be_exact_bool(self):
        for value in (1, None, 'true'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'decode-once'):
                m.batch_snapshot_builder(self.plan, decode_once=value)


class NativeDecodeOnceTests(unittest.TestCase):
    setUpClass = classmethod(native_fixture.SubsetStopTests.setUpClass.__func__)
    tearDownClass = classmethod(native_fixture.SubsetStopTests.tearDownClass.__func__)
    setUp = native_fixture.SubsetStopTests.setUp
    tearDown = native_fixture.SubsetStopTests.tearDown
    peer_loop = native_fixture.SubsetStopTests.peer_loop

    def pair(self):
        host, peer = socket.socketpair(); host.setblocking(False)
        self.addCleanup(peer.close); self.addCleanup(host.close)
        return host, peer

    def adapter(self, ids=(1, 2, 3), *, once=True, host=None):
        group = Group('port0', ids)
        half = range(group.first_id, group.first_id+6)
        result = ThreeAxisTransport.create(self.lib, (host or self.host).fileno(), group=group,
            cancel_fd=self.cancel_read, boot_fd=self.boot.fileno(), boot_id=BOOT,
            raw_lower_by_id={mid: -1. for mid in half}, raw_upper_by_id={mid: 1. for mid in half},
            decode_once=once)
        self.adapters.append(result); return result

    def stop(self, adapter, peer=None):
        self.peer_loop(3, fd=None if peer is None else peer.fileno())
        return adapter.output_stop(deadline_ns=time.monotonic_ns()+20_000_000, check=lambda: None)

    def test_rows_are_read_only_original_decode_and_reused_without_redecode(self):
        adapter = self.adapter(); batch = self.stop(adapter)
        self.assertIs(type(batch.rows), MappingProxyType)
        self.assertEqual(batch.rows, decode_records((batch.records, batch.stats)))
        with self.assertRaises(TypeError):
            batch.rows[(1, 'feedback')] = batch.rows[(2, 'feedback')]
        from unittest.mock import patch
        with patch('experiments.four_bus_diagnostic.transport_adapter.decode_records',
                   side_effect=AssertionError('re-decoded')):
            self.assertIs(adapter.verify_batch(batch, 'output_stop'), batch.rows)

    def test_raw_and_stats_tamper_are_still_rejected(self):
        adapter = self.adapter()
        for target in ('rx', 'stats'):
            batch = self.stop(adapter)
            if target == 'rx': batch.records[1].rx[9] ^= 1
            else: batch.stats.writes += 1
            with self.subTest(target=target), self.assertRaisesRegex(ValueError, 'raw batch'):
                adapter.verify_batch(batch, 'output_stop')

    def test_forged_foreign_or_relabelled_batches_are_rejected(self):
        first = self.adapter(); batch = self.stop(first)
        forged = Batch(batch.group, batch.label, batch.records, batch.stats, batch.rows,
                       batch.record_image, batch.stats_image, batch.completed_ns)
        with self.assertRaisesRegex(ValueError, 'Genuine'):
            first.verify_batch(forged, 'output_stop')
        with self.assertRaisesRegex(ValueError, 'Genuine'):
            first.verify_batch(batch, 'feedback')
        host, _ = self.pair(); second = self.adapter((4, 5, 6), host=host)
        with self.assertRaisesRegex(ValueError, 'Genuine'):
            second.verify_batch(batch, 'output_stop')

    def test_bypassed_row_substitution_falls_back_to_full_redecode(self):
        adapter = self.adapter(); batch = self.stop(adapter)
        rows = dict(batch.rows); value, start, received = rows[(1, 'feedback')]
        rows[(1, 'feedback')] = (replace(value, torque_nm=value.torque_nm + 1.), start, received)
        object.__setattr__(batch, 'rows', rows)
        with self.assertRaisesRegex(ValueError, 'decoded rows changed'):
            adapter.verify_batch(batch, 'output_stop')
        object.__setattr__(batch, 'rows', MappingProxyType(rows))
        with self.assertRaisesRegex(ValueError, 'decoded rows changed'):
            adapter.verify_batch(batch, 'output_stop')

    def test_mutable_voltage_rows_keep_full_redecode(self):
        adapter = self.adapter(); self.peer_loop(1)
        batch = adapter.read_voltage(2, deadline_ns=time.monotonic_ns()+20_000_000, check=lambda: None)
        self.assertEqual(adapter.verify_batch(batch, 'setup_voltage'), decode_records((batch.records, batch.stats)))
        batch.rows[(2, 'voltage')][0]['value'] = 0.
        with self.assertRaisesRegex(ValueError, 'decoded rows changed'):
            adapter.verify_batch(batch, 'setup_voltage')

    def test_selection_is_sealed_exact_and_default_is_unchanged(self):
        adapter = self.adapter()
        adapter.decode_once = False
        with self.assertRaisesRegex(ValueError, 'binding changed'):
            adapter._verify()
        adapter.decode_once = True
        host, peer = self.pair()
        default = self.adapter((4, 5, 6), once=False, host=host); batch = self.stop(default, peer)
        self.assertIs(type(batch.rows), dict)
        self.assertEqual(default.verify_batch(batch, 'output_stop'), batch.rows)
        host, _ = self.pair()
        with self.assertRaisesRegex(ValueError, 'exact bool'):
            self.adapter((7, 8, 9), once=1, host=host)

    def test_identity_index_retains_every_owner_batch(self):
        adapter = self.adapter(); made = [self.stop(adapter) for _ in range(5)]
        for batch in made:
            self.assertIs(adapter.verify_batch(batch, 'output_stop'), batch.rows)
        self.assertEqual(len(adapter._batches), 5); self.assertEqual(len(adapter._batch_index), 5)


class SeparateDecodeOnceTests(SeparateBoundaryIdentityTests):
    """Every separate-branch case also passes with decode-once selected."""
    def setUp(self):
        super().setUp()
        self.config = replace(self.config, decode_once=True)

    def factory(self, group):
        # Follow the current Config so inherited cases that rebuild it still match.
        value = super().factory(group); value.decode_once = self.config.decode_once; return value

    def test_plan_records_decode_once(self):
        value = pipeline.run(self.config)
        self.assertTrue(value['decode_once_selected'])
        self.assertEqual(value['plain_data_copy'], 'pickle_protocol5_roundtrip')
        self.assertNotIn('decode_once_selected', pipeline.run(replace(self.config, decode_once=False)))
        with self.assertRaisesRegex(ValueError, 'exact bool'):
            pipeline.plan(replace(self.config, decode_once=1))

    def test_transport_selection_mismatch_rejects_before_owner_calls(self):
        from .test_pipeline import PipelineTests
        for config_once, adapter_once in ((True, False), (False, True)):
            self.config = replace(self.config, decode_once=config_once); self.canceled.clear()
            def factory(group, once=adapter_once):
                value = PipelineTests.factory(self, group); value.decode_once = once; return value
            self.factory = factory
            result = self.run_case()
            with self.subTest(config=config_once, adapter=adapter_once):
                self.assertEqual(result['status'], 'ABORTED')
                self.assertIn('decode-once', result['primary_error']['message'])
                self.assertFalse(result['setup_voltage'])


if __name__ == '__main__':
    unittest.main()
