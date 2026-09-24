"""Atomic snapshot publication and unchanged bounded queue/log contracts."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import dual_can_pipeline_benchmark as dual
from singularitydog_hw import policy_observer_live as live


def event():
    shared = {'value': [1., 2.], 'flags': {'ok': True}}
    return {'kind': 'pipeline_reply', 'monotonic_ns': 123,
            'result': shared, 'aliases': [shared], 'tuple': ({'raw': '4154'},)}


def invalid_events():
    cycle = []; cycle.append(cycle)
    deep = None
    for _ in range(26): deep = [deep]
    return ({'bad': float('nan')}, {'bad': object()}, {'bad': cycle},
            {'bad': deep}, {'bad': [0] * 20_001}, {'bad': 'x' * 100_000})


class EventSnapshotIntegrationTests(unittest.TestCase):
    def test_writer_keeps_owned_copy_and_flush_without_generic_deepcopy(self):
        bus = live.SessionBus(clock=lambda: 789)
        bus.publish(event(), telemetry_input=True)
        expected = copy.deepcopy(bus.logs.queue[0])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'events.jsonl'
            writer = live.AuditWriter(path, bus)
            with patch.object(copy, 'deepcopy', side_effect=AssertionError('generic deepcopy')):
                writer.start()
                writer.close()
            self.assertFalse(bus.errors)
            self.assertTrue(writer.flushed)
            self.assertEqual(writer.written, 1)
            self.assertEqual(json.loads(path.read_text()), json.loads(json.dumps(expected)))

    def test_invalid_committed_row_stops_writer_and_cannot_report_flush(self):
        bus = live.SessionBus(clock=lambda: 789)
        bus.logs.put({'bad': float('nan')})
        with tempfile.TemporaryDirectory() as directory:
            writer = live.AuditWriter(Path(directory)/'events.jsonl', bus)
            writer.start()
            writer.close()
            self.assertTrue(bus.stop.is_set())
            self.assertFalse(writer.flushed)
            self.assertEqual(writer.written, 0)
            self.assertEqual(bus.errors[0]['component'], 'log_writer')

    def test_event_buffer_detaches_all_descendants_and_matches_prior_json(self):
        source = event()
        previous = {'scope': 'front', 'wall_time_ns': 456, **copy.deepcopy(source)}
        buffer = dual.EventBuffer('front')
        with patch.object(dual.time, 'time_ns', return_value=456), \
             patch.object(copy, 'deepcopy', side_effect=AssertionError('hot deepcopy')), \
             patch.object(json, 'dumps', side_effect=AssertionError('hot JSON')):
            buffer.emit(source)
        self.assertEqual(json.dumps(buffer.rows[0]), json.dumps(previous))
        source['result']['value'].append(99)
        source['tuple'][0]['raw'] = 'changed'
        source['aliases'].clear()
        self.assertEqual(buffer.rows[0]['result']['value'], [1., 2.])
        self.assertEqual(buffer.rows[0]['tuple'][0]['raw'], '4154')
        self.assertEqual(buffer.rows[0]['aliases'][0]['value'], [1., 2.])

    def test_publish_owns_event_and_keeps_availability_and_queue_identity(self):
        source = event()
        previous = {'wall_time_ns': 456, **copy.deepcopy(source), 'available_monotonic_ns': 789}
        bus = live.SessionBus(clock=lambda: 789)
        with patch.object(live.time, 'time_ns', return_value=456), \
             patch.object(copy, 'deepcopy', side_effect=AssertionError('hot deepcopy')), \
             patch.object(json, 'dumps', side_effect=AssertionError('hot JSON')):
            bus.publish(source, telemetry_input=True)
        logged = bus.logs.get_nowait()
        self.assertEqual(json.dumps(logged), json.dumps(previous))
        self.assertEqual(list(bus.available(788)), [])
        inputs = list(bus.available(789))
        self.assertIs(inputs[0], logged)  # One committed owned row, as before.
        source['result']['value'].clear()
        source['tuple'][0]['raw'] = 'changed'
        self.assertEqual(logged['result']['value'], [1., 2.])
        self.assertEqual(logged['tuple'][0]['raw'], '4154')
        self.assertFalse(bus.stop.is_set())

    def test_invalid_snapshot_never_appends_partial_event_to_buffer(self):
        for source in invalid_events():
            with self.subTest(kind=type(source['bad'])):
                buffer = dual.EventBuffer('rear')
                buffer.emit({'kind': 'prior'})
                with self.assertRaises((ValueError, TypeError)):
                    buffer.emit(source)
                self.assertEqual(len(buffer.rows), 1)
                self.assertEqual(buffer.rows[0]['kind'], 'prior')

    def test_invalid_snapshot_stops_session_without_inserting_either_queue(self):
        for source in invalid_events():
            with self.subTest(kind=type(source['bad'])):
                bus = live.SessionBus(clock=lambda: 789)
                with self.assertRaises((ValueError, TypeError)):
                    bus.publish(source, telemetry_input=True)
                self.assertTrue(bus.inputs.empty() and bus.logs.empty())
                self.assertTrue(bus.stop.is_set())
                self.assertEqual(bus.errors[0]['component'], 'event_snapshot')
                self.assertEqual(bus.dropped_events, 0)

    def test_existing_event_bound_sealed_buffer_and_queue_overflow_still_fail(self):
        buffer = dual.EventBuffer('front')
        with patch.object(dual, 'MAX_EVENTS_PER_BUS', 1):
            buffer.emit({'kind': 'one'})
            with self.assertRaisesRegex(ValueError, 'budget'):
                buffer.emit({'kind': 'two'})
        buffer.sealed = True
        with self.assertRaisesRegex(ValueError, 'sealed'):
            buffer.emit({'kind': 'three'})
        self.assertEqual(len(buffer.rows), 1)
        bus = live.SessionBus(clock=lambda: 789, log_capacity=1)
        bus.publish({'kind': 'one'})
        with self.assertRaisesRegex(RuntimeError, 'queue overflow'):
            bus.publish({'kind': 'two'})
        self.assertEqual(bus.logs.qsize(), 1)
        self.assertTrue(bus.stop.is_set())
        self.assertEqual(bus.dropped_events, 1)


if __name__ == '__main__':
    unittest.main()
