"""Synthetic causal replay tests; no CAN, I2C, hardware or files are used."""
from dataclasses import FrozenInstanceError
import math
import unittest

from singularitydog_hw.telemetry_snapshot import MOTOR_KEYS, TelemetrySnapshotBuffer


def motor(buffer, mid=1, parameter="position", start=10, end=20, value=.1, **overrides):
    kwargs = dict(can_type=17, motor_id=mid, parameter=parameter, value=value,
                  unit="rad" if parameter == "position" else "rad_s",
                  request_ns=start, received_ns=end)
    kwargs.update(overrides)
    return buffer.ingest_motor(**kwargs)


def imu(buffer, start=10, end=20, **overrides):
    kwargs = dict(accel_m_s2=[0., 0., 9.81], gyro_rad_s=[0., .1, 0.],
                  read_started_ns=start, read_finished_ns=end)
    kwargs.update(overrides)
    return buffer.ingest_imu(**kwargs)


def all_sources(buffer, start=10, end=20, omit=None):
    for mid, parameter in MOTOR_KEYS:
        if (mid, parameter) != omit:
            motor(buffer, mid, parameter, start, end)
    imu(buffer, start, end)


def pipeline_row():
    return {"kind": "pipeline_reply", "ok": True, "motor_id": 1, "parameter": "position",
            "write_started_monotonic_ns": 10, "write_finished_monotonic_ns": 11,
            "received_monotonic_ns": 20,
            "result": {"ok": True, "motor_id": 1, "parameter": "position", "status": 0,
                       "index": 0x7019, "value": .25, "unit": "rad_output_shaft"}}


class SnapshotTests(unittest.TestCase):
    def test_complete_data_is_diagnostic_only_and_reports_interval_metrics(self):
        buffer = TelemetrySnapshotBuffer(max_age_ns=50, max_spread_ns=30)
        all_sources(buffer)
        snap = buffer.snapshot(30)
        self.assertEqual(snap.status, "DIAGNOSTIC_READY")
        self.assertFalse(snap.output_allowed)
        self.assertEqual(snap.blocked_reasons, ())
        self.assertEqual(len(snap.motor_samples), 24)
        self.assertEqual(snap.oldest_observation_age_ns, 20)
        self.assertEqual(snap.acquisition_spread_ns, 10)
        self.assertEqual(snap.receive_spread_ns, 0)
        row = snap.as_dict()
        self.assertFalse(row['output_allowed'])
        self.assertEqual(row['imu']['frame'], 'raw_sensor')
        self.assertTrue(all(s['age_upper_bound_ns'] == 20 for s in row['motors']))

    def test_known_loss_is_missing_never_zero_or_ready(self):
        buffer = TelemetrySnapshotBuffer()
        all_sources(buffer, omit=(9, 'position'))
        snap = buffer.snapshot(20)
        self.assertEqual(snap.status, 'BLOCKED')
        self.assertIn('missing:ID9.position', snap.blocked_reasons)
        missing = [r for r in snap.as_dict()['motors'] if r['value'] is None]
        self.assertEqual(len(missing), 1)
        self.assertEqual(missing[0]['motor_id'], 9)
        self.assertIsNone(missing[0]['request_ns'])
        self.assertIsNone(missing[0]['received_ns'])

    def test_empty_buffer_metrics_and_values_are_absent(self):
        snap = TelemetrySnapshotBuffer().snapshot(0)
        self.assertEqual(len(snap.blocked_reasons), 25)
        self.assertIsNone(snap.imu_sample)
        self.assertIsNone(snap.oldest_observation_age_ns)
        self.assertIsNone(snap.acquisition_spread_ns)
        self.assertIsNone(snap.receive_spread_ns)
        self.assertTrue(all(r['value'] is None for r in snap.as_dict()['motors']))

    def test_lost_update_keeps_original_timestamp_and_eventually_blocks_as_stale(self):
        buffer = TelemetrySnapshotBuffer(max_age_ns=50, max_spread_ns=1000)
        all_sources(buffer, start=10, end=20)
        for mid, parameter in MOTOR_KEYS:
            if (mid, parameter) != (2, 'position'):
                motor(buffer, mid, parameter, start=100, end=110, value=.5)
        imu(buffer, start=100, end=110)
        snap = buffer.snapshot(110)
        old = next(s for s in snap.motor_samples if (s.motor_id, s.parameter) == (2, 'position'))
        self.assertEqual((old.value, old.request_ns, old.received_ns), (.1, 10, 20))
        self.assertEqual(snap.oldest_observation_age_ns, 100)
        self.assertEqual(snap.blocked_reasons, ('stale:ID2.position',))
        self.assertFalse(snap.output_allowed)

    def test_future_ingestion_cannot_replace_tick_causal_history(self):
        buffer = TelemetrySnapshotBuffer(max_age_ns=1000, max_spread_ns=1000)
        all_sources(buffer, start=10, end=20)
        motor(buffer, start=100, end=110, value=9.)
        imu(buffer, start=100, end=110, accel_m_s2=[1., 2., 3.])
        old = buffer.snapshot(30)
        self.assertEqual(old.motor_samples[0].value, .1)
        self.assertEqual(old.imu_sample.accel_m_s2, (0., 0., 9.81))
        self.assertEqual(old.oldest_observation_age_ns, 20)
        self.assertEqual(buffer.snapshot(110).motor_samples[0].value, 9.)
        self.assertEqual(buffer.snapshot(30), old)  # Replaying backwards is allowed.

    def test_tick_waits_for_read_completion_even_if_request_started(self):
        buffer = TelemetrySnapshotBuffer()
        all_sources(buffer, start=10, end=20)
        before = buffer.snapshot(19)
        self.assertEqual(before.motor_samples, ())
        self.assertIsNone(before.imu_sample)
        self.assertTrue(all(r.startswith('not_yet_available:') for r in before.blocked_reasons))
        self.assertEqual(buffer.snapshot(20).status, 'DIAGNOSTIC_READY')

    def test_age_and_spread_boundaries_use_acquisition_start_conservatively(self):
        buffer = TelemetrySnapshotBuffer(max_age_ns=15, max_spread_ns=10)
        all_sources(buffer, start=10, end=20)
        self.assertEqual(buffer.snapshot(25).status, 'DIAGNOSTIC_READY')
        aged = buffer.snapshot(26)
        self.assertEqual(sum(r.startswith('stale:') for r in aged.blocked_reasons), 25)
        motor(buffer, mid=12, parameter='velocity', start=21, end=22)
        spread = buffer.snapshot(25)
        self.assertEqual(spread.acquisition_spread_ns, 12)
        self.assertEqual(spread.receive_spread_ns, 2)
        self.assertIn('acquisition_spread_exceeded', spread.blocked_reasons)

    def test_duplicate_and_reordered_per_key_samples_are_rejected_without_mutation(self):
        buffer = TelemetrySnapshotBuffer()
        first = motor(buffer)
        for start, end in ((10, 20), (9, 21), (11, 19), (11, 20), (10, 21)):
            with self.subTest(start=start, end=end), self.assertRaises(ValueError):
                motor(buffer, start=start, end=end)
        self.assertEqual(buffer.history_sizes()[(1, 'position')], 1)
        self.assertEqual(buffer.snapshot(30).motor_samples, (first,))

    def test_same_receive_timestamp_across_keys_and_global_reordering_are_valid(self):
        buffer = TelemetrySnapshotBuffer()
        motor(buffer, mid=2, start=12, end=20)
        motor(buffer, mid=1, start=10, end=20)
        motor(buffer, mid=1, parameter='velocity', start=5, end=19)
        snap = buffer.snapshot(20)
        self.assertEqual(len(snap.motor_samples), 3)
        self.assertEqual(snap.receive_spread_ns, 1)

    def test_bounded_history_distinguishes_evicted_data_from_future_data(self):
        buffer = TelemetrySnapshotBuffer(history_per_key=2)
        for start in (10, 20, 30, 40):
            motor(buffer, start=start, end=start+1, value=start)
            imu(buffer, start=start, end=start+1)
        self.assertEqual(buffer.history_sizes()[(1, 'position')], 2)
        self.assertEqual(buffer.history_sizes()['imu'], 2)
        gap = buffer.snapshot(25)
        self.assertIn('history_gap:ID1.position', gap.blocked_reasons)
        self.assertIn('history_gap:imu', gap.blocked_reasons)
        self.assertEqual(gap.motor_samples, ())
        self.assertIsNone(gap.imu_sample)
        self.assertIn('not_yet_available:ID1.position', buffer.snapshot(0).blocked_reasons)
        self.assertEqual(buffer.snapshot(31).motor_samples[0].value, 30.)
        self.assertEqual(buffer.snapshot(41).motor_samples[0].value, 40.)

    def test_observations_and_snapshot_are_immutable_and_input_vectors_copied(self):
        buffer = TelemetrySnapshotBuffer()
        sample = motor(buffer)
        vector = [0., 0., 9.81]
        sensor = imu(buffer, accel_m_s2=vector)
        vector[2] = 0.
        self.assertEqual(sensor.accel_m_s2[2], 9.81)
        for obj, field, value in ((sample, 'value', 5.), (sensor, 'read_finished_ns', 99),
                                  (buffer.snapshot(20), 'tick_ns', 99)):
            with self.assertRaises(FrozenInstanceError): setattr(obj, field, value)
        export = buffer.snapshot(20).as_dict()
        export['motors'][0]['value'] = 99
        self.assertEqual(buffer.snapshot(20).motor_samples[0].value, .1)


class InputTests(unittest.TestCase):
    def test_motor_rejects_wrong_type_id_field_units_nonfinite_and_time(self):
        invalid = ({'can_type': 1}, {'can_type': True}, {'motor_id': 0}, {'motor_id': 13},
                   {'motor_id': True}, {'motor_id': '1'}, {'parameter': 'voltage'},
                   {'parameter': []}, {'unit': 'deg'}, {'unit': 'rad_s'},
                   {'value': math.nan}, {'value': math.inf}, {'value': True},
                   {'value': '0.1'}, {'value': 10**1000}, {'request_ns': -1},
                   {'request_ns': True}, {'received_ns': 1.5}, {'received_ns': 9})
        for override in invalid:
            with self.subTest(override=override):
                buffer = TelemetrySnapshotBuffer()
                with self.assertRaises(ValueError): motor(buffer, **override)
                self.assertEqual(sum(buffer.history_sizes().values()), 0)

    def test_imu_requires_finite_triplets_and_strict_per_source_timestamps(self):
        for override in ({'accel_m_s2': [1, 2]}, {'gyro_rad_s': [0, 0, math.inf]},
                         {'gyro_rad_s': [0, True, 0]}, {'read_started_ns': True},
                         {'read_finished_ns': 9}, {'read_finished_ns': math.nan}):
            with self.subTest(override=override), self.assertRaises(ValueError):
                imu(TelemetrySnapshotBuffer(), **override)
        buffer = TelemetrySnapshotBuffer()
        imu(buffer)
        for start, end in ((10, 20), (9, 21), (11, 20)):
            with self.assertRaises(ValueError): imu(buffer, start=start, end=end)
        self.assertEqual(buffer.history_sizes()['imu'], 1)

    def test_configuration_and_tick_validation(self):
        for kwargs in ({'history_per_key': 0}, {'history_per_key': 1025},
                       {'history_per_key': True}, {'max_age_ns': -1},
                       {'max_spread_ns': math.inf}, {'max_age_ns': True}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                TelemetrySnapshotBuffer(**kwargs)
        for tick in (-1, True, 1.5, math.nan):
            with self.assertRaises(ValueError): TelemetrySnapshotBuffer().snapshot(tick)

    def test_successful_pipeline_event_maps_source_units_and_host_times(self):
        buffer = TelemetrySnapshotBuffer()
        sample = buffer.ingest_pipeline_reply(pipeline_row())
        self.assertEqual((sample.value, sample.unit), (.25, 'rad'))
        self.assertEqual((sample.request_ns, sample.received_ns), (10, 20))
        self.assertEqual(buffer.snapshot(19).motor_samples, ())
        self.assertEqual(buffer.snapshot(20).motor_samples, (sample,))

    def test_pipeline_rejects_failed_identity_mismatched_and_invalid_intervals(self):
        changes = (('ok', False), ('kind', 'pipeline_tx_intent'), ('parameter', 'identity'),
                   ('motor_id', 2), ('write_finished_monotonic_ns', 21),
                   ('write_finished_monotonic_ns', 9), ('received_monotonic_ns', None))
        for key, value in changes:
            row = pipeline_row(); row[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                TelemetrySnapshotBuffer().ingest_pipeline_reply(row)
        for key, value in (('ok', False), ('motor_id', True), ('status', 1), ('status', False),
                           ('index', 0x701B), ('value', math.nan), ('unit', 'deg')):
            row = pipeline_row(); row['result'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                TelemetrySnapshotBuffer().ingest_pipeline_reply(row)


if __name__ == '__main__':
    unittest.main()
