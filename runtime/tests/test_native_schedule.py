"""Deterministic scheduler-delay regression; no wall-clock sleeps or hardware."""
import struct
import threading
import unittest

from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw import native_pipeline_benchmark as benchmark
from singularitydog_hw.can_readonly import ATParser


class SimulatedClock:
    def __init__(self, first_sleep_extra_ns=40_000_000):
        self.value = 1_000_000_000
        self.lock = threading.Lock()
        self.sleeps = []
        self.first_sleep_extra_ns = first_sleep_extra_ns

    def now(self):
        with self.lock:
            return self.value

    def reserve(self, duration):
        with self.lock:
            start = self.value
            self.value += duration
            return start

    def sleep(self, seconds):
        with self.lock:
            self.sleeps.append(seconds)
            self.value += round(seconds*1e9)
            if len(self.sleeps) == 1:
                self.value += self.first_sleep_extra_ns  # Scheduler oversleep, not slow input work.


class SimulatedSession:
    def __init__(self, clock):
        self.clock = clock

    def exchange(self, wires):
        rows = (native.Record*len(wires))()
        for row, wire in zip(rows, wires):
            tx = ATParser().feed(wire)[0]
            self_kind = 2 if tx.kind == 4 else 17
            can_id = self_kind << 24 | tx.destination << 8 | 0xfd
            data = (struct.pack('>4H',32768,32768,32768,250) if tx.kind == 4 else
                    tx.data[:4]+struct.pack('<f', .2))
            reply = b'AT'+((can_id << 3) | 4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'
            row.tx[:] = wire
            row.rx[:] = reply
            row.start_ns = self.clock.reserve(20_000)
            row.finish_ns = row.start_ns+1
            row.read_start_ns = row.start_ns+2
            row.received_ns = row.start_ns+3
            row.deadline_ns = row.start_ns+100_000_000
            row.written = row.received = 17
        return rows, native.Stats()


class SimulatedIMU:
    def __init__(self, clock):
        self.clock = clock

    def read_sample(self):
        start = self.clock.reserve(100_000)
        return {'frame':'sensor','accel_m_s2':[0.,0.,-9.81],'gyro_rad_s':[0.,0.,0.],
                'read_started_monotonic_ns':start, 'read_finished_monotonic_ns':start+1}


class BenchmarkObserver:
    def arm_run(self, tick):
        self.tick=tick

    def consume(self, snapshot):
        return {'q_target_rad_diagnostic_only':[0.]*12}

    def invalidate(self, error):
        self.error=error

    def finish(self):
        return {'diagnostic_only':True}


class NativeScheduleTests(unittest.TestCase):
    def test_absolute_epoch_reports_strict_20ms_starts_when_all_slots_meet(self):
        clock = SimulatedClock(first_sleep_extra_ns=0)
        report, _ = benchmark.collect(
            {scope: SimulatedSession(clock) for scope in ('front', 'rear')}, SimulatedIMU(clock),
            BenchmarkObserver(), mode='stop-proxy', cycles=3, clock=clock.now,
            sleep=clock.sleep, absolute_epoch_cadence=True)
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC')
        self.assertEqual([r['cadence_slot'] for r in report['measurements']], [0, 1, 2])
        self.assertEqual([r['actual_release_interval_ms'] for r in report['measurements']],
                         [None, 20., 20.])
        self.assertTrue(report['absolute_epoch_schedule']['strict_start_interval_20ms_met'])
        self.assertEqual(report['absolute_epoch_schedule']['slots_skipped'], 0)
        self.assertFalse(report['full_controller_50Hz_verified'])

    def test_absolute_epoch_slot_skips_a_nearly_immediate_followup(self):
        epoch = 1_000_000_000
        # The first cycle started at the end of slot zero. A fast completion
        # must not run the next cycle 0.2 ms later merely to catch up.
        slot, release = benchmark._absolute_epoch_slot(
            epoch, 0, epoch+19_800_000, epoch+19_900_000)
        self.assertEqual((slot, release), (2, epoch+40_000_000))
        self.assertGreaterEqual(release-(epoch+19_800_000),
                                benchmark.ABSOLUTE_MIN_START_SEPARATION_NS)

    def test_absolute_epoch_is_opt_in_and_skips_delayed_wakeup_slots(self):
        clock = SimulatedClock()
        report, _ = benchmark.collect(
            {scope: SimulatedSession(clock) for scope in ('front', 'rear')}, SimulatedIMU(clock),
            BenchmarkObserver(), mode='stop-proxy', cycles=3, clock=clock.now,
            sleep=clock.sleep, absolute_epoch_cadence=True)
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC')
        rows = report['measurements']
        self.assertEqual([row['cadence_slot'] for row in rows], [0, 3, 4])
        self.assertEqual([row['skipped_slots_before'] for row in rows], [0, 2, 0])
        self.assertEqual([row['scheduled_release_ns']-report['absolute_epoch_schedule']['epoch_ns']
                          for row in rows], [0, 60_000_000, 80_000_000])
        self.assertEqual(report['absolute_epoch_schedule']['slots_skipped'], 2)
        self.assertEqual(report['absolute_epoch_schedule']['start_intervals_over_20ms'], 1)
        self.assertFalse(report['absolute_epoch_schedule']['strict_start_interval_20ms_met'])
        self.assertTrue(all(row['actual_release_interval_ms'] is None or
                            row['actual_release_interval_ms']>=15.
                            for row in rows))
        self.assertFalse(report['motor_enable_sent'])
        self.assertFalse(report['learned_targets_sent'])

    def test_absolute_epoch_rejects_non_proxy_or_unbounded_run(self):
        clock = SimulatedClock()
        cases = [('type17', BenchmarkObserver(), 3),
                 ('stop-proxy', None, 3),
                 ('stop-proxy', BenchmarkObserver(), 501)]
        for mode, observer, cycles in cases:
            with self.subTest(mode=mode, cycles=cycles):
                with self.assertRaisesRegex(ValueError, 'Absolute-epoch cadence'):
                    benchmark.collect(
                        {scope: SimulatedSession(clock) for scope in ('front', 'rear')},
                        SimulatedIMU(clock), observer, mode=mode, cycles=cycles,
                        absolute_epoch_cadence=True)

    def test_delayed_wakeup_skips_catchup_and_reports_release_miss(self):
        clock = SimulatedClock()
        report, _ = benchmark.collect(
            {scope: SimulatedSession(clock) for scope in ('front', 'rear')}, SimulatedIMU(clock), None,
            mode='type17', cycles=3, clock=clock.now, sleep=clock.sleep)
        self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC')
        rows = report['measurements']
        releases = [row['release_ns'] for row in rows]
        self.assertEqual(releases[1]-releases[0], 60_000_000)
        # Old code used the previous planned release and immediately fired this
        # next acquisition, only 0.58ms after the delayed cycle began.
        self.assertGreaterEqual(releases[2]-releases[1], benchmark.PERIOD_NS)
        self.assertEqual(releases[2]-releases[1], 20_000_000)
        self.assertEqual(report['iteration_deadline_misses'], 0)
        self.assertEqual(report['release_lateness_over_1ms'], 1)
        self.assertEqual(report['release_intervals_over_21ms'], 1)
        self.assertAlmostEqual(rows[1]['release_lateness_ms'], 40.)
        self.assertAlmostEqual(rows[2]['release_lateness_ms'], 0.)
        self.assertIsNone(rows[0]['actual_release_interval_ms'])
        self.assertEqual(rows[1]['actual_release_interval_ms'], 60.)
        self.assertEqual(rows[2]['actual_release_interval_ms'], 20.)


if __name__ == '__main__':
    unittest.main()
