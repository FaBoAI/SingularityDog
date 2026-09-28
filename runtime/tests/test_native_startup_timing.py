"""Startup exception preserves every sample and cannot mask steady deadline failures."""
import unittest

from singularitydog_hw import native_pipeline_benchmark as benchmark
from test_native_schedule import SimulatedClock, SimulatedSession, SimulatedIMU, BenchmarkObserver


class TimedObserver(BenchmarkObserver):
    def __init__(self, clock, work):
        self.clock=clock;self.work=work;self.calls=0

    def consume(self, snapshot):
        self.clock.reserve(self.work(self.calls))
        self.calls+=1
        return super().consume(snapshot)


class StartupTimingTests(unittest.TestCase):
    def run_case(self, cycles=501, work=lambda n:20_000_000 if n==0 else 1_000_000,
                 oversleep=0, **options):
        clock=SimulatedClock(first_sleep_extra_ns=oversleep)
        observer=TimedObserver(clock,work)
        sessions={s:SimulatedSession(clock) for s in ('front','rear')}
        report, records=benchmark.collect(sessions,SimulatedIMU(clock),observer,
            mode='stop-proxy',cycles=cycles,clock=clock.now,sleep=clock.sleep,
            v3_voltage_proxy=True,record_storage='trace',absolute_epoch_cadence=True,
            startup_cycle_allowance=1,**options)
        return report,records,observer

    def test_one_startup_plus_500_steady_keeps_all_26_request_cycles(self):
        report,records,observer=self.run_case()
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC')
        self.assertEqual((report['cycles_completed'],len(records),observer.calls),(501,501,501))
        self.assertEqual(report['iteration_deadline_misses'],1)
        summary=report['steady_timing']
        self.assertEqual(summary['startup_iteration_deadline_misses'],1)
        self.assertEqual(summary['steady_cycles_completed'],500)
        self.assertEqual(summary['steady_iteration_deadline_misses'],0)
        self.assertTrue(summary['steady_processing_20ms_met'])
        self.assertTrue(summary['steady_scheduled_deadlines_met'])
        self.assertTrue(summary['strict_steady_start_interval_20ms_met'])
        self.assertEqual(summary['steady_start_interval_ms']['count'],499)
        self.assertGreater(summary['startup_to_steady_interval_ms'],20.)
        self.assertFalse(report['absolute_epoch_schedule']['strict_start_interval_20ms_met'])
        self.assertEqual([r['timing_phase'] for r in report['measurements']].count('startup'),1)
        self.assertEqual(sum(slot.count for slot in records[0].storage.slots),501*26)
        self.assertFalse(report['full_controller_50Hz_verified'])

    def test_later_processing_miss_is_not_forgiven(self):
        report,_,_=self.run_case(cycles=5,work=lambda n:20_000_000 if n in (0,3) else 1_000_000)
        self.assertEqual(report['iteration_deadline_misses'],2)
        self.assertEqual(report['steady_timing']['steady_iteration_deadline_misses'],1)
        self.assertFalse(report['steady_timing']['steady_processing_20ms_met'])
        self.assertFalse(report['steady_timing']['steady_scheduled_deadlines_met'])

    def test_sleep_lateness_misses_absolute_deadline_despite_fast_processing(self):
        report,_,_=self.run_case(cycles=4,work=lambda _:17_700_000,oversleep=2_000_000)
        summary=report['steady_timing']
        self.assertTrue(summary['steady_processing_20ms_met'])
        self.assertGreater(summary['steady_scheduled_deadline_misses'],0)
        self.assertFalse(summary['steady_scheduled_deadlines_met'])
        self.assertEqual(report['measurements'][1]['wait_calls'],1)
        self.assertGreater(report['measurements'][1]['wait_return_ns'],
                           report['measurements'][1]['scheduled_release_ns'])

    def test_native_wait_cancellation_retains_startup_but_does_not_pass_partial_run(self):
        def cancelled(_):raise RuntimeError('cancelled wait')
        report,records,_=self.run_case(cycles=4,work=lambda _:1_000_000,deadline_wait=cancelled)
        self.assertEqual(report['status'],'ABORTED')
        self.assertEqual((report['cycles_completed'],len(records)),(1,1))
        self.assertFalse(report['steady_timing']['steady_processing_20ms_met'])
        self.assertFalse(report['steady_timing']['steady_scheduled_deadlines_met'])
        self.assertFalse(report['steady_timing']['all_cycles_retained'])
        self.assertFalse(report['motor_enable_sent'])


if __name__=='__main__':unittest.main()
