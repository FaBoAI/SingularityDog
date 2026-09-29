"""Offline checks for the opt-in active release schedule; no motor devices."""

import ctypes
import threading
import unittest

from singularitydog_hw import native_active_transport as native
from singularitydog_hw import policy_output_runtime as active
import test_policy_output_runtime as fixtures


class AbsoluteEpochSlotTests(unittest.TestCase):
    def test_fixed_epoch_does_not_accumulate_wakeup_lateness(self):
        epoch=1_000_000_000
        self.assertEqual(active._absolute_epoch_slot(epoch,None,None,epoch),(0,epoch))
        self.assertEqual(active._absolute_epoch_slot(
            epoch,0,epoch+70_000,epoch+18_500_000),(1,epoch+20_000_000))
        self.assertEqual(active._absolute_epoch_slot(
            epoch,1,epoch+20_110_000,epoch+39_000_000),(2,epoch+40_000_000))

    def test_late_wakeup_identifies_skipped_slot_instead_of_replay(self):
        epoch=1_000_000_000
        self.assertEqual(active._absolute_epoch_slot(
            epoch,0,epoch+80_000,epoch+60_100_000),(3,epoch+60_000_000))
        # A very late first start cannot be followed by an almost immediate
        # second active command at the next nominal epoch.
        self.assertEqual(active._absolute_epoch_slot(
            epoch,0,epoch+18_000_000,epoch+19_000_000),(2,epoch+40_000_000))

    def test_invalid_state_is_rejected(self):
        epoch=1_000_000_000
        for arguments in ((epoch,1,None,epoch),
                          (epoch,None,epoch,epoch),
                          (epoch,0,epoch,epoch-1)):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                active._absolute_epoch_slot(*arguments)


class NativeActiveReleaseWaitTests(unittest.TestCase):
    def test_wait_returns_actual_monotonic_time(self):
        class Library:
            @staticmethod
            def sda_wait_until(fd,deadline,spin,actual,error,size):
                self.assertEqual((fd,deadline,spin),(4,1_000_000_000,500))
                ctypes.cast(actual,ctypes.POINTER(ctypes.c_uint64))[0]=deadline+12_345
                return 0
        self.assertEqual(native.wait_until(Library(),4,1_000_000_000),1_000_012_345)

    def test_missing_waiter_or_backdated_result_fails_closed(self):
        with self.assertRaises(native.ActiveWaitError):
            native.wait_until(object(),4,1_000_000_000)
        class Backdated:
            @staticmethod
            def sda_wait_until(fd,deadline,spin,actual,error,size):
                ctypes.cast(actual,ctypes.POINTER(ctypes.c_uint64))[0]=deadline-1
                return 0
        with self.assertRaisesRegex(native.ActiveWaitError,'backdated'):
            native.wait_until(Backdated(),4,1_000_000_000)


class ActiveRuntimeEpochIntegrationTests(unittest.TestCase):
    def test_opt_in_uses_contiguous_fixed_slots_and_restores_stop(self):
        clock=fixtures.SimulatedClock()
        stop=threading.Event();stop.set()
        case=fixtures.OutputRuntimeTests()
        report,sessions=case.run_case(profile_data=fixtures.profile(),
            front=fixtures.FakeSession(1,clock=clock),
            rear=fixtures.FakeSession(7,clock=clock),
            imu=fixtures.FakeIMU(clock=clock),clock=clock,sleep=clock.sleep,
            absolute_epoch_cadence=True,stop_requested=stop)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        schedule=report['absolute_epoch_schedule']
        self.assertEqual(schedule['completed_slots'],list(range(len(report['cycles']))))
        self.assertEqual(schedule['skipped_slots'],0)
        self.assertEqual(report['start_intervals_over_20ms'],0)
        self.assertTrue(report['stop_confirmed'])
        self.assertTrue(all(len(session.stop_times)==1 for session in sessions.values()))

    def test_injected_native_wait_is_used_only_for_future_epoch(self):
        clock=fixtures.SimulatedClock()
        stop=threading.Event();stop.set();waits=[]
        def wait_until(target):
            waits.append(target);clock.advance_to(target)
            return clock()
        case=fixtures.OutputRuntimeTests()
        report,_=case.run_case(profile_data=fixtures.profile(),
            front=fixtures.FakeSession(1,clock=clock),
            rear=fixtures.FakeSession(7,clock=clock),
            imu=fixtures.FakeIMU(clock=clock),clock=clock,sleep=clock.sleep,
            absolute_epoch_cadence=True,deadline_wait=wait_until,stop_requested=stop)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertTrue(waits)
        self.assertEqual(report['absolute_epoch_schedule']['skipped_slots'],0)

    def test_early_wait_return_stops_before_another_hold(self):
        clock=fixtures.SimulatedClock()
        stop=threading.Event();stop.set();waits=[]
        def early_wait(target):
            waits.append(target)
            return clock()
        case=fixtures.OutputRuntimeTests()
        report,sessions=case.run_case(profile_data=fixtures.profile(),
            front=fixtures.FakeSession(1,clock=clock),
            rear=fixtures.FakeSession(7,clock=clock),
            imu=fixtures.FakeIMU(clock=clock),clock=clock,sleep=clock.sleep,
            absolute_epoch_cadence=True,deadline_wait=early_wait,stop_requested=stop)
        self.assertTrue(waits)
        self.assertEqual(report['status'],'ABORTED')
        self.assertTrue(any('release before scheduled slot' in error for error in report['errors']))
        self.assertEqual(len(report['cycles']),1)
        self.assertTrue(report['stop_confirmed'])
        self.assertTrue(all(len(session.stop_times)==1 for session in sessions.values()))


if __name__=='__main__':unittest.main()
