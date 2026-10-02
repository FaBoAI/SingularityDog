"""File-only voltage checks against immutable, already validated feedback."""

import copy
import ctypes as C
import math
import struct
import threading
import time
import unittest
from unittest.mock import patch

from singularitydog_hw import native_diagnostic_transport as native
from singularitydog_hw import native_pipeline_benchmark as bench
from test_native_pipeline_benchmark import Device, Session
import test_native_voltage_fast_pipeline as fast_fixture
from test_native_voltage_overlap import OverlapObserver


class FrozenFeedbackVoltageTests(unittest.TestCase):
    def fixture(self):
        acquired={scope:Session().exchange([native.stop_wire(i) for i in ids])
                  for scope,ids in bench.dual.SCOPES.items()}
        sample=Device().read_sample()
        snapshot,proof=bench._validated_feedback_for_voltage(acquired,sample,time.monotonic_ns())
        expected={'front':1,'rear':7}
        voltage={scope:Session().exchange([bench.codec.read_request(mid,'voltage')])
                 for scope,mid in expected.items()}
        for rows,_ in voltage.values():rows[0].rx[11:15]=struct.pack('<f',40.)
        now=time.monotonic_ns()
        return acquired,voltage,sample,snapshot,expected,lambda:now,proof

    def verify(self,fixture):
        acquired,voltage,sample,snapshot,expected,clock,proof=fixture
        return bench._verify_voltage_after_inference(acquired,voltage,sample,snapshot,
                                                     expected,clock,42,proof)

    def final(self,fixture,full):
        acquired,voltage,sample,snapshot,expected,clock,proof=fixture
        return bench._verify_voltage_final_freshness(acquired,voltage,sample,snapshot,
                                                     full,expected,clock,42,proof)

    def test_fast_voltage_result_equals_complete_redecode_without_reparsing_feedback(self):
        f=self.fixture()
        before=copy.deepcopy(f[3])
        legacy,end=bench._verify_voltage_after_inference(*f[:6],42)
        with patch.object(bench,'snapshot_from_records',side_effect=AssertionError('redecode')):
            fast,fast_end=self.verify(f)
            self.final(f,fast)
        fast=dict(fast);fast.pop('_validated_voltage_proof')
        self.assertEqual(fast,legacy)
        self.assertEqual(fast_end,end)
        self.assertEqual(f[3],before)
        self.assertIsNot(fast['motors'][0],f[3]['motors'][0])
        self.assertIsNot(fast['imu']['accel_m_s2'],f[3]['imu']['accel_m_s2'])

    def test_native_feedback_mutations_rejected_before_new_voltage_is_used(self):
        for field in ('tx','rx','start_ns','finish_ns','received_ns','deadline_ns','written','received'):
            with self.subTest(field=field):
                f=self.fixture();row=f[0]['front'][0][0]
                if field in ('tx','rx'):getattr(row,field)[4]^=1
                else:setattr(row,field,getattr(row,field)+1)
                with self.assertRaisesRegex(ValueError,'Feedback/IMU changed'):self.verify(f)

    def test_imu_and_feedback_snapshot_mutations_rejected(self):
        mutations=(lambda f:f[2]['accel_m_s2'].__setitem__(0,1.),
                   lambda f:f[2]['gyro_rad_s'].__setitem__(1,.1),
                   lambda f:f[2].__setitem__('read_finished_monotonic_ns',f[2]['read_finished_monotonic_ns']+1),
                   lambda f:f[3]['motors'][0].__setitem__('value',.4),
                   lambda f:f[3]['motors'][0].__setitem__('unit','degree'),
                   lambda f:f[3]['motors'][0].__setitem__('age_upper_bound_ns',0),
                   lambda f:f[3]['imu']['accel_m_s2'].__setitem__(2,float('nan')),
                   lambda f:f[3]['source_flags'].__setitem__('sensor_type2_candidate',False),
                   lambda f:f[3].__setitem__('tick_ns',f[3]['tick_ns']+1))
        for index,mutate in enumerate(mutations):
            with self.subTest(mutation=index):
                f=self.fixture();mutate(f)
                with self.assertRaisesRegex(ValueError,'Feedback/IMU changed'):self.verify(f)

    def test_feedback_mutation_during_initial_decode_is_not_sealed_as_valid(self):
        f=self.fixture();original=bench.snapshot_from_records
        def changed(*args,**kwargs):
            value=original(*args,**kwargs)
            f[0]['rear'][0][0].rx[11]^=1
            return value
        with patch.object(bench,'snapshot_from_records',changed):
            with self.assertRaisesRegex(ValueError,'changed during initial'):
                bench._validated_feedback_for_voltage(f[0],f[2],f[5]())

    def test_each_voltage_requires_one_complete_causal_record(self):
        for failure in ('missing_bus','empty','duplicate','partial','future','noncausal','expired'):
            with self.subTest(failure=failure):
                f=self.fixture();voltage=f[1];row=voltage['front'][0][0]
                if failure=='missing_bus':voltage.pop('rear')
                elif failure=='empty':voltage['front']=((native.Record*0)(),native.Stats())
                elif failure=='duplicate':
                    pair=(native.Record*2)();C.memmove(C.addressof(pair),C.addressof(voltage['front'][0]),C.sizeof(native.Record))
                    C.memmove(C.addressof(pair[1]),C.addressof(pair[0]),C.sizeof(native.Record))
                    voltage['front']=(pair,native.Stats())
                elif failure=='partial':row.received=16
                elif failure=='future':row.received_ns=f[5]()+1
                elif failure=='noncausal':row.start_ns=row.received_ns+1
                elif failure=='expired':row.deadline_ns=row.received_ns
                with self.assertRaises(ValueError):self.verify(f)

    def test_voltage_rejects_nonfinite_range_reserved_status_and_header_errors(self):
        for failure in ('nan','inf','low','high','reserved','status','flags','trailer','wrong_parameter'):
            with self.subTest(failure=failure):
                f=self.fixture();row=f[1]['front'][0][0]
                if failure in ('nan','inf','low','high'):
                    row.rx[11:15]=struct.pack('<f',{'nan':math.nan,'inf':math.inf,'low':34.99,'high':42.01}[failure])
                elif failure=='reserved':row.rx[9]=1
                elif failure=='status':row.rx[3]|=8
                elif failure=='flags':row.rx[5]^=1
                elif failure=='trailer':row.rx[16]=0
                else:row.tx[:]=bench.codec.read_request(1,'position')
                with self.assertRaises(ValueError):self.verify(f)

    def test_wrong_bus_and_rotating_axis_are_rejected(self):
        for failure in ('cross_bus','wrong_axis','extra_expected_bus'):
            with self.subTest(failure=failure):
                f=self.fixture()
                if failure=='cross_bus':f[4]['front']=7
                elif failure=='wrong_axis':f[4]['front']=2
                else:f[4]['other']=1
                with self.assertRaises(ValueError):self.verify(f)

    def test_final_gate_detects_after_worker_native_or_decoded_voltage_mutation(self):
        for failure in ('raw_voltage','decoded_voltage','raw_feedback','snapshot','imu'):
            with self.subTest(failure=failure):
                f=self.fixture();full,_=self.verify(f)
                if failure=='raw_voltage':f[1]['rear'][0][0].rx[11:15]=struct.pack('<f',41.)
                elif failure=='decoded_voltage':full['voltage_by_bus']['front']['value_v']=41.
                elif failure=='raw_feedback':f[0]['rear'][0][0].rx[11]^=1
                elif failure=='snapshot':f[3]['motors'][0]['value']+=.01
                else:f[2]['gyro_rad_s'][0]=.1
                with self.assertRaises(ValueError):self.final(f,full)

    def test_final_gate_still_checks_full_input_age_after_worker_returns(self):
        f=self.fixture();full,_=self.verify(f)
        with self.assertRaisesRegex(ValueError,'Expired feedback/voltage/IMU'):
            bench._verify_voltage_final_freshness(*f[:4],full,f[4],lambda:f[5]()+bench.LIMIT_NS+1,42,f[6])

    def test_worker_completion_rejects_expired_original_inputs(self):
        f=self.fixture()
        with self.assertRaisesRegex(ValueError,'Expired/noncausal'):
            bench._verify_voltage_after_inference(*f[:5],lambda:f[5]()+bench.LIMIT_NS+1,42,f[6])

    def test_fast_pipeline_has_one_final_timestamp_gate_at_actual_stop_dispatch(self):
        started=(threading.Event(),threading.Event());release=threading.Event()
        sessions={scope:fast_fixture.PreparedSession(started[index],release)
                  for index,scope in enumerate(('front','rear'))}
        checked=[];original=bench._verify_voltage_final_freshness
        def gate(*args,**kwargs):
            result=original(*args,**kwargs);checked.append(result);return result
        with patch.object(bench,'_verify_voltage_final_freshness',gate):
            report,raw=bench.collect(sessions,fast_fixture.WaitForVoltageDevice(started),
                OverlapObserver(started,release),mode='stop-proxy',cycles=1,
                v3_voltage_proxy=True,v3_voltage_overlap=True,
                v3_voltage_validation_overlap=True,v3_voltage_fast_pipeline=True,
                record_storage='trace')
        self.assertEqual(report['status'],'COMPLETE_DIAGNOSTIC',report['errors'])
        self.assertEqual(len(checked),1)
        row=bench._serialize(raw)[0];proof=row['voltage_fast_pipeline']
        self.assertLessEqual(proof['voltage_join_ns'],proof['post_inference_verified_ns'])
        self.assertLessEqual(proof['post_inference_verified_ns'],checked[0])
        self.assertEqual(row['voltage_overlap']['final_freshness_checked_ns'],checked[0])
        self.assertLessEqual(checked[0],min(v['records'][0]['start_ns']
                                          for v in row['output'].values()))


if __name__=='__main__':unittest.main()
