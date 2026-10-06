"""Offline synthetic snapshot parity; no runtime import, device or library."""
import contextlib
import copy
import importlib.util
import io
import json
import os
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SOURCE=Path(__file__).resolve().parents[1]/'experiments/native_stop_snapshot_decode/candidate.py'
spec=importlib.util.spec_from_file_location('native_stop_snapshot_decode_candidate',SOURCE)
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)

class DecodeParity(unittest.TestCase):
    def setUp(self):self.e=m.load_engine();self.owned,self.sample,self.tick=m.synthetic_input(self.e)
    def both_reject(self):
        errors=[]
        for name in ('snapshot_from_records','direct_stop_snapshot'):
            with self.subTest(method=name),self.assertRaises(ValueError) as caught:getattr(self.e,name)(self.owned,self.sample,self.tick)
            errors.append(str(caught.exception))
        self.assertEqual(errors[0],errors[1])
    def test_all_snapshot_values_and_source_times_exact_for_many_payloads(self):
        rng=random.Random(20261006)
        for _ in range(100):
            for rows in self.owned.values():
                for row in rows:
                    data=struct.pack('>4H',*(rng.randrange(65536) for _ in range(4)))
                    row.rx[7:15]=data
            a=self.e.snapshot_from_records(self.owned,self.sample,self.tick)
            b=self.e.direct_stop_snapshot(self.owned,self.sample,self.tick)
            self.assertEqual(a,b)
            self.assertEqual(json.dumps(a,sort_keys=True,allow_nan=False),json.dumps(b,sort_keys=True,allow_nan=False))
        self.assertEqual([r['request_ns'] for r in b['motors']],
                         [r.start_ns for rows in self.owned.values() for r in rows for _ in range(2)])
    def test_existing_frame_allocations_eliminated_only_in_candidate_branch(self):
        original=self.e.codec.Frame;calls=[]
        def counted(*args):calls.append(args);return original(*args)
        self.e.codec.Frame=counted
        a=self.e.snapshot_from_records(self.owned,self.sample,self.tick)
        self.assertEqual(len(calls),24);calls.clear()
        b=self.e.direct_stop_snapshot(self.owned,self.sample,self.tick)
        self.assertEqual(calls,[]);self.assertEqual(a,b)
    def test_each_header_or_terminator_byte_corruption_rejected(self):
        row=self.owned['front'][0];wire=bytes(row.rx)
        for index in (*range(7),15,16):
            with self.subTest(index=index):
                row.rx[:]=wire;row.rx[index]^=1;self.both_reject()
        row.rx[:]=wire
    def test_fault_mode_wrong_sender_and_sentinel_rejected(self):
        row=self.owned['front'][0];wire=bytes(row.rx)
        ids=(((2<<24)|(1<<8)|0xfd|(1<<16)),((2<<24)|(1<<8)|0xfd|(2<<22)),((2<<24)|(2<<8)|0xfd))
        for can_id in ids:
            with self.subTest(can_id=can_id):
                row.rx[:]=wire;row.rx[2:6]=((can_id<<3)|4).to_bytes(4,'big');self.both_reject()
        row.rx[:]=wire;row.rx[7:10]=b'\x00\xc4\x56';self.both_reject()
    def test_noncanonical_stop_and_cross_bus_rejected(self):
        row=self.owned['front'][0];wire=bytes(row.tx)
        row.tx[7]=1;self.both_reject()
        row.tx[:]=self.e._STOP_WIRES[7];self.both_reject()
        row.tx[:]=wire
    def test_missing_or_duplicate_axis_never_zero_filled(self):
        self.owned['front'][1].tx[:]=self.e._STOP_WIRES[1]
        self.owned['front'][1].rx[:]=self.owned['front'][0].rx
        self.both_reject()
        self.owned.pop('rear');self.both_reject()
    def test_original_time_and_complete_receipt_limits_rejected(self):
        row=self.owned['front'][0]
        changes=(('written',16),('received',0),('start_ns',0),
                 ('finish_ns',row.received_ns+1),('received_ns',self.tick+1),
                 ('deadline_ns',row.received_ns))
        for key,value in changes:
            with self.subTest(key=key):
                old=getattr(row,key);setattr(row,key,value);self.both_reject();setattr(row,key,old)
        self.tick+=100_000_001;self.both_reject()
    def test_nonfinite_and_noncausal_imu_rejected(self):
        for name in ('accel_m_s2','gyro_rad_s'):
            old=self.sample[name]
            for value in (float('nan'),float('inf')):
                self.sample[name]=[value,0.,0.];self.both_reject()
            self.sample[name]=old
        self.sample['read_finished_monotonic_ns']=self.tick+1;self.both_reject()
    def test_type17_is_explicitly_outside_candidate_domain(self):
        row=self.owned['front'][0];row.tx[:]=self.e._READ_WIRES[1,'position']
        can_id=(17<<24)|(1<<8)|0xfd
        row.rx[:]=b'AT'+((can_id<<3)|4).to_bytes(4,'big')+b'\x08\x19\x70\x00\x00'+struct.pack('<f',1.)+b'\r\n'
        with self.assertRaisesRegex(ValueError,'canonical STOP'):self.e.direct_stop_snapshot(self.owned,self.sample,self.tick)
    def test_snapshot_mutation_does_not_change_original_buffers_or_other_snapshot(self):
        images={scope:bytes(rows) for scope,rows in self.owned.items()}
        sample=copy.deepcopy(self.sample)
        one=self.e.direct_stop_snapshot(self.owned,self.sample,self.tick)
        two=self.e.direct_stop_snapshot(self.owned,self.sample,self.tick)
        one['motors'][0]['value']=123.
        one['imu']['accel_m_s2'][0]=234.
        self.assertEqual(self.sample,sample)
        self.assertEqual({scope:bytes(rows) for scope,rows in self.owned.items()},images)
        self.assertEqual(two,self.e.snapshot_from_records(self.owned,self.sample,self.tick))
    def test_nonregular_symlink_and_oversized_source_rejected(self):
        name=next(iter(m.PINS))
        with tempfile.TemporaryDirectory() as root:
            clone=Path(root).resolve();target=clone/name;target.parent.mkdir(parents=True)
            target.symlink_to(m.DEFAULT_ROOT/name)
            with self.assertRaises(ValueError):m._source_bytes(clone,name)
            target.unlink()
            with target.open('wb') as stream:stream.truncate(1024*1024+1)
            with self.assertRaisesRegex(ValueError,'Bounded regular'):m._source_bytes(clone,name)
            target.unlink();target.mkdir()
            with self.assertRaises((ValueError,IsADirectoryError)):m._source_bytes(clone,name)
    def test_fifo_source_is_rejected_without_waiting_for_a_writer(self):
        name=next(iter(m.PINS))
        with tempfile.TemporaryDirectory() as root:
            clone=Path(root).resolve();target=clone/name;target.parent.mkdir(parents=True)
            os.mkfifo(target)
            code="""import importlib.util,sys
spec=importlib.util.spec_from_file_location('candidate',sys.argv[1])
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
try:m._source_bytes(sys.argv[2],sys.argv[3])
except ValueError as e:
 if 'Bounded regular' not in str(e):raise
else:raise AssertionError('FIFO accepted')
"""
            run=subprocess.run([sys.executable,'-B','-c',code,str(SOURCE),str(clone),name],capture_output=True,text=True,timeout=3)
            self.assertEqual(run.returncode,0,run.stderr)
    def test_profile_bounded_synthetic_only_and_keeps_raw_samples(self):
        with self.assertRaises(ValueError):m.profile_synthetic(self.e,True)
        for n in (0,10001):
            with self.assertRaises(ValueError):m.profile_synthetic(self.e,n)
        report=m.profile_synthetic(self.e,2)
        self.assertEqual([(r['block'],r['method']) for r in report['raw_samples']],[(1,'baseline'),(1,'candidate'),(2,'candidate'),(2,'baseline')])
        self.assertTrue(all(len(r['wall_ns'])==len(r['thread_cpu_ns'])==2 for r in report['raw_samples']))
        self.assertIs(report['active_output_eligible'],False);self.assertIs(report['live_performance_verified'],False)
    def test_plan_does_not_build_or_execute_engine(self):
        with patch.object(m,'load_engine',side_effect=AssertionError('PLAN engine')),patch.object(sys,'argv',[str(SOURCE)]),contextlib.redirect_stdout(io.StringIO()) as out:
            m.main()
        plan=json.loads(out.getvalue());self.assertEqual(plan['status'],'PLAN_FILE_ONLY');self.assertIs(plan['active_output_eligible'],False)
    def test_mutated_source_rejected_before_pure_execution(self):
        with tempfile.TemporaryDirectory() as root:
            clone=Path(root).resolve()
            for name in m.PINS:
                target=clone/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes((m.DEFAULT_ROOT/name).read_bytes())
            name=next(iter(m.PINS));target=clone/name;target.write_bytes(target.read_bytes()+b'\n')
            with patch.object(m,'DEFAULT_ROOT',clone),self.assertRaisesRegex(ValueError,'source changed'):
                m.load_engine()

if __name__=='__main__':unittest.main()
