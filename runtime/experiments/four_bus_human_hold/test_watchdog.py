"""Actual Channel byte-engine and independent raw validator over memory ports.

No serial device/native library/physical conditions/permission are created. The
frames below are explicitly synthetic; successful tests are not live evidence.
"""
import copy
import hashlib
import io
import json
import math
import os
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import patch,Mock

from singularitydog_hw import can_readonly as codec
from singularitydog_hw.motor_version_probe import VERSION_PREFIX
from . import profile as HP, watchdog as W, watchdog_foreground as F
from . import test_profile as profile_cases
from .test_profile import wire


class Clock:
    def __init__(self):self.now=1_000_000_000
    def __call__(self):return self.now
    def wait(self,s):self.now+=max(1,round(s*1e9))


class MemoryPort:
    """Four independently owned fake endpoints issue one finite canonical reply."""
    def __init__(self,ids,base,clock,bad=None):
        self.ids,self.base,self.clock,self.bad=tuple(ids),base,clock,bad
        self.queue=bytearray();self.enabled=set();self.last_zero={};self.calls=[];self.zero_count={}
        self.raw={mid:base['model_plan']['provenance']['raw_rad_by_id'][str(mid)] for mid in ids}
    @property
    def in_waiting(self):return len(self.queue)
    def fb(self,mid,mode):
        q=self.raw[mid]
        v=.8 if self.bad=='speed' and mid==2 else 0.
        body=struct.pack('>4H',int((q+12.57)*65535/25.14),int((v+50)*65535/100),32767,250)
        return wire((2<<24)|(mode<<22)|(mid<<8)|codec.HOST_ID,body)
    def write(self,raw):
        value=codec.ATParser().feed(raw)[0];mid=value.destination
        if mid not in self.ids:raise AssertionError('Cross-owner fake request')
        self.calls.append((self.clock(),mid,value.kind,raw.hex()))
        if self.bad=='partial' and mid==2 and value.kind==3:return 16
        if value.kind==0:
            uid=self.base['uids_by_id'][str(mid)]
            if self.bad=='uid' and mid==2:uid='00'*8
            reply=wire((mid<<8)|0xfe,bytes.fromhex(uid))
        elif value.kind==17:
            index=int.from_bytes(value.data[:2],'little')
            if index==0x7028:payload=struct.pack('<I',3999 if self.bad=='timeout' and mid==2 else 4000)
            elif index==0x7005:payload=bytes(4)
            elif index==0x701c:payload=struct.pack('<f',34.9 if self.bad=='voltage' and mid==2 else 40.)
            else:raise AssertionError('Unapproved watchdog parameter')
            reply=wire((17<<24)|(mid<<8)|codec.HOST_ID,value.data[:4]+payload)
        elif value.kind==4 and value.data[:2]==b'\x00\xc4':
            reply=wire((2<<24)|(mid<<8)|codec.HOST_ID,VERSION_PREFIX+bytes.fromhex('05001300')+b'\0')
        elif value.kind==18:
            if int.from_bytes(value.data[:2],'little') != 0x7028:raise AssertionError('Nonwatchdog write')
            reply=self.fb(mid,0)
        elif value.kind==3:
            self.enabled.add(mid);self.last_zero[mid]=self.clock();reply=self.fb(mid,2)
        elif value.kind==1:
            self.zero_count[mid]=self.zero_count.get(mid,0)+1
            fields=struct.unpack('>4H',value.data)
            if fields[2] or fields[3]:raise AssertionError('Positive gains')
            if (mid in self.enabled and self.clock()-self.last_zero.get(mid,self.clock()) >= 200_000_000
                    and self.bad!='no_disable'):
                self.enabled.discard(mid)
            mode=2 if mid in self.enabled else 0
            if self.bad=='late_probe' and self.zero_count[mid]==3:self.clock.now+=41_000_000
            self.last_zero[mid]=self.clock();reply=self.fb(mid,mode)
            if self.bad=='duplicate' and mid==2 and self.zero_count[mid]==2:reply+=reply
        elif value.kind==4:
            self.enabled.discard(mid);reply=self.fb(mid,0)
        else:raise AssertionError('Unapproved raw request kind')
        self.queue.extend(reply);self.clock.now+=1000
        return len(raw)


class MemoryReader:
    def __init__(self,port,*,clock,check):self.port,self.clock,self.check=port,clock,check
    def stats(self):return {'synthetic_read_calls':0}
    def read_until(self,wake,hard):
        self.check()
        if not self.port.queue:
            self.clock.now=hard;return b'',self.clock()
        self.clock.now+=100_000
        raw=bytes(self.port.queue);self.port.queue.clear();return raw,self.clock()


class WatchdogRawFlowTests(unittest.TestCase):
    def setUp(self):
        self.fixture=profile_cases.ContractPlanTests();self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.contract=self.fixture.human;self.base=self.contract['base_numerical_contract']
        self.firmware={str(mid):'05001300' for mid in HP.IDS}

    def run_case(self,bad=None,check=lambda:None):
        clock=Clock();channels={};ports={}
        with patch('singularitydog_hw.serial_deadline_reader.DeadlineSerialReader',MemoryReader), \
             patch('singularitydog_hw.watchdog_commissioning.time.sleep',clock.wait):
            for port,ids in self.base['topology_by_port'].items():
                endpoint=MemoryPort(ids,self.base,clock,bad=bad if 2 in ids else None);ports[port]=endpoint
                channels[port]=W.Channel(endpoint,ids,clock=clock,check=check)
            report=W.run(channels,self.contract,self.firmware,clock=clock,wait=clock.wait,
                         check=check,announce=lambda:None,execute=True)
        return report,W.trace(report,channels),channels,ports

    def validate(self,report,trace):
        return HP.validate_watchdog(report,trace,self.contract,after_ns=1,
            source_manifest_sha256=self.base['source_manifest']['sha256'])

    def test_original_channel_real_byte_flow_satisfies_unmodified_primary_validator(self):
        report,trace,channels,ports=self.run_case()
        self.assertEqual(report['status'],'COMPLETE_COMMAND_LOSS_DIAGNOSTIC',report['errors'])
        result=self.validate(report,trace)
        self.assertEqual(result['firmware_by_id'],self.firmware)
        self.assertTrue(report['stop_confirmed'])
        self.assertFalse(report['positive_gain_sent'] or report['learned_targets_sent'])
        self.assertFalse(report['approved_for_runtime'] or report['usb_disconnect_tested'])
        self.assertEqual(sum(len(ch.ids) for ch in channels.values()),12)
        for mid in HP.IDS:
            axis=report['axes'][str(mid)]
            self.assertTrue(axis['disabled_on_command_loss'])
            self.assertGreaterEqual(axis['disable_reply_upper_bound_ms'],210)
            self.assertLessEqual(axis['disable_reply_upper_bound_ms'],250)
        allowed={0,1,3,4,17,18}
        self.assertTrue(all(kind in allowed for p in ports.values() for _,_,kind,_ in p.calls))

    def test_every_failed_input_retains_raw_and_never_qualifies(self):
        for bad in ('uid','voltage','timeout','speed','no_disable','partial','duplicate','late_probe'):
            with self.subTest(bad=bad):
                report,trace,channels,_=self.run_case(bad)
                self.assertFalse(report['status'].startswith('COMPLETE'))
                self.assertTrue(report['errors'])
                self.assertEqual(set(report['stop_reports']),set(HP.PORTS))
                self.assertFalse(trace['failed_receive_or_partial_events_filtered'])
                with self.assertRaises(ValueError):self.validate(report,trace)
                if bad in ('partial','duplicate'):
                    owner=next(ch for ch in channels.values() if 2 in ch.ids)
                    self.assertTrue(owner._ambiguous_ids or owner._stop_boundary_uncertain)

    def test_global_silence_raw_extra_tx_and_wrong_header_do_not_qualify(self):
        report,trace,_,_=self.run_case()
        for mutate in (lambda t:t.update(report_canonical_sha256='0'*64),
                       lambda t:t['events_by_port']['port0'][0].update(motor_id=12),
                       lambda t:t['events_by_port']['port0'][0].update(returned_bytes=16),
                       lambda t:t['events_by_port']['port0'][0].update(hex='00'*17)):
            changed=copy.deepcopy(trace);mutate(changed)
            with self.assertRaises(ValueError):self.validate(report,changed)

    def test_globally_silent_window_cannot_be_a_per_port_silence(self):
        report,trace,_,_=self.run_case()
        port=next(port for port,ids in self.base['topology_by_port'].items() if 1 not in ids)
        changed=copy.deepcopy(trace);events=changed['events_by_port'][port]
        end=next(i for i,event in enumerate(events) if event.get('kind')=='tx' and event.get('step')=='enable')
        offset=report['axes']['1']['last_zero_write_finish_ns']+1_000_000-events[0]['start_ns']
        # Retain each canonical request/reply and every same-port causal order.
        # Shift only this port's entire preflight before its first enable into
        # another port's silence. Global silence must reject otherwise valid IO.
        for event in events[:end]:
            for key in ('start_ns','finish_ns','received_ns'):
                if key in event:event[key]+=offset
        with self.assertRaisesRegex(ValueError,'CAN transmission occurred during watchdog silence'):
            self.validate(report,changed)

    def test_semantic_relabel_and_sticky_stop_ambiguity_do_not_qualify(self):
        report,trace,_,_=self.run_case()
        for mutate in (lambda r:r.update(positive_gain_sent=True),
                       lambda r:r['axes']['2'].update(disabled_on_command_loss=False),
                       lambda r:r['stop_reports']['port0'].update(ambiguous_ids=[7]),
                       lambda r:r['axes']['2'].update(disable_reply_upper_bound_ms=211)):
            changed=copy.deepcopy(report);mutate(changed)
            changed_trace=copy.deepcopy(trace);changed_trace['report_canonical_sha256']=HP.canonical_sha256(changed)
            with self.assertRaises(ValueError):self.validate(changed,changed_trace)

    def test_default_plan_opens_no_channel_and_calls_no_capabilities(self):
        error=lambda *_a,**_k:(_ for _ in ()).throw(AssertionError('capability called'))
        result=W.run({},self.contract,{},check=error,clock=error,wait=error,announce=error)
        self.assertEqual(result['status'],'PLAN_ONLY')
        self.assertFalse(result['hardware_opened'])
        self.assertFalse(result['positive_gain_sent'] or result['learned_targets_sent'])

    def test_cross_owner_shared_channel_and_six_axis_constructor_refused(self):
        with self.assertRaises(RuntimeError):W.Channel(None,(1,2,3,4,5,6))
        clock=Clock();endpoint=MemoryPort((1,2,3),self.base,clock)
        ch=W.Channel(endpoint,(1,2,3),clock=clock,reader=MemoryReader(endpoint,clock=clock,check=lambda:None))
        with self.assertRaises(RuntimeError):ch._wire(4,'enable')
        for step in ('positive_gain','save_flash','firmware_update','arbitrary'):
            with self.assertRaises(ValueError):ch._wire(1,step)

    def test_copied_public_grants_cannot_execute_watchdog_foreground(self):
        env=Mock()
        with self.assertRaises(ValueError):F.execute({'output_allowed':True},environment=env)
        self.assertEqual(env.mock_calls,[])

    def foreground_case(self,failure=None):
        """Explicit private test-token seam; no source/native/physical admission is claimed."""
        root=Path(self.fixture.tmp.name).resolve()
        receipt=root/'synthetic-native-receipt.json'
        raw=json.dumps({'native_current_guard':{'SYNTHETIC_ONLY':True}}).encode();receipt.write_bytes(raw)
        ref={'path':str(receipt),'sha256':hashlib.sha256(raw).hexdigest()}
        review=HP.build_review_plan(self.fixture.base)
        review['evidence']={'source_root':str(root/'synthetic-kit'),'source_manifest':{'path':str(root/'not-read'),'sha256':'9'*64},
                            'source_count':0,'implementation_receipt':ref}
        pins=HP.Pins();pins.raw(ref)
        capture={'boot_before':self.base['boot_id'],'ids_by_port':self.base['topology_by_port'],
                 'ports':{port:{'SYNTHETIC_ONLY':True} for port in HP.PORTS}}
        prepared=F.Prepared(F._TOKEN,review,pins,self.firmware,capture,
            {'path':'/SYNTHETIC_ONLY','sha256':'8'*64,'device':'SYNTHETIC','duration_s':.02},root/'foreground-run',None)
        clock=Clock();order=[];base=self.base
        class Guard:
            setup={'file_only_mock':True};calls=[]
            def __call__(self):pass
            def finish(self):
                order.append('guard_finish')
                if failure=='source_change':receipt.write_bytes(raw+b'changed')
                if failure=='guard_finish':raise ValueError('Synthetic end guard failed')
                return {'end_full_path_resolution_verified':True,'file_only_mock':True}
            def close(self):order.append('guard_close')
        class Environment:
            def clock(self):return clock()
            def startup(self):return {'file_only_mock_readback':True}
            def verify_terminal(self):return {'file_only_mock_visible_terminal':True}
            def boot_id(self):return base['boot_id']
            def bind(self,_):return copy.deepcopy(capture['ports'])
            def holders(self,_):return {'file_only_mock_no_holders':True}
            def load_guard(self,_):return object()
            def locks(self,stack,bindings):pass
            def channels(self,stack,bindings,check,ids_by_port,channels):
                for i,(port,ids) in enumerate(ids_by_port.items()):
                    if failure=='partial_open' and i==1:raise OSError('Synthetic second port open failed')
                    endpoint=MemoryPort(ids,base,clock)
                    channels[port]=W.Channel(endpoint,ids,clock=clock,check=check)
                    stack.callback(lambda port=port:order.append('close_'+port))
                return channels,{port:1 for port in channels},{port:2 for port in channels}
            def current_guard(self,*_):
                if failure=='guard_ctor':raise OSError('Synthetic guard ctor failed')
                return Guard()
            def announcer(self,*_):return lambda:None
        # Record each inherited physical-STOP attempt before fake descriptor close.
        original_stop=W.Channel.stop_all
        def stop(channel):
            result=original_stop(channel);order.append('stop_'+str(channel.ids[0]));return result
        with patch('singularitydog_hw.serial_deadline_reader.DeadlineSerialReader',MemoryReader), \
             patch('singularitydog_hw.watchdog_commissioning.time.sleep',clock.wait), \
             patch.object(W.Channel,'stop_all',stop), \
             patch.object(HP,'verify_manifest',return_value={}), \
             patch.object(W,'time',SimpleNamespace(monotonic_ns=clock,sleep=clock.wait)):
            # Pass actual virtual wait to the unchanged engine; original default
            # sleep is a function object frozen at definition, not module lookup.
            run=W.run
            with patch.object(W,'run',lambda *a,**kw:run(*a,**kw,wait=clock.wait)):
                report=F.execute(prepared,environment=Environment())
        return report,order

    def test_normal_foreground_closes_registered_guard_after_STOP_before_port_FDs(self):
        report,order=self.foreground_case()
        self.assertEqual(report['status'],'COMPLETE_COMMAND_LOSS_DIAGNOSTIC',report['errors'])
        self.assertTrue(report['stop_confirmed'] and report['restoration_complete'])
        for first in (1,4,7,10):self.assertLess(order.index('stop_'+str(first)),order.index('guard_finish'))
        self.assertLess(order.index('guard_finish'),order.index('guard_close'))
        self.assertLess(order.index('guard_close'),min(i for i,s in enumerate(order) if s.startswith('close_')))

    def test_partial_open_and_guard_constructor_failures_STOP_every_open_owner_before_close(self):
        for failure in ('partial_open','guard_ctor'):
            with self.subTest(failure=failure):
                report,order=self.foreground_case(failure)
                self.assertFalse(report['status'].startswith('COMPLETE'))
                self.assertFalse(report['motor_enable_sent'])
                self.assertFalse(report['restoration_complete'])
                for port in report['stop_reports']:
                    first=self.base['topology_by_port'][port][0]
                    self.assertLess(order.index('stop_'+str(first)),order.index('close_'+port))
                if failure=='partial_open':self.assertTrue(report['physical_cutoff_required'])
                # Each test needs a distinct fresh output, exactly like actual code.
                import shutil
                shutil.rmtree(Path(self.fixture.tmp.name).resolve()/'foreground-run')

    def test_source_or_guard_end_failure_retains_all_STOP_but_never_qualifies(self):
        for failure in ('source_change','guard_finish'):
            with self.subTest(failure=failure):
                report,order=self.foreground_case(failure)
                self.assertEqual(report['status'],'ABORTED')
                self.assertTrue(report['stop_confirmed'])
                self.assertFalse(report['positive_gain_sent'] or report['learned_targets_sent'])
                if failure=='source_change':self.assertFalse(report['source_and_input_pins_unchanged'])
                if failure=='guard_finish':self.assertFalse(report['restoration_complete'])
                import shutil
                shutil.rmtree(Path(self.fixture.tmp.name).resolve()/'foreground-run')


if __name__=='__main__':unittest.main()
