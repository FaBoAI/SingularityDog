"""No robot, serial device or positive-gain command is used in these tests."""
from contextlib import redirect_stdout
import io
import json
import math
import socket
import threading
import time
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import can_readonly as codec
from singularitydog_hw import watchdog_commissioning as watchdog


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now
    def wait(self, seconds): self.now += max(1, round(seconds*1e9))


def fb(mode, clock):
    clock.now += 500_000
    return {'mode_state': mode, 'fault_bits': 0, 'protocol_position_rad': 0.,
            'velocity_rad_s': 0., 'temperature_c': 25., 'write_finish_ns': clock.now-100_000,
            'request_start_ns': clock.now-200_000, 'received_ns': clock.now}


class FakeChannel:
    def __init__(self, ids, clock, *, bad=None):
        self.ids, self.clock, self.bad = ids, clock, bad
        self.calls, self.enabled, self.zero_counts = [], set(), {}
        self.stop_calls = 0
    def exchange(self, mid, step, center=0.):
        self.calls.append((self.clock(), mid, step))
        if self.bad == 'uid' and mid == 2 and step == 'identity':
            return {'mcu_uid_hex': '00'*8}
        if step == 'identity': return {'mcu_uid_hex': (bytes([mid])*8).hex()}
        if step == 'stop': self.enabled.discard(mid); return fb(0, self.clock)
        if step == 'version': return {'version_bytes_hex': '05001300', 'semantic_firmware_version': None}
        if step == 'voltage': return {'value': 34.9 if self.bad == 'voltage' else 40.}
        if step == 'run_mode': return {'value': 0}
        if step == 'watchdog_write': return fb(0, self.clock)
        if step == 'can_timeout': return {'value': 3999 if self.bad == 'watchdog' else 4000}
        if step == 'enable':
            self.enabled.add(mid)
            if self.bad == 'enable_timeout': raise TimeoutError('ambiguous enable')
            return fb(2, self.clock)
        if step == 'zero':
            count = self.zero_counts.get(mid, 0)+1; self.zero_counts[mid] = count
            if count == 1: return fb(2 if self.bad == 'autoenable' else 0, self.clock)
            if count == 2:
                value = fb(2, self.clock)
                if self.bad == 'late_write_completion':
                    value['request_start_ns'] -= 41_000_000
                return value
            if self.bad == 'late': self.clock.now += 41_000_000
            if self.bad != 'no_disable': self.enabled.discard(mid)
            value = fb(2 if mid in self.enabled else 0, self.clock)
            probe_overrides = {
                'probe_angle': {'protocol_position_rad': math.radians(3)+1e-6},
                'probe_velocity': {'velocity_rad_s': 10.1312275883},
                'probe_hot': {'temperature_c': 60.},
                'probe_cold': {'temperature_c': -10.01},
                'probe_bounds': {'protocol_position_rad': math.radians(3),
                                 'velocity_rad_s': -.5, 'temperature_c': -10.},
            }
            value.update(probe_overrides.get(self.bad, {}))
            return value
        raise AssertionError(step)
    def stop_all(self):
        self.stop_calls += 1; self.enabled.clear()
        return {'confirmed_ids': list(self.ids) if self.bad != 'stop' else [],
                'unconfirmed_ids': list(self.ids) if self.bad == 'stop' else [],
                'ambiguous_ids': [], 'errors': [], 'complete': self.bad != 'stop'}


class CommissioningTests(unittest.TestCase):
    def run_case(self, group='all', bad=None, **kwargs):
        clock = Clock()
        channels = {name: FakeChannel(ids, clock, bad=bad if name == 'front' else None)
                    for name, ids in watchdog.BUSES.items()}
        expected = {mid: (bytes([mid])*8).hex() for mid in watchdog.IDS}
        report = watchdog.run(channels, expected, group=group, clock=clock, wait=clock.wait, **kwargs)
        return report, channels
    def test_explicit_full_charge_limit_preserves_lower_bound_and_default(self):
        original=FakeChannel.exchange
        for maximum,value,complete in ((42,42.247,False),(43,42.247,True),
                (43,43.,True),(43,35.,True),(43,43.01,False),(43,34.99,False),
                (43,float('nan'),False),(43,float('inf'),False)):
            with self.subTest(maximum=maximum,value=value):
                def reply(channel,mid,step,center=0.):
                    if step=='voltage':return {'value':value}
                    return original(channel,mid,step,center)
                with patch.object(FakeChannel,'exchange',reply):
                    report,channels=self.run_case(voltage_max_v=maximum)
                self.assertEqual(report['status'],
                    'COMPLETE_COMMAND_LOSS_DIAGNOSTIC' if complete else 'ABORTED')
                self.assertEqual(report['voltage_max_v'],maximum)
                self.assertEqual(report['voltage_range_v'],[35,maximum])
                self.assertTrue(report['stop_confirmed'])
                if not complete:
                    self.assertFalse(report['motor_enable_sent'])
                    self.assertFalse(any(step=='enable' for c in channels.values() for _,_,step in c.calls))
        self.assertEqual(watchdog.plan()['voltage_max_v'],42)
        for value in (41,44,True,float('nan'),float('inf')):
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'Voltage maximum'):
                watchdog.plan(voltage_max_v=value)

    def test_voltage_cli_selection_is_saved_in_plan_without_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'uids.json'
            path.write_text(json.dumps({str(mid):(bytes([mid])*8).hex() for mid in watchdog.IDS}))
            for extra,maximum in (([],42),(['--voltage-max-v','43'],43)):
                output=io.StringIO()
                with redirect_stdout(output):
                    self.assertEqual(watchdog.main(['--expected-uids',str(path),*extra]),0)
                result=json.loads(output.getvalue())
                self.assertFalse(result['hardware_opened'])
                self.assertEqual(result['voltage_max_v'],maximum)
                self.assertEqual(result['voltage_range_v'],[35,maximum])
            with patch('sys.stderr',io.StringIO()),self.assertRaises(SystemExit):
                watchdog.main(['--expected-uids',str(path),'--voltage-max-v','44'])
    def test_all12_one_run_checks_watchdog_no_usb_or_approval_claim(self):
        report, channels = self.run_case()
        self.assertEqual(report['status'], 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC')
        self.assertTrue(report['stop_confirmed'])
        self.assertFalse(report['usb_disconnect_tested']); self.assertFalse(report['approved_for_runtime'])
        self.assertFalse(report['positive_gain_sent']); self.assertFalse(report['learned_targets_sent'])
        for mid in watchdog.IDS:
            axis = report['axes'][str(mid)]
            self.assertTrue(axis['command_loss_tested']); self.assertTrue(axis['disabled_on_command_loss'])
            self.assertGreaterEqual(axis['disable_reply_upper_bound_ms'], 210)
            self.assertLessEqual(axis['disable_reply_upper_bound_ms'], 250)
            self.assertEqual(axis['disable_upper_bound_origin'], 'last_zero_host_write_started_ns')
            self.assertAlmostEqual(axis['disable_reply_upper_bound_ms'],
                (axis['stop_probe']['received_ns']-axis['last_zero_write_start_ns'])/1e6)
        # No outgoing status reads in any silence interval, on either bus.
        calls = sorted(call for channel in channels.values() for call in channel.calls)
        for mid in watchdog.IDS:
            zeros = [call for call in calls if call[1:] == (mid, 'zero')]
            start, end = zeros[1][0], zeros[2][0]
            self.assertFalse(any(start < call[0] < end for call in calls))
        self.assertEqual(sum(c.stop_calls for c in channels.values()), 2)
    def test_group_selection_only_enables_the_requested_four(self):
        for name in ('calf', 'thigh', 'hip'):
            report, channels = self.run_case(name)
            enabled = sorted(mid for c in channels.values() for _, mid, step in c.calls if step == 'enable')
            self.assertEqual(enabled, list(watchdog.GROUPS[name]))
            self.assertEqual(len(report['axes']), 12)
    def test_pre_enable_failures_never_enable_and_stop_both_buses(self):
        for bad in ('uid', 'voltage', 'watchdog', 'autoenable'):
            with self.subTest(bad=bad):
                report, channels = self.run_case(bad=bad)
                self.assertEqual(report['status'], 'ABORTED')
                self.assertFalse(report['motor_enable_sent'])
                self.assertFalse(any(step == 'enable' for c in channels.values() for _, _, step in c.calls))
                self.assertEqual(sum(c.stop_calls for c in channels.values()), 2)
    def test_timeout_disable_failure_or_late_reply_does_not_continue_to_next_axis(self):
        for bad in ('enable_timeout', 'no_disable', 'late', 'late_write_completion'):
            with self.subTest(bad=bad):
                report, channels = self.run_case(bad=bad)
                self.assertEqual(report['status'], 'ABORTED'); self.assertTrue(report['motor_enable_sent'])
                self.assertEqual([mid for c in channels.values() for _, mid, step in c.calls if step == 'enable'], [1])
                self.assertTrue(report['stop_confirmed'])
    def test_stop_failure_keeps_physical_cutoff_requirement(self):
        report, _ = self.run_case(bad='stop')
        self.assertEqual(report['status'], 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
        self.assertFalse(report['stop_confirmed'])
    def test_disabled_probe_outside_motion_bounds_stops_before_next_enable(self):
        for bad, field, value in (
                ('probe_angle', 'protocol_position_rad', math.radians(3)+1e-6),
                ('probe_velocity', 'velocity_rad_s', 10.1312275883),
                ('probe_hot', 'temperature_c', 60.),
                ('probe_cold', 'temperature_c', -10.01)):
            with self.subTest(bad=bad):
                report, channels = self.run_case(bad=bad)
                self.assertEqual(report['status'], 'ABORTED')
                self.assertTrue(report['stop_confirmed'])
                self.assertIn('after command silence', report['errors'][0])
                axis = report['axes']['1']
                self.assertEqual(axis['stop_probe']['mode_state'], 0)
                self.assertEqual(axis['stop_probe'][field], value)
                self.assertLessEqual(axis['disable_reply_upper_bound_ms'], 250)
                self.assertFalse(axis['command_loss_tested'])
                self.assertFalse(axis['disabled_on_command_loss'])
                self.assertEqual([mid for c in channels.values()
                                  for _, mid, step in c.calls if step == 'enable'], [1])
                self.assertTrue(all(c.stop_calls == 1 for c in channels.values()))
                self.assertFalse(any(c.enabled for c in channels.values()))
    def test_disabled_probe_uses_existing_inclusive_motion_bounds(self):
        report, _ = self.run_case(bad='probe_bounds')
        self.assertEqual(report['status'], 'COMPLETE_COMMAND_LOSS_DIAGNOSTIC', report['errors'])
        self.assertTrue(report['stop_confirmed'])
    def test_unsafe_disabled_probe_with_missing_stop_keeps_cutoff_requirement(self):
        original = FakeChannel.stop_all
        def stop_with_front_failure(channel):
            result = original(channel)
            if channel.ids == watchdog.BUSES['front']:
                result.update(complete=False, confirmed_ids=[],
                              unconfirmed_ids=list(channel.ids), errors=['STOP reply missing'])
            return result
        with patch.object(FakeChannel, 'stop_all', stop_with_front_failure):
            report, channels = self.run_case(bad='probe_velocity')
        self.assertEqual(report['status'], 'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
        self.assertFalse(report['stop_confirmed'])
        self.assertEqual(report['axes']['1']['stop_probe']['velocity_rad_s'], 10.1312275883)
        self.assertTrue(all(c.stop_calls == 1 for c in channels.values()))
        self.assertEqual([mid for c in channels.values()
                          for _, mid, step in c.calls if step == 'enable'], [1])
    def test_interrupt_and_announcement_failure_stop_without_enable(self):
        def interrupt(): raise InterruptedError('cancel')
        for kwargs in ({'check': interrupt}, {'announce': interrupt}):
            report, channels = self.run_case(**kwargs)
            self.assertEqual(report['status'], 'ABORTED'); self.assertFalse(report['motor_enable_sent'])
            self.assertEqual(sum(c.stop_calls for c in channels.values()), 2)
    def test_default_cli_opens_no_hardware(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder)/'uid.json'
            source.write_text(json.dumps({str(mid): (bytes([mid])*8).hex() for mid in watchdog.IDS}))
            stream = io.StringIO()
            with redirect_stdout(stream), patch.object(watchdog, 'Channel') as channel:
                self.assertEqual(watchdog.main(['--expected-uids', str(source)]), 0)
                channel.assert_not_called()
            self.assertFalse(json.loads(stream.getvalue())['hardware_opened'])
    def test_wire_scope_cannot_send_positive_gains_flash_or_broadcast(self):
        channel = object.__new__(watchdog.Channel); channel.ids = watchdog.BUSES['front']
        for mid in channel.ids:
            parsed = codec.ATParser().feed(channel._wire(mid, 'zero', .5))[0]
            _, velocity, kp, kd = struct.unpack('>4H', parsed.data)
            self.assertEqual((velocity, kp, kd), (32767, 0, 0))
            self.assertEqual((parsed.can_id >> 8)&65535, 32767)
        for mid, step in ((0, 'zero'), (7, 'enable'), (1, 'save'), (1, 'motion')):
            with self.assertRaises((ValueError, RuntimeError)): channel._wire(mid, step)

    def test_receive_setup_failure_still_attempts_six_stops(self):
        class BrokenPort:
            def __init__(self): self.writes=[]
            @property
            def in_waiting(self): raise OSError('broken receive ioctl')
            def fileno(self): raise OSError('broken receive fd')
            def write(self, data): self.writes.append(data);return len(data)
        port=BrokenPort();channel=object.__new__(watchdog.Channel)
        channel.port=port;channel.ids=watchdog.BUSES['front'];channel.clock=time.monotonic_ns
        channel.check=lambda: None;channel.pending={};channel.parser=codec.ATParser()
        channel.events=[];channel.reader=object()
        result=channel.stop_all()
        self.assertEqual([codec.ATParser().feed(raw)[0].destination for raw in port.writes],[1,2,3,4,5,6])
        self.assertFalse(result['complete']);self.assertEqual(result['unconfirmed_ids'],[1,2,3,4,5,6])
    def test_consumed_but_rejected_raw_is_saved_without_becoming_a_reply(self):
        from singularitydog_hw.serial_deadline_reader import ReceivedChunk
        error=TimeoutError('late read');error.serial_read_evidence=ReceivedChunk(b'AT',10,20)
        class Reader:
            def read_until(self, wake, hard): raise error
        channel=object.__new__(watchdog.Channel);channel.reader=Reader();channel.events=[]
        with self.assertRaises(TimeoutError) as caught: channel._read(10,20)
        self.assertIs(caught.exception,error)
        self.assertEqual(channel.events[0]['kind'],'rx_rejected')
        self.assertEqual(channel.events[0]['hex'],'4154')


def wire(can_id, data):
    return b'AT'+((can_id<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'


class SocketPort:
    def __init__(self, sock): self.sock=sock;self.timeout=0;self.write_timeout=.02
    def fileno(self): return self.sock.fileno()
    def write(self, data): return self.sock.send(data)
    @property
    def in_waiting(self):
        try: return len(self.sock.recv(4096, socket.MSG_PEEK))
        except BlockingIOError: return 0


class TransportTests(unittest.TestCase):
    """Exercise actual select/os.read requests, parsing and STOP over sockets."""
    def sockets(self, ids, *, failure=None):
        host,peer=socket.socketpair();host.setblocking(False);peer.settimeout(.1)
        seen=[];errors=[];closing=threading.Event()
        def device():
            parser=codec.ATParser();enabled=set();last={}
            try:
                while not closing.is_set():
                    try: data=peer.recv(4096)
                    except socket.timeout: continue
                    if not data: break
                    for f in parser.feed(data):
                        mid=f.destination;seen.append(f)
                        if mid not in ids: raise ValueError('cross bus')
                        if mid in last and time.monotonic_ns()-last[mid]>=200_000_000: enabled.discard(mid)
                        if f.kind==0: reply=wire((mid<<8)|0xfe,bytes([mid])*8)
                        elif f.kind==17:
                            index=int.from_bytes(f.data[:2],'little')
                            value={0x7005:bytes(4),0x701c:struct.pack('<f',40.),0x7028:struct.pack('<I',4000)}[index]
                            reply=wire((17<<24)|(mid<<8)|0xfd,f.data[:4]+value)
                        elif f.kind==18:
                            if failure=='drop_watchdog_write': continue
                            reply_mid=mid+1 if failure=='wrong_watchdog_write' else mid
                            reply_mode=2 if failure=='active_watchdog_write' else 0
                            reply=wire((2<<24)|(reply_mode<<22)|(reply_mid<<8)|0xfd,
                                       struct.pack('>4H',32767,32767,32767,250))
                            if failure=='partial_watchdog_write':
                                peer.sendall(reply[:9]);time.sleep(.005);reply=reply[9:]
                        elif f.kind==4 and f.data[:2]==b'\x00\xc4':
                            reply=wire((2<<24)|(mid<<8)|0xfd,bytes.fromhex('00c45605001300a5'))
                            if failure=='ordinary_as_version': reply=wire((2<<24)|(mid<<8)|0xfd,struct.pack('>4H',32767,32767,32767,250))
                        else:
                            if f.kind==3: enabled.add(mid);last[mid]=time.monotonic_ns()
                            elif f.kind==4: enabled.discard(mid)
                            elif f.kind==1:
                                p,v,kp,kd=struct.unpack('>4H',f.data)
                                if (v,kp,kd)!=(32767,0,0): raise ValueError('nonzero gain')
                                last[mid]=time.monotonic_ns()
                            else: raise ValueError('unexpected kind')
                            reply=wire((2<<24)|((2 if mid in enabled else 0)<<22)|(mid<<8)|0xfd,
                                       struct.pack('>4H',32767,32767,32767,250))
                        if failure=='drop_enable' and f.kind==3: continue
                        if failure=='drop_stop_3' and f.kind==4 and f.data==bytes(8) and mid==ids[2]: continue
                        if failure=='stop_tail' and f.kind==4 and f.data==bytes(8) and mid==ids[-1]: reply+=b'AT'
                        peer.sendall(reply)
            except BaseException as error:
                if not closing.is_set(): errors.append(error)
        thread=threading.Thread(target=device,daemon=True);thread.start()
        def cleanup():
            closing.set();host.close();peer.close();thread.join(timeout=.5)
            self.assertFalse(thread.is_alive());self.assertFalse(errors,errors)
        self.addCleanup(cleanup)
        return watchdog.Channel(SocketPort(host), ids),seen
    def test_actual_two_socket_supported_zero_gain_group(self):
        channels={name:self.sockets(ids)[0] for name,ids in watchdog.BUSES.items()}
        expected={mid:(bytes([mid])*8).hex() for mid in watchdog.IDS}
        report=watchdog.run(channels,expected,group='hip')
        self.assertEqual(report['status'],'COMPLETE_COMMAND_LOSS_DIAGNOSTIC',report['errors'])
        self.assertTrue(report['stop_confirmed'])
        for mid in watchdog.GROUPS['hip']:
            self.assertEqual(report['axes'][str(mid)]['version']['version_bytes_hex'],'05001300')
    def test_normal_feedback_is_not_firmware_identity(self):
        channel,seen=self.sockets(watchdog.BUSES['front'],failure='ordinary_as_version')
        with self.assertRaises(ValueError): channel.exchange(1,'version')
        self.assertTrue(channel.failed)
        self.assertFalse(any(f.kind==3 for f in seen))
    def test_watchdog_write_consumes_one_state_reply_before_separate_parameter_read(self):
        channel,seen=self.sockets(watchdog.BUSES['front'],failure='partial_watchdog_write')
        state=channel.exchange(1,'watchdog_write')
        self.assertEqual(state['mode_state'],0)
        self.assertGreater(state['received_ns'],state['write_finish_ns'])
        self.assertEqual(len(seen),1)
        readback=channel.exchange(1,'can_timeout')
        self.assertEqual(readback['value'],4000)
        self.assertEqual([f.kind for f in seen],[18,17])
        second_tx=[e for e in channel.events if e['kind']=='tx'][1]
        self.assertGreater(second_tx['start_ns'],state['received_ns'])
    def test_watchdog_write_rejects_wrong_source_or_active_state(self):
        for failure in ('wrong_watchdog_write','active_watchdog_write'):
            with self.subTest(failure=failure):
                channel,seen=self.sockets(watchdog.BUSES['front'],failure=failure)
                with self.assertRaises((ValueError,RuntimeError)):
                    channel.exchange(1,'watchdog_write')
                self.assertTrue(channel.failed)
                self.assertEqual([f.kind for f in seen],[18])
    def test_timed_out_watchdog_write_cannot_be_miscounted_as_stop_ack(self):
        channel,_=self.sockets(watchdog.BUSES['front'],failure='drop_watchdog_write')
        with self.assertRaises(TimeoutError): channel.exchange(1,'watchdog_write')
        stopped=channel.stop_all()
        self.assertEqual(stopped['ambiguous_ids'],[1])
        self.assertEqual(stopped['unconfirmed_ids'],[1])
        self.assertEqual(stopped['confirmed_ids'],[2,3,4,5,6])
    def test_timed_out_enable_stop_reply_remains_ambiguous(self):
        channel,_=self.sockets(watchdog.BUSES['front'],failure='drop_enable')
        with self.assertRaises(TimeoutError): channel.exchange(1,'enable')
        failure=next(event for event in channel.events if event['kind']=='exchange_failure')
        counters=failure['reader_counters_delta']
        self.assertGreaterEqual(counters['select_calls'],1)
        self.assertEqual(counters['read_calls'],0)
        self.assertEqual(counters['bytes_received'],0)
        self.assertEqual(counters['hard_expiries'],1)
        self.assertEqual(failure['hard_deadline_ns']-failure['request_start_ns'],watchdog.REQUEST_NS)
        self.assertEqual(failure['receive_boundary_evidence'],
                         dict(partial_hex='',discarded_bytes=0,backlogged_bytes=0))
        saved=json.dumps(failure,sort_keys=True)
        stopped=channel.stop_all()
        self.assertIn(1,stopped['ambiguous_ids']);self.assertIn(1,stopped['unconfirmed_ids'])
        self.assertEqual(stopped['confirmed_ids'],[2,3,4,5,6])
        self.assertEqual(json.dumps(failure,sort_keys=True),saved)
    def test_cleanup_awaits_each_axis_before_next_stop(self):
        channel,seen=self.sockets(watchdog.BUSES['front'])
        stopped=channel.stop_all()
        self.assertTrue(stopped['complete'],stopped)
        self.assertEqual([f.destination for f in seen],[1,2,3,4,5,6])
        tx=[(i,e) for i,e in enumerate(channel.events) if e['kind']=='tx']
        for (left,a),(right,b) in zip(tx,tx[1:]):
            between=channel.events[left+1:right]
            self.assertTrue(any(e['kind']=='rx_bytes' for e in between))
            self.assertGreaterEqual(b['start_ns']-a['finish_ns'],800_000)
    def test_cleanup_missing_reply_still_stops_every_remaining_axis(self):
        channel,seen=self.sockets(watchdog.BUSES['front'],failure='drop_stop_3')
        started=time.monotonic()
        stopped=channel.stop_all()
        self.assertLess(time.monotonic()-started,1.5)
        self.assertFalse(stopped['complete'])
        self.assertEqual(stopped['unconfirmed_ids'],[3])
        self.assertEqual(stopped['confirmed_ids'],[1,2,4,5,6])
        self.assertEqual([f.destination for f in seen],[1,2,3,4,5,6])
    def test_trailing_partial_frame_prevents_successful_stop_claim(self):
        channel,_=self.sockets(watchdog.BUSES['front'],failure='stop_tail')
        result=channel.stop_all()
        self.assertEqual(result['confirmed_ids'],[1,2,3,4,5,6])
        self.assertFalse(result['complete']);self.assertTrue(result['errors'])


class FailedExchangeEvidenceTests(unittest.TestCase):
    """Keep host failure evidence without changing request or STOP semantics."""
    def channel(self, *, chunks=(), returned_bytes=17, backlog=0, broken_stats=False):
        clock=Clock()
        class Port:
            def __init__(self): self.writes=[]
            def write(self,data): self.writes.append(data);return returned_bytes
            @property
            def in_waiting(self):
                if isinstance(backlog,BaseException): raise backlog
                return backlog
        error=TimeoutError('fixed deadline')
        class Reader:
            def __init__(self): self.pending=list(chunks);self.calls=0;self.received=0
            def stats(self):
                if broken_stats: raise OSError('counter unavailable')
                return dict(read_until_calls=self.calls,bytes_received=self.received)
            def read_until(self,wake,hard):
                self.calls+=1
                if self.pending:
                    data=self.pending.pop(0);self.received+=len(data);clock.now+=100_000
                    return data,clock.now
                clock.now=hard
                raise error
        port=Port();reader=Reader()
        channel=watchdog.Channel(port,watchdog.BUSES['front'],clock=clock,reader=reader)
        return channel,port,error

    def test_partial_receive_boundary_and_exact_counters_survive_parser_replacement(self):
        channel,port,error=self.channel(chunks=(b'AT',))
        with self.assertRaises(TimeoutError) as caught: channel.exchange(1,'enable')
        self.assertIs(caught.exception,error)
        failure=channel.events[-1]
        self.assertEqual(failure['kind'],'exchange_failure')
        self.assertEqual(failure['reader_counters_delta'],dict(read_until_calls=2,bytes_received=2))
        self.assertEqual(failure['receive_boundary_evidence']['partial_hex'],'4154')
        self.assertEqual(failure['event_start_index'],0)
        self.assertEqual(failure['event_end_index'],2)
        self.assertFalse(failure['automatic_retry'])
        self.assertEqual(channel.pending,{1:'enable'})
        self.assertEqual(len(port.writes),1)
        channel.parser=codec.ATParser()
        self.assertEqual(failure['receive_boundary_evidence']['partial_hex'],'4154')

    def test_unclean_start_boundary_records_no_request_or_receive_and_sends_nothing(self):
        channel,port,_=self.channel(backlog=17)
        with self.assertRaisesRegex(RuntimeError,'No fresh request boundary'):
            channel.exchange(1,'enable')
        failure=channel.events[-1]
        self.assertIsNone(failure['request_start_ns']);self.assertIsNone(failure['hard_deadline_ns'])
        self.assertEqual(failure['reader_counters_delta'],dict(read_until_calls=0,bytes_received=0))
        self.assertEqual(failure['receive_boundary_evidence']['backlogged_bytes'],17)
        self.assertFalse(port.writes)

    def test_partial_write_is_recorded_with_no_receive_or_retry(self):
        channel,port,_=self.channel(returned_bytes=16)
        with self.assertRaisesRegex(RuntimeError,'Partial write; no retry'):
            channel.exchange(1,'enable')
        failure=channel.events[-1]
        self.assertEqual(channel.events[0]['returned_bytes'],16)
        self.assertEqual(failure['reader_counters_delta'],dict(read_until_calls=0,bytes_received=0))
        self.assertEqual(failure['hard_deadline_ns']-failure['request_start_ns'],watchdog.REQUEST_NS)
        self.assertEqual(channel.pending,{1:'enable'});self.assertEqual(len(port.writes),1)

    def test_broken_counters_or_boundary_never_replace_the_original_error(self):
        for options,expected in ((dict(broken_stats=True),'counter unavailable'),
                                 (dict(backlog=OSError('boundary unavailable')),'boundary unavailable')):
            with self.subTest(options=options):
                channel,_,error=self.channel(**options)
                with self.assertRaises((TimeoutError,OSError)) as caught: channel.exchange(1,'enable')
                if options.get('broken_stats'): self.assertIs(caught.exception,error)
                failure=channel.events[-1]
                self.assertEqual(failure['kind'],'exchange_failure')
                self.assertTrue(any(expected in item for item in failure['diagnostic_errors']))
                if options.get('broken_stats'):
                    self.assertIsNone(failure['reader_counters_before'])
                    self.assertIsNone(failure['reader_counters_after'])
                    self.assertIsNone(failure['reader_counters_delta'])

    def test_failure_storage_error_keeps_original_exception_and_failed_channel(self):
        channel,_,error=self.channel()
        original_event=channel.event
        def fail_only_failure(value):
            if value['kind']=='exchange_failure': raise OSError('storage unavailable')
            original_event(value)
        channel.event=fail_only_failure
        with self.assertRaises(TimeoutError) as caught: channel.exchange(1,'enable')
        self.assertIs(caught.exception,error)
        self.assertTrue(channel.failed);self.assertTrue(channel.unlogged_exchange_failure)
        self.assertEqual(channel.pending,{1:'enable'})


if __name__ == '__main__': unittest.main()
