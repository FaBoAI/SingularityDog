#!/usr/bin/env python3
"""Real active C++ transport + output coordinator over two LOCAL socketpairs.

Optional real pinned TorchScript policy. No serial device, network, SSH, physical
review artifact, hardware permission, or hardware timing claim is produced.
The synthetic plant has instantaneous tracking and no physical dynamics.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO/'runtime'), str(REPO/'runtime/experiments')]
from singularitydog_hw import can_readonly as codec
from singularitydog_hw import native_active_transport as native
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw import policy_shadow as shadow
from singularitydog_hw.policy_motion_envelope import AxisLimits

BOOT = '11111111-2222-3333-4444-555555555555'
SCOPES = {'front': tuple(range(1,7)), 'rear': tuple(range(7,13))}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes():
    runtime_dir=REPO/'runtime/singularitydog_hw'
    files={'tool':Path(__file__),'coordinator':Path(runtime.__file__),
        'active_python':Path(native.__file__),
        'active_cpp':REPO/'runtime/experiments/native_active_transport/transport.cpp',
        'model_adapter':runtime_dir/'policy_output_model.py',
        'profile':runtime_dir/'policy_live_profile.py',
        'envelope':runtime_dir/'policy_motion_envelope.py',
        'cli':runtime_dir/'policy_output.py'}
    return {key:digest(path) for key,path in files.items()}


def wire(can_id, payload):
    return b'AT'+((can_id << 3)|4).to_bytes(4,'big')+b'\x08'+payload+b'\r\n'


def quantize(value, limit):
    require(math.isfinite(value) and -limit <= value <= limit, 'Synthetic encoder overflow; no clipping')
    return int((value+limit)*65535./(2.*limit))


def synthetic_profile():
    """In-memory test gate only. This is deliberately NOT a load_profile artifact."""
    axes={}
    for index, mid in enumerate(shadow.CAN_ORDER):
        limits=AxisLimits(shadow.LOWER[index], shadow.UPPER[index],4.,.2,
                          .1,1.,.3,1.,3.,60.,3.,.175)
        axes[str(mid)]={**asdict(limits), 'uid':(bytes([mid])*8).hex(),'sign':1,
                        'offset_rad':0., 'physical_lower_rad':limits.lower_rad-.001,
                        'physical_upper_rad':limits.upper_rad+.001,
                        'uncertainty_rad':.001}
    return {'schema':'singularitydog.supported-policy-profile.v2',
        'request_gap_us':600,'request_window':3,
        'simulation_only':True,'physical_review_generated':False,
        'motor_power_epoch':'SIMULATED_NO_MOTOR_POWER',
        'output_allowed':True, 'duration_s':1.2,'startup_duration_s':.2,
        'policy_ramp_s':.2,'stop_duration_s':.15,'policy_weight':.05,
        'max_sample_age_ms':100.,'max_sample_gap_ms':100.,'hard_cycle_ms':100.,
        # Offline OS scheduling is not being approved for real-time motor control.
        'max_consecutive_20ms_misses':100,
        'voltage_min_v':35.,'voltage_max_v':42.,'type2_position_tolerance_rad':.02,
        'command':[0.,0.,0.],'h_hypothesis':0,
        'imu_accel_norm_min_m_s2':9.5,'imu_accel_norm_max_m_s2':10.1,
        'imu_tilt_limit_rad':.1,'imu_gyro_limit_rad_s':.1,'axes':axes,
        # Synthetic raw fingerprint, never a claim about a motor FW label.
        'watchdog_by_id':{str(i):{'configured_timeout_ms':200.,'max_observed_disable_ms':180.,
                                 'firmware_version':None,'version_bytes_hex':'05001300'} for i in range(1,13)}}


def initial_positions():
    return {mid: quantize(q,12.57) for mid,q in zip(shadow.CAN_ORDER,[0.,.4,-.8]*4)}


class SyntheticIMU:
    def __init__(self):self.calls=0
    def __call__(self):
        start=time.monotonic_ns();self.calls+=1
        return {'simulation_only':True,'frame':'sensor','accel_m_s2':[0.,0.,9.80665],
            'gyro_rad_s':[0.,0.,0.],'read_started_monotonic_ns':start,
            'read_finished_monotonic_ns':time.monotonic_ns()}


class SocketPlant:
    """Byte-accurate protocol peer, NOT a motor dynamics or watchdog simulation."""
    def __init__(self, sock, ids, *, failure=None):
        self.sock,self.ids,self.failure=sock,ids,failure
        self.positions={i:initial_positions()[i] for i in ids}
        self.enabled=set();self.timeout={i:0 for i in ids}
        self.parser=codec.ATParser();self.rows=[];self.errors=[]
        self.rx_bytes=0;self.injected=False;self.injection_ns=None
        self.thread=threading.Thread(target=self.run,name='simulated-active-can-'+str(ids[0]),daemon=True)

    def respond(self, f):
        mid=f.destination
        require(mid in self.ids and f.flags==4,'Synthetic peer rejects cross-bus/standard frames')
        mode=None
        if f.kind==0:
            require(f.wire==codec.read_request(mid),'Noncanonical identity request')
            return wire((mid<<8)|0xfe,bytes([mid])*8)
        if f.kind==17:
            index=int.from_bytes(f.data[:2],'little')
            require(f.data[2:]==bytes(6),'Noncanonical Type17 request')
            name,(_,fmt,_)=next((name,item) for name,item in codec.PARAMETERS.items() if item[0]==index)
            value={'position':self.positions[mid]*25.14/65535.-12.57,
                   'velocity':0.,'run_mode':0,'voltage':40.,'can_timeout':self.timeout[mid]}[name]
            return wire((17<<24)|(mid<<8)|0xfd,(f.data[:4]+struct.pack('<'+fmt,value)).ljust(8,b'\0'))
        if f.kind==18:
            require(f.data==bytes.fromhex('28700000a00f0000'),'Only exact watchdog setup is simulated')
            # The write returns Type2 status; Type17 separately verifies ticks.
            self.timeout[mid]=4000
        elif f.kind==3:
            require(f.data==bytes(8),'Nonzero enable request');self.enabled.add(mid)
        elif f.kind==4:
            if f.data==bytes.fromhex('00c4000000000000'):
                require(mid not in self.enabled,'Synthetic version query requires stopped motor')
                return wire((2<<24)|(mid<<8)|0xfd,bytes.fromhex('00c45605001300a5'))
            require(f.data==bytes(8),'STOP must not clear faults');self.enabled.discard(mid)
        elif f.kind==1:
            require(mid in self.enabled,'Motion before explicit enable')
            p,v,kp,kd=struct.unpack('>4H',f.data)
            require(v==32767 and ((f.can_id>>8)&65535)==32767,'Nonzero synthetic velocity/FF')
            self.positions[mid]=p
            if self.failure and not self.injected and mid==self.ids[0] and kp>0:
                self.injected=True;self.injection_ns=time.monotonic_ns()
                if self.failure=='drop-feedback':return b''
                if self.failure=='usb-disconnect':
                    self.sock.shutdown(socket.SHUT_RDWR);self.sock.close();return None
                raise ValueError('Unknown synthetic failure')
        else:
            raise ValueError('Unexpected command in synthetic plant: '+str(f.kind))
        mode=2 if mid in self.enabled else 0
        return wire((2<<24)|(mode<<22)|(mid<<8)|0xfd,
                    struct.pack('>4H',self.positions[mid],32767,32767,250))

    def run(self):
        try:
            self.sock.settimeout(.2);deadline=time.monotonic()+12.
            while time.monotonic()<deadline:
                try:raw=self.sock.recv(4096)
                except socket.timeout:continue
                if not raw:break
                self.rx_bytes+=len(raw)
                for f in self.parser.feed(raw):
                    row={'observed_monotonic_ns':time.monotonic_ns(),'kind':f.kind,
                         'motor_id':f.destination,'request_hex':f.wire.hex(),'reply_hex':''}
                    self.rows.append(row)
                    response=self.respond(f)
                    if response is None:return
                    if response:
                        self.sock.sendall(response);row['reply_hex']=response.hex()
                        row['reply_sent_monotonic_ns']=time.monotonic_ns()
                require(not self.parser.discarded_bytes,'Synthetic request parser discarded bytes')
            else:raise TimeoutError('Finite local peer deadline exceeded')
            require(not self.parser.buffer,'Partial simulator request at EOF')
        except BaseException as error:
            self.errors.append(type(error).__name__+': '+str(error))
            try:self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:pass

    def evidence(self):
        kinds=Counter(r['kind'] for r in self.rows)
        return {'simulation_only':True,'request_count':len(self.rows),
            'reply_count':sum(bool(r['reply_hex']) for r in self.rows),'request_types':dict(kinds),
            'request_bytes':self.rx_bytes,'reply_bytes':sum(len(bytes.fromhex(r['reply_hex'])) for r in self.rows),
            'enabled_ids_at_end_in_simulator':sorted(self.enabled),'failure_injected':self.injected,
            'failure_kind':self.failure,'injection_monotonic_ns':self.injection_ns,
            'errors':self.errors,'discarded_bytes':self.parser.discarded_bytes,
            'residual_bytes':len(self.parser.buffer),'thread_joined':not self.thread.is_alive(),
            'rows':self.rows}


class RecordedPolicy:
    def __init__(self, policy, *, real_model):
        self.policy,self.real_model=policy,real_model;self.calls=[]
    def validate_inputs(self,*args):
        if hasattr(self.policy,'validate_inputs'):return self.policy.validate_inputs(*args)
    def __call__(self,sample,imu,now_ns):
        values=self.validate_inputs(sample,imu,now_ns)
        target=tuple(self.policy(sample,imu,now_ns))
        self.calls.append({'simulation_only':True,'call_monotonic_ns':now_ns,'inputs':values,
                           'target_can_order':list(target)})
        return target


def write_json(path,value):
    fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
    with os.fdopen(fd,'w') as stream:
        json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def learned_policy(profile,state_path,bundle,out):
    from singularitydog_hw.policy_output_model import LivePolicyModel
    state=json.loads(Path(state_path).read_text())
    mount={'schema_version':1,'status':'IMU_MOUNT_CANDIDATE_ONLY','input_frame':'sensor',
        'output_frame':'body_x_forward_y_left_z_up','R_body_from_sensor':[[1,0,0],[0,1,0],[0,0,1]],
        'raw_driver_axes_verified':False,'approved_for_runtime':False,
        'provenance':{'scope':'simulation_only','description':'Identity rotation for synthetic upright IMU'}}
    # Null is explicitly no gyro-bias correction; do not fabricate a stationary-fit review.
    write_json(out/'synthetic-mount.json',mount);write_json(out/'no-bias.json',None)
    profile['bundle_path']=str(bundle)
    profile['artifacts']={'mount':{'path':str(out/'synthetic-mount.json'),'sha256':digest(out/'synthetic-mount.json')},
        'bias':{'path':str(out/'no-bias.json'),'sha256':digest(out/'no-bias.json')},'model_manifest':{
            'path':state['native_policy_manifest'],'sha256':state['native_policy_manifest_sha256']}}
    model=LivePolicyModel(profile)
    return RecordedPolicy(model,real_model=True),model.provenance


def eager_parity(recorded,bundle):
    """Replay exact synthetic model inputs; compare target and final model state."""
    import torch
    from native_policy_overnight.verification import compare_state,compare_tensor
    from singularitydog_hw.policy_observer_replay import warmup_policy
    eager,_=shadow.load_policy(bundle);warmup_policy(eager,torch,0,10)
    maxima={}
    with torch.inference_mode():
        eager.reset(torch.tensor([0],dtype=torch.long))
        for row in recorded.calls:
            result=eager(*(torch.tensor([values],dtype=torch.float32) for values in row['inputs']))
            expected=torch.tensor([[row['target_can_order'][i-1] for i in shadow.CAN_ORDER]],dtype=torch.float32)
            compare_tensor(torch,result,expected,'target',maxima)
        compare_state(torch,eager,recorded.policy.policy,maxima)
    return {'status':'PASS','calls':len(recorded.calls),'max_errors':maxima,
            'final_parameters_and_state_compared':True}


def audit_case(report,peers,case,recorded):
    """Derive assertions from real emitted bytes and coordinator output."""
    runtime_report=report['runtime'];cycles=runtime_report['cycles']
    allrows=[row for peer in peers.values() for row in peer.rows]
    require(all(row['kind'] in (0,1,3,4,17,18) for row in allrows),'Unexpected frame kind')
    require(all(not peer.errors and not peer.parser.buffer and not peer.parser.discarded_bytes for peer in peers.values()),
            'Emulator failed or retained malformed bytes')
    require(all(not peer.thread.is_alive() for peer in peers.values()),'Emulator thread still running')
    sent_enables={row['motor_id'] for row in allrows if row['kind']==3}
    require(sent_enables==set(range(1,13)),'Expected all twelve explicit enables')
    motion=[row for row in allrows if row['kind']==1]
    require(bool(motion),'No actual native motion bytes exercised')
    require(all(struct.unpack('>4H',bytes.fromhex(r['request_hex'])[7:15])[1]==32767 for r in motion),
            'Unexpected velocity command')
    require(all(((int.from_bytes(bytes.fromhex(r['request_hex'])[2:6],'big')>>3)>>8)&65535==32767 for r in motion),
            'Unexpected FF command')
    phases=sorted(set(r['phase'] for r in cycles))
    audit={'status':'PASS','phase_set':phases,'native_motion_frames':len(motion),
           'enable_ids':sorted(sent_enables),'policy_calls':len(recorded.calls),
           'physical_review_generated':False,'real_motor_commands_sent':False}
    if case=='real-success':
        require(runtime_report['status']=='COMPLETE_SUPPORTED_OUTPUT',str(runtime_report['errors']))
        require({'starting','active','stopping','stopped'}<=set(phases),'Missing gain/stop phase')
        require(runtime_report['normal_ramp_completed'] and runtime_report['stop_confirmed'],'Normal ramp/STOP incomplete')
        require(runtime_report['learned_targets_sent'] and len(recorded.calls)>5,'Real learned target path not exercised')
        require(cycles[-1]['command']['gain_scale']==0.,'Final gain not zero')
        require(all(not peer.enabled for peer in peers.values()),'Synthetic actuator still enabled')
        target_changed=False
        max_dq=0.;max_dv=0.
        for a,b in zip(cycles,cycles[1:]):
            dt=b['command']['monotonic_s']-a['command']['monotonic_s']
            require(dt>0,'Nonincreasing synthetic cycle time')
            dq=max(abs(x-y) for x,y in zip(a['command']['q_model_rad'],b['command']['q_model_rad']))
            dv=max(abs(x-y) for x,y in zip(a['command']['command_velocity_rad_s'],b['command']['command_velocity_rad_s']))
            require(dq<=.1*dt+1e-10 and dv<=1.*dt+1e-10,'Envelope continuity violated')
            max_dq=max(max_dq,dq);max_dv=max(max_dv,dv)
        initial=runtime_report['initial_raw_rad_by_id']
        for r in motion:
            q=struct.unpack('>4H',bytes.fromhex(r['request_hex'])[7:15])[0]*25.14/65535.-12.57
            target_changed|=abs(q-initial[r['motor_id']])>.001
        require(target_changed,'Learned blend did not reach native command bytes')
        for peer in peers.values():
            last_motion=max(index for index,r in enumerate(peer.rows) if r['kind']==1)
            terminal=peer.rows[last_motion+1:]
            require([(r['kind'],r['motor_id']) for r in terminal]==[(4,i) for i in peer.ids],
                    'Terminal STOP bytes differ from one all-axis pass')
        audit.update(normal_stop_gain_zero=True, learned_blend_changed_command_bytes=True,
                     max_command_position_step_rad=max_dq,max_command_velocity_step_rad_s=max_dv)
    else:
        require(peers['front'].injected,'Requested failure was not injected')
        require(not runtime_report['normal_ramp_completed'],'Fault wrongly used a normal ramp')
        require(runtime_report['status']=='STOP_UNCONFIRMED_POWER_OFF_REQUIRED',
                'Ambiguous/lost native stop must require physical cutoff: '+runtime_report['status'])
        front=runtime_report['stop_reports']['front'];rear=runtime_report['stop_reports']['rear']
        require(set(rear['attempted_ids'])==set(range(7,13)),'Independent rear STOP not attempted')
        require({r['motor_id'] for r in rear['replies']}==set(range(7,13)),'Rear STOP replies missing')
        require(not peers['rear'].enabled,'Rear simulated STOP did not disable all axes')
        # Cancellation may catch the independent rear bus while a Type1 reply is
        # pending. It still sends all six STOPs, but that ID stays ambiguous.
        require(set(rear['unconfirmed_ids'])==set(rear['ambiguous_ids']),
                'Unexpected unresolved rear STOP failure')
        require(1 in front['unconfirmed_ids'],'Failed axis falsely confirmed')
        if case=='drop-feedback':
            require(1 in front['ambiguous_ids'],'Missing Type1 attribution ambiguity lost')
            require(front['attempted_ids']==list(range(1,7)),'Dropped reply prevented other STOP attempts')
            require(all(not peer.enabled for peer in peers.values()),'Simulated STOP failed to clear plant enable')
        else:
            require(front['confirmed_ids']==[],'Disconnected peer falsely confirmed STOP')
        audit.update(failure_injected=case,physical_cutoff_required=True,
                     front_unconfirmed_ids=front['unconfirmed_ids'],rear_confirmed_ids=rear['confirmed_ids'])
    return audit


def run_case(case,lib,out,*,state=None,bundle=None):
    out=Path(out);out.mkdir(mode=0o700)
    profile=synthetic_profile();peers={};hosts=[];devices=[];sessions={};cancel_fds=[]
    report={'status':'INCOMPLETE_OFFLINE_SIMULATION','simulation_only':True,
        'hardware_opened':False,'physical_motor_enable_sent':False,'physical_learned_targets_sent':False,
        'approved_for_runtime':False,'output_allowed':False,'physical_review_generated':False,
        'real_controller_50Hz_verified':False,'jetson_latency_measurement':False,
        'all_new_timestamps_are_simulated':True,'new_robot_samples':0,'case':case,
        'plant_description':'Synthetic direct sign+1 mapping, instantaneous position tracking, no physical dynamics or watchdog emulation',
        'socket_transport':'Exactly two local AF_UNIX socketpairs; no /dev, SSH or network',
        'errors':[],'source_sha256':source_hashes()}
    boot=tempfile.TemporaryFile();boot.write((BOOT+'\n').encode());boot.flush()
    recorded=None
    try:
        if case=='real-success':
            recorded,provenance=learned_policy(profile,state,bundle,out)
            report['real_policy_provenance']=provenance
        else:
            fixed={i:initial_positions()[i]*25.14/65535.-12.57+.04 for i in range(1,13)}
            recorded=RecordedPolicy(lambda sample,imu,now:tuple(fixed[i] for i in range(1,13)),real_model=False)
        imu=SyntheticIMU()
        for scope,ids in SCOPES.items():
            host,device=socket.socketpair(socket.AF_UNIX,socket.SOCK_STREAM)
            hosts.append(host);devices.append(device);host.setblocking(False)
            cr,cw=os.pipe();cancel_fds.extend((cr,cw))
            peer=SocketPlant(device,ids,failure=case if case!='real-success' and scope=='front' else None)
            peers[scope]=peer;peer.thread.start()
            axis=lambda key:{i:profile['axes'][str(i)][key] for i in ids}
            sessions[scope]=native.ActiveSession(lib,host.fileno(),first_id=ids[0],cancel_fd=cr,
                boot_fd=boot.fileno(),boot_id=BOOT,raw_lower_by_id=axis('lower_rad'),raw_upper_by_id=axis('upper_rad'),
                kp_max_by_id=axis('kp'),kd_max_by_id=axis('kd'))
        cancelled=threading.Event()
        def cancel():
            if not cancelled.is_set():
                cancelled.set()
                for fd in cancel_fds[1::2]:os.write(fd,b'x')
        report['runtime']=runtime.run_supported_policy(profile,sessions,imu,recorded,cancel_io=cancel)
        report['simulated_imu_reads']=imu.calls
        report['owner_cancel_invoked']=cancelled.is_set()
        if case=='real-success' and report['runtime']['status']=='COMPLETE_SUPPORTED_OUTPUT':
            report['eager_parity']=eager_parity(recorded,bundle)
    except BaseException as error:
        report['errors'].append(type(error).__name__+': '+str(error))
    finally:
        for session in sessions.values():session.close()
        for host in hosts:
            try:host.shutdown(socket.SHUT_RDWR)
            except OSError:pass
            host.close()
        for peer in peers.values():peer.thread.join(timeout=1.)
        for device in devices:device.close()
        for fd in cancel_fds:os.close(fd)
        boot.close()
        report['all_local_fds_closed']=all(x.fileno()==-1 for x in hosts+devices) and boot.closed
        report['peers']={s:p.evidence() for s,p in peers.items()}
        report['policy_calls']=[] if recorded is None else recorded.calls
        if not report['errors']:
            try:
                report['audit']=audit_case(report,peers,case,recorded)
                report['status']='PASS_OFFLINE_SIMULATION'
            except BaseException as error:report['errors'].append(type(error).__name__+': '+str(error))
        report['total_native_tx_frames']=sum(len(p.rows) for p in peers.values())
        report['total_native_rx_frames']=sum(bool(r['reply_hex']) for p in peers.values() for r in p.rows)
        if report['source_sha256']!=source_hashes():
            report['status']='INCOMPLETE_OFFLINE_SIMULATION'
            report['errors'].append('Source changed during this simulated run')
        write_json(out/'report.json',report)
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--state',type=Path)
    parser.add_argument('--bundle',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',choices=('all','real-success','drop-feedback','usb-disconnect'),default='all')
    args=parser.parse_args(argv)
    if args.case in ('all','real-success') and (args.state is None or args.bundle is None):
        parser.error('Real learned-model smoke requires explicit --state and --bundle')
    out=args.output.absolute()
    require(not out.exists() and out.parent.is_dir(),'Use a new output directory with an existing parent')
    require(not any((p/'.git').exists() for p in (out,*out.parents)),'Raw simulation reports must stay outside Git')
    out.mkdir(mode=0o700)
    if args.case in ('all','real-success'):
        import torch
        torch.set_num_threads(1);torch.set_num_interop_threads(1)
    lib=native.load_library(args.library)
    reports={case:run_case(case,lib,out/case,state=args.state,bundle=args.bundle)
             for case in (('real-success','drop-feedback','usb-disconnect') if args.case=='all' else (args.case,))}
    summary={'simulation_only':True,'hardware_opened':False,'approved_for_runtime':False,
        'output_allowed':False,'physical_review_generated':False,'jetson_latency_measurement':False,
        'library_sha256':digest(args.library),'cases':{case:{'status':r['status'],'errors':r['errors'],
            'runtime_status':r.get('runtime',{}).get('status'),'tx_frames':r['total_native_tx_frames'],
            'rx_frames':r['total_native_rx_frames'],'real_model_calls':len(r['policy_calls']) if case=='real-success' else 0,
            'report_sha256':digest(out/case/'report.json')} for case,r in reports.items()}}
    summary['status']='PASS' if all(r['status']=='PASS_OFFLINE_SIMULATION' for r in reports.values()) else 'FAILED'
    write_json(out/'summary.json',summary);print(json.dumps(summary))
    return 0 if summary['status']=='PASS' else 2


if __name__=='__main__':raise SystemExit(main())
