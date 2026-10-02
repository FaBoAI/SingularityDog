"""Pure/file-only branch tests. No model, serial device, motor or network."""
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import statistics
import tempfile
import unittest
from unittest.mock import patch

try:
    from singularitydog_hw import diagnostic_angle_branch as tool
except ImportError:
    spec=importlib.util.spec_from_file_location('diagnostic_angle_branch',
        Path(__file__).with_name('diagnostic_capture_branch_derivation_20261002.py'))
    tool=importlib.util.module_from_spec(spec); spec.loader.exec_module(tool)

BOOT='00000000-0000-4000-8000-000000000002'
EPOCH='operator-confirmed-human-full-support-SYNTHETIC-r10'
PIN=lambda name:dict(path='/SYNTHETIC/'+name,sha256='a'*64)


def fixtures(turns=None,prior=None):
    turns=turns or {}; prior=prior or {}
    uids={str(i):f'{i:016x}' for i in range(1,13)}
    bounds={i:(lo,hi) for i,lo,hi in zip(tool.shadow.CAN_ORDER,tool.shadow.LOWER,tool.shadow.UPPER)}
    base=dict(status='MANUAL_NOMINAL_CANDIDATES_ONLY',
        formula='q_model = sign * raw + offset; rad; no wrapping',
        model_can_order_candidate=tool.shadow.CAN_ORDER[:],identities=copy.deepcopy(uids),
        approved_for_runtime=False,motor_output_allowed=False,
        absolute_calibration_error_rad=None,feedback_scale_verified=False,
        candidates=[])
    capture=dict(schema='singularitydog.readonly-12-angle-capture.v1',status='RECORDED_REVIEW_REQUIRED',
        errors=[],boot_id=BOOT,approved_for_runtime=False,motor_output_allowed=False,
        angle_wrap_applied=False,stop_state='UNVERIFIED_BY_READ_ONLY_PROTOCOL',
        motor_power_epoch='NOT_INFERRED_FROM_JETSON_BOOT',expected_uids_sha256='a'*64,
        identities={},telemetry={'rows':{}})
    for i in range(1,13):
        key=str(i); sign=1 if i%2 else -1; lo,hi=bounds[i]; q=(lo+hi)/2
        base['candidates'].append(dict(motor_id=i,sign_candidate=sign,
            offset_candidate_rad=-sign*prior.get(i,0)*2*math.pi,
            diagnostic_branch_turns_embedded_in_offset=prior.get(i,0),
            physical_angle_accuracy_verified=False,sign_revalidated_for_runtime=False,
            approved_for_runtime=False))
        raw=sign*q+turns.get(i,0)*2*math.pi; offset=i*1000
        capture['identities'][key]=dict(mcu_uid_hex=uids[key],request_monotonic_ns=100+offset,
            reply_monotonic_ns=110+offset)
        samples=[dict(rad=raw+d,request_monotonic_ns=120+offset+j*10,
            reply_monotonic_ns=125+offset+j*10) for j,d in enumerate((-1e-5,0.,1e-5))]
        capture['telemetry']['rows'][key]=dict(run_mode=0,current=0.,voltage=39.,
            position_samples=samples,median_position_rad=statistics.median(s['rad'] for s in samples),
            position_span_deg=math.degrees(samples[-1]['rad']-samples[0]['rad']))
    return base,capture,uids


def derive(base,capture,uids):
    return tool.derive_document(base,capture,uids,base_pin=PIN('base'),capture_pin=PIN('capture'),
        uid_pin=PIN('uids'),boot_id=BOOT,power_epoch=EPOCH)


class BranchTests(unittest.TestCase):
    def test_each_axis_plus_minus_full_turn_recovers_same_angle_without_mutating_inputs(self):
        for i in range(1,13):
            for turn in (-1,0,1):
                with self.subTest(axis=i,turn=turn):
                    base,cap,uids=fixtures({i:turn}); before=copy.deepcopy((base,cap,uids))
                    result=derive(base,cap,uids); row=result['candidates'][i-1]
                    self.assertEqual(row['diagnostic_branch_turns_embedded_in_offset'],turn)
                    self.assertEqual(row['sign_candidate'],base['candidates'][i-1]['sign_candidate'])
                    index=tool.shadow.CAN_ORDER.index(i)
                    self.assertAlmostEqual(result['model_rad_at_source_capture_by_id'][str(i)],
                        (tool.shadow.LOWER[index]+tool.shadow.UPPER[index])/2)
                    self.assertEqual((base,cap,uids),before)

    def test_previous_embedded_turn_is_undone_before_reselection_no_double_correction(self):
        base,cap,uids=fixtures({1:1,2:-1,3:0},{1:-1,2:1,3:1})
        out=derive(base,cap,uids)
        self.assertEqual([out['candidates'][i-1]['diagnostic_branch_turns_embedded_in_offset']
            for i in (1,2,3)],[1,-1,0])
        for mid in (1,2,3):
            a=out['diagnostic_branch_derivation']['axes'][str(mid)]
            self.assertAlmostEqual(a['nominal_zero_offset_rad'],0.)
        again=derive(out,cap,uids)
        self.assertEqual(again['candidates'],out['candidates'])

    def test_all_unknown_and_no_output_claims_stay_unapproved_with_current_receipts(self):
        base,cap,uids=fixtures({1:1,3:1,11:1}); out=derive(base,cap,uids)
        for k in ('output_allowed','motor_output_allowed','approved_for_runtime','calibration_verified',
                  'physical_joint_limits_verified','raw_angles_modified','epoch_binding_created',
                  'motor_supply_off_on_evidence_complete','cross_boot_angle_continuity_verified'):
            self.assertIs(out[k],False)
        self.assertIsNone(out['absolute_calibration_error_rad']); self.assertFalse(out['feedback_scale_verified'])
        self.assertEqual(out['motor_power_epoch'],EPOCH)
        self.assertEqual(out['source_capture_power_epoch_label_preserved'],'NOT_INFERRED_FROM_JETSON_BOOT')
        proof=out['diagnostic_branch_derivation']; self.assertEqual(proof['fresh_capture'],PIN('capture'))
        self.assertFalse(proof['physical_statement_verified_by_helper'])
        self.assertTrue(proof['model_intervals_unchanged'])

    def test_no_unique_branch_reports_axis_raw_calibration_and_limits_without_clipping(self):
        base,cap,uids=fixtures(); i=1; key=str(i)
        index=tool.shadow.CAN_ORDER.index(i); q=tool.shadow.LOWER[index]-.01
        row=cap['telemetry']['rows'][key]; s=base['candidates'][i-1]['sign_candidate']
        for p in row['position_samples']: p['rad']=s*q
        row['median_position_rad']=s*q; row['position_span_deg']=0.
        before=copy.deepcopy((base,cap))
        with self.assertRaisesRegex(ValueError,'ID1.*raw=.*base_offset=.*q_unadjusted=.*model_limits='):
            derive(base,cap,uids)
        self.assertEqual((base,cap),before)

    def test_intervals_never_broadened_and_one_turn_wide_range_rejected(self):
        base,cap,uids=fixtures()
        with patch.object(tool.shadow,'UPPER',[x+2*math.pi for x in tool.shadow.UPPER]):
            with self.assertRaisesRegex(ValueError,'narrower than one turn'): derive(base,cap,uids)

    def test_identity_boot_power_status_or_approval_failure_rejected(self):
        for kind in ('uid','boot','power','errors','wrapped','enable','approved','base-approved','base-sign'):
            base,cap,uids=fixtures()
            if kind=='uid': cap['identities']['1']['mcu_uid_hex']='f'*16
            elif kind=='boot': cap['boot_id']='11111111-1111-1111-1111-111111111111'
            elif kind=='power': cap['motor_power_epoch']='old epoch'
            elif kind=='errors': cap['errors']=['read incomplete']
            elif kind=='wrapped': cap['angle_wrap_applied']=True
            elif kind=='enable': cap['motor_output_allowed']=True
            elif kind=='approved': cap['approved_for_runtime']=True
            elif kind=='base-approved': base['approved_for_runtime']=True
            else: base['candidates'][0]['sign_candidate']=True
            with self.subTest(kind=kind),self.assertRaises(ValueError): derive(base,cap,uids)

    def test_nonquiet_unstable_nonfinite_bool_or_noncause_samples_rejected(self):
        for kind in ('mode','current','bool-mode','bool-position','bool-median','span','median','nan','causal','late'):
            base,cap,uids=fixtures(); row=cap['telemetry']['rows']['1']
            if kind=='mode': row['run_mode']=1
            elif kind=='current': row['current']=.1
            elif kind=='bool-mode': row['run_mode']=False
            elif kind=='bool-position': row['position_samples'][0]['rad']=True
            elif kind=='bool-median': row['median_position_rad']=True
            elif kind=='span': row['position_span_deg']=1.
            elif kind=='median': row['median_position_rad']+=.01
            elif kind=='nan': row['position_samples'][0]['rad']=float('nan')
            elif kind=='causal': row['position_samples'][0]['request_monotonic_ns']=1
            else: row['position_samples'][0]['reply_monotonic_ns']+=31_000_000
            with self.subTest(kind=kind),self.assertRaises(ValueError): derive(base,cap,uids)

    def test_invalid_embedded_turn_rejected(self):
        for value in (True,1.5,21,'1'):
            base,cap,uids=fixtures(); base['candidates'][0]['diagnostic_branch_turns_embedded_in_offset']=value
            with self.subTest(value=value),self.assertRaises(ValueError): derive(base,cap,uids)

    def test_file_hash_private_output_and_nonoverwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp).resolve(); base,cap,uids=fixtures({1:1,3:1,11:1})
            params=dict(boot_id=BOOT,power_epoch=EPOCH,output_path=str(directory/'derived.json'))
            for name,data in [('base_calibration',base),('capture',cap),('expected_uids',uids)]:
                if name=='expected_uids': raw=json.dumps(data).encode(); cap['expected_uids_sha256']=hashlib.sha256(raw).hexdigest()
            for name,data in [('base_calibration',base),('capture',cap),('expected_uids',uids)]:
                path=directory/(name+'.json'); raw=json.dumps(data).encode(); path.write_bytes(raw)
                params[name+'_path']=str(path); params[name+'_sha256']=hashlib.sha256(raw).hexdigest()
            out=tool.derive_diagnostic_calibration(**params); path=Path(out['path']); saved=path.read_bytes()
            self.assertEqual(out['sha256'],hashlib.sha256(saved).hexdigest())
            self.assertEqual(path.stat().st_mode&0o777,0o600)
            self.assertEqual({k:v for k,v in out['turns_by_id'].items() if v}, {'1':1,'3':1,'11':1})
            with self.assertRaises(ValueError): tool.derive_diagnostic_calibration(**params)
            self.assertEqual(path.read_bytes(),saved)
            params['output_path']=str(directory/'second.json')
            Path(params['capture_path']).write_bytes(b'{}')
            with self.assertRaisesRegex(ValueError,'changed'): tool.derive_diagnostic_calibration(**params)
            self.assertFalse((directory/'second.json').exists())

    def test_duplicate_json_key_nonfinite_and_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp).resolve()
            for raw in (b'{"x":1,"x":2}',b'{"x":NaN}'):
                path=directory/'bad.json'; path.write_bytes(raw)
                with self.assertRaises(ValueError): tool.read_pinned(path,hashlib.sha256(raw).hexdigest())
            link=directory/'link.json'; link.symlink_to(path)
            with self.assertRaises(ValueError): tool.read_pinned(link,hashlib.sha256(raw).hexdigest())


if __name__=='__main__': unittest.main()
