"""Norm hypotheses use audited immutable raw files, never calibration approval."""
import copy
from dataclasses import FrozenInstanceError, replace
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from singularitydog_hw import imu_accel_input_hypothesis as loader
import test_imu_fixed_mount_baseline as fixtures

TOOLS = Path(__file__).resolve().parents[2]/'tools'
sys.path.insert(0,str(TOOLS))
import analyze_imu_pose_tilt as diagnostic

G = diagnostic.calibration.GRAVITY
BIAS = [.12,-.18,-.61]
SCALE = [1.02,.98,.975]
DIRECTIONS = [[1,.2,.1],[-1,.2,-.1],[.1,1,.2],[-.15,-1,.3],[.1,-.12,1],[-.2,-.05,-1]]
ROTATION = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]


def write(path,data):
    path.write_text(json.dumps(data,allow_nan=False)+'\n')
    return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


class HypothesisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name).resolve()
        cls.manifest = {'schema':diagnostic.INPUT_SCHEMA,'operator_confirmed_stationary':True,'fit':{},'independent':{}}
        for partition_index,partition in enumerate(('fit','independent')):
            for j,(label,v) in enumerate(zip(diagnostic.calibration.FACES,DIRECTIONS)):
                mean = [b+G*x/math.hypot(*v)/s for b,s,x in zip(BIAS,SCALE,v)]
                meta,rows = fixtures.synthetic_capture(20+j+partition_index*10,100+20*(j+6*partition_index))
                meta['plan']['face_label'] = label
                scale = meta['configuration']['accel_m_s2_per_lsb']
                for i,row in enumerate(rows):
                    row['raw_accel'] = [round(x/scale)+(1 if i%2 else -1) for x in mean]
                    row['accel_m_s2'] = [x*scale for x in row['raw_accel']]
                fixtures.refresh_summary(meta,rows)
                directory = cls.root/(partition+label); directory.mkdir()
                summary = write(directory/'summary.json',meta)
                (directory/'events.jsonl').write_text('\n'.join(json.dumps(r) for r in [{'kind':'capture_metadata',**meta['plan']}]+rows)+'\n')
                cls.manifest[partition][label] = {'directory':str(directory),'summary_sha256':summary['sha256'],
                    'events_sha256':hashlib.sha256((directory/'events.jsonl').read_bytes()).hexdigest()}
        cls.manifest_ref = write(cls.root/'manifest.json',cls.manifest)
        cls.report = diagnostic.analyze_file(cls.manifest_ref['path'])
        cls.candidate_ref = write(cls.root/'candidate.json',{**cls.report,'generated_at_utc':'2026-10-06T00:00:00Z'})
        cls.template = loader.create_template(cls.manifest_ref,cls.candidate_ref,ROTATION)

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def setUp(self):
        self.wrapper = copy.deepcopy(self.template)
        self.wrapper.update(raw_norm_bounds_m_s2=[9.7,11.1],corrected_norm_bounds_m_s2=[9.4,10.2],
            assumptions_acknowledged=dict.fromkeys(loader.ASSUMPTIONS,True),
            hypothesis_review={'reviewer':'SYNTHETIC TEST ONLY','reviewed_at':'2026-10-06T12:00:00+09:00',
                'decision':loader.DECISION,'rationale':'Synthetic small-boxed input experiment, no calibration approval.'})
        self.path = self.root/'wrapper.json'

    def load(self,wrapper=None):
        ref = write(self.path,self.wrapper if wrapper is None else wrapper)
        return loader.load_accel_input_hypothesis(ref,ROTATION)

    def test_full_raw_audit_matches_separate_analyzer_and_preserves_unapproved_flags(self):
        loaded = self.load()
        for a,b in zip(loaded.bias_m_s2,BIAS): self.assertAlmostEqual(a,b,delta=.001)
        for a,b in zip(loaded.scale,SCALE): self.assertAlmostEqual(a,b,delta=.0002)
        self.assertEqual(loaded.candidate_sha256,self.candidate_ref['sha256'])
        p = loaded.provenance()
        self.assertEqual(p['hypothesis_sha256'],hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertTrue(p['fit_and_independent_captures_reaudited'])
        self.assertFalse(p['formal_calibration_approved'] or p['grants_motor_output'])
        self.assertIsNone(p['absolute_orientation_error_bound_rad'])
        self.assertEqual(json.loads(Path(self.candidate_ref['path']).read_text())['approved_for_runtime'],False)

    def test_template_is_unreviewed_no_file_write_and_rejects_use(self):
        before = {p:p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        template = loader.create_template(self.manifest_ref,self.candidate_ref,ROTATION)
        self.assertEqual(template,self.template)
        self.assertEqual(before,{p:p.read_bytes() for p in before})
        with self.assertRaisesRegex(ValueError,'acknowledged|review'): self.load(template)

    def test_correction_is_non_normalizing_and_raw_input_unchanged(self):
        loaded = self.load()
        raw = [b+x/s for b,s,x in zip(loaded.bias_m_s2,loaded.scale,[0.,0.,-G])]
        original = raw[:]
        corrected,norm = loaded.correct(raw)
        self.assertAlmostEqual(norm,G)
        self.assertEqual(raw,original)
        raw[2] -= .1/loaded.scale[2]
        self.assertAlmostEqual(loaded.correct(raw)[1],G+.1)

    def test_raw_and_corrected_norm_guards_are_both_required(self):
        loaded = self.load()
        with self.assertRaisesRegex(ValueError,'Raw acceleration'): loaded.correct([0.,0.,9.3])
        with self.assertRaisesRegex(ValueError,'Corrected acceleration'): loaded.correct([0.,0.,10.4])

    def test_triplets_finiteness_types_and_frozen_provenance(self):
        loaded = self.load()
        for values in ([0.,0.], [True,0.,-10.5], [0.,0.,math.inf], {'accel':[0.,0.,-10.5]}):
            with self.subTest(values=values),self.assertRaises(ValueError): loaded.correct(values)
        with self.assertRaises(FrozenInstanceError): loaded.scale = (1.,1.,1.)
        p = loaded.provenance(); p['scale_sensor'][0] = 999; p['source_sha256'].clear()
        self.assertNotEqual(p,loaded.provenance())
        forged = replace(loaded,_proof=None)
        with self.assertRaisesRegex(ValueError,'proof'): forged.correct([0.,0.,-10.5])

    def test_formal_approval_unknown_error_scope_and_extra_keys_reject(self):
        for key,value in (('formal_calibration_approved',True),('grants_motor_output',1),
                          ('absolute_orientation_error_bound_rad',0.),('scope','ground'),('schema','reviewed'),('extra',False)):
            changed = copy.deepcopy(self.wrapper); changed[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError): self.load(changed)

    def test_named_review_timestamp_and_assumptions_required(self):
        variants=[]
        for key,value in (('reviewer',' '),('reviewed_at','2026-10-06T12:00:00'),('decision','UNREVIEWED')):
            changed=copy.deepcopy(self.wrapper); changed['hypothesis_review'][key]=value; variants.append(changed)
        for key in loader.ASSUMPTIONS:
            changed=copy.deepcopy(self.wrapper); changed['assumptions_acknowledged'][key]=1; variants.append(changed)
        for changed in variants:
            with self.assertRaises(ValueError): self.load(changed)

    def test_mount_must_be_same_proper_rotation_with_real_entries(self):
        for matrix in ([[True,0,0],[0,1,0],[0,0,1]], [[-1,0,0],[0,1,0],[0,0,1]],
                       [[1,0,0],[0,1,0],[0,0,2]], [[0,-1,0],[1,0,0],[0,0,1]]):
            changed=copy.deepcopy(self.wrapper); changed['R_body_from_sensor']=matrix
            with self.subTest(matrix=matrix),self.assertRaises(ValueError): self.load(changed)

    def test_bounds_invalid_or_excluding_independent_samples_rejected(self):
        for key,value in (('raw_norm_bounds_m_s2',[8.8,11.2]),('corrected_norm_bounds_m_s2',[9.80664,9.80666]),
                          ('corrected_norm_bounds_m_s2',[True,10.2]),('corrected_norm_bounds_m_s2',None)):
            changed=copy.deepcopy(self.wrapper); changed[key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError): self.load(changed)

    def test_wrapper_source_graph_and_sha_tampering_rejected(self):
        changed=copy.deepcopy(self.wrapper); changed['source_sha256']['singularitydog_hw/imu.py']='0'*64
        with self.assertRaisesRegex(ValueError,'runtime sources'): self.load(changed)
        ref=write(self.path,self.wrapper); self.path.write_bytes(self.path.read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError,'Hypothesis SHA'): loader.load_accel_input_hypothesis(ref,ROTATION)

    def test_report_forgery_with_updated_sha_fails_recomputation(self):
        changes=(('heldout_used_for_fit',0),('approved_for_runtime',True),('physical_pose_uncertainty_rad',0.),
                 ('accel_diagnostic_candidate',{**self.report['accel_diagnostic_candidate'],'scale':[1.,1.,1.]}),
                 ('evaluation',{}))
        for key,value in changes:
            changed=copy.deepcopy(self.wrapper)
            changed['candidate']=write(self.root/'forged-report.json',{**self.report,key:value})
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'recomputation'): self.load(changed)

    def test_diagnostic_source_code_pin_cannot_be_replaced_even_with_claimed_sha(self):
        changed=copy.deepcopy(self.report)
        source=self.root/'analyze_imu_pose_tilt.py'; source.write_text('raise RuntimeError("MUST NOT EXECUTE")\n')
        changed['source_bindings'][0]={'path':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'byte_count':source.stat().st_size}
        wrapper=copy.deepcopy(self.wrapper); wrapper['candidate']=write(self.root/'forged-source.json',changed)
        with self.assertRaisesRegex(ValueError,'implementation source'): self.load(wrapper)

    def test_manifest_and_capture_bytes_changes_rejected(self):
        changed=copy.deepcopy(self.wrapper); changed['manifest']['sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'Manifest SHA'): self.load(changed)
        path=Path(self.manifest['independent']['z-']['directory'])/'events.jsonl'; raw=path.read_bytes()
        try:
            path.write_bytes(raw+b'\n')
            with self.assertRaisesRegex(ValueError,'Capture input SHA'): self.load()
        finally: path.write_bytes(raw)

    def test_concurrent_input_mutation_after_computation_rejected(self):
        original=loader.analyze_datasets
        path=Path(self.candidate_ref['path']); raw=path.read_bytes()
        def mutate(*args):
            result=original(*args); path.write_bytes(raw+b' '); return result
        try:
            with patch.object(loader,'analyze_datasets',side_effect=mutate),self.assertRaisesRegex(ValueError,'changed during'):
                self.load()
        finally: path.write_bytes(raw)

    def test_symlink_and_nonregular_files_rejected(self):
        ref=write(self.path,self.wrapper)
        link=self.root/'linked-wrapper.json'; link.symlink_to(self.path)
        try:
            with self.assertRaisesRegex(ValueError,'symlink'): loader.load_accel_input_hypothesis({**ref,'path':str(link)},ROTATION)
        finally: link.unlink()
        with self.assertRaises((ValueError,IsADirectoryError)):
            loader.load_accel_input_hypothesis({'path':str(self.root),'sha256':'0'*64},ROTATION)

    def test_duplicate_json_and_boolean_type_alias_rejected(self):
        raw=b'{"schema":1,"schema":2}'
        self.path.write_bytes(raw)
        with self.assertRaisesRegex(ValueError,'Duplicate'):
            loader.load_accel_input_hypothesis({'path':str(self.path),'sha256':hashlib.sha256(raw).hexdigest()},ROTATION)
        changed=copy.deepcopy(self.wrapper); changed['formal_calibration_approved']=0
        with self.assertRaisesRegex(ValueError,'certify'): self.load(changed)


if __name__ == '__main__': unittest.main()
