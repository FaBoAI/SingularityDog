"""Synthetic file-only profile assembly; fixtures are never hardware evidence."""
import copy
import hashlib
import io
import json
from contextlib import redirect_stdout, redirect_stderr
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'runtime'),str(ROOT/'runtime/tests')]
import prepare_supported_profile as prep
from test_policy_live_profile import synthetic_fixture


class PreparationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.base=Path(self.tmp.name)
        self.original,self.docs,pins=synthetic_fixture(self.base)
        self.enterContext(patch.object(prep.live.shadow,'SOURCE_HASHES',pins))
        self.ap={'schema':prep.angles.PROFILE_SCHEMA,'assembly_revision':'SYNTHETIC ONLY',
            'approved_for_runtime':False,'motor_output_available':False,'physical_uncertainty_known':False,
            'evidence_files':{},'axes':[]}
        self.capture={'status':'RECORDED_REVIEW_REQUIRED','errors':[], 'motor_output_allowed':False,
            'angle_wrap_applied':False,'plan':{'allowed_can_types':[0,17]},
            'boot_id':self.original['boot_id'],'motor_power_epoch':'SYNTHETIC POWER EPOCH',
            'identities':{},'telemetry':{'rows':{}}}
        for mid,row in self.original['axes'].items():
            self.ap['axes'].append(dict(motor_id=int(mid),uid=row['uid'],sign=1,offset_rad=0.,
                lower_rad=row['physical_lower_rad'],upper_rad=row['physical_upper_rad'],uncertainty_rad=0.,
                zero_reviewed=False,direction_reviewed=False,physical_limits_reviewed=False,
                zero_evidence_sha256=None,direction_evidence_sha256=None,physical_limits_evidence_sha256=None))
            self.capture['identities'][mid]={'mcu_uid_hex':row['uid']}
            self.capture['telemetry']['rows'][mid]={'run_mode':0,'current':0,'position_span_deg':.01,
                'median_position_rad':-1. if (int(mid)-1)%3==0 else 0.}
        self.cal=self.docs['calibration']
        self.cal.update(source_current_boot_id=self.original['boot_id'],
            source_current_motor_power_epoch_label=self.capture['motor_power_epoch'])
        for row in self.cal['candidates']:row['diagnostic_branch_turns_embedded_in_offset']=0
        self.diagnostic=self.docs['pipeline_diagnostic']
        self.diagnostic.update(boot_id=self.original['boot_id'],motor_power_epoch=self.capture['motor_power_epoch'])
        self.refresh()

    def write(self,name,data):
        path=self.base/name;path.write_bytes(prep._json_bytes(data));return path

    def refresh(self):
        self.write('angle-profile.json',self.ap);self.write('capture.json',self.capture)
        self.cal['source_capture_sha256']=hashlib.sha256((self.base/'capture.json').read_bytes()).hexdigest()
        self.cal['source_raw_rad_by_id']={mid:row['median_position_rad'] for mid,row in self.capture['telemetry']['rows'].items()}
        self.cal['source_calibration_sha256_by_id']={mid:hashlib.sha256((self.base/'angle-profile.json').read_bytes()).hexdigest() for mid in prep.live.IDS}
        self.write('calibration.json',self.cal)
        self.diagnostic['input_sha256']['calibration']=hashlib.sha256((self.base/'calibration.json').read_bytes()).hexdigest()
        self.write('pipeline_diagnostic.json',self.diagnostic)

    def args(self):
        return dict(calibration=self.base/'calibration.json',angle_profile=self.base/'angle-profile.json',
            mount=self.base/'mount.json',bias=self.base/'bias.json',model_manifest=self.base/'model_manifest.json',
            pipeline_diagnostic=self.base/'pipeline_diagnostic.json',assembly_id='SYNTHETIC ASSEMBLY ONLY',
            output=self.base/'assembled',source_capture=self.base/'capture.json',bundle=self.base/'bundle')

    def run_prepare(self,**extra):
        args=self.args();args.update(extra);return prep.prepare(**args)

    def result(self,name):return json.loads((self.base/'assembled'/name).read_text())

    def fast_args(self, *, overlap=True, pipeline=True, encoder=True):
        """Fake files only; no shared library or policy is loaded."""
        scalar={'schema':'native-step-scalar-file-only-v1','status':'PASS_FILE_ONLY_COMPARE',
            'baseline_manifest_sha256':hashlib.sha256((self.base/'model_manifest.json').read_bytes()).hexdigest(),
            **dict.fromkeys(('hardware_opened','output_allowed','approved_for_runtime','live_50hz_verified'),False)}
        scalar_path=self.write('scalar.json',scalar)
        settings=prep.settings_template()
        settings['run_settings'].update(model_backend=prep.live.SCALAR_BACKEND,
            voltage_overlap=overlap,voltage_pipeline=pipeline,policy_weight=1.,duration_s=3.)
        for group,kp in [('calf',4.),('thigh',8.),('hip',6.)]:settings['groups'][group]['kp']=kp
        if encoder:
            binary=self.base/'bundle'/'synthetic_batch.so'
            binary.write_bytes(b'SYNTHETIC ONLY: never dlopen')
            settings['run_settings']['native_batch_encoder']={'path':binary.name,
                'sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
        self.diagnostic['model_source']={'manifest_sha256':hashlib.sha256(scalar_path.read_bytes()).hexdigest(),
            'baseline_provenance':{'manifest_sha256':scalar['baseline_manifest_sha256']}}
        self.diagnostic['plan'].update(v3_voltage_overlap=overlap,v3_voltage_validation_overlap=overlap,
            v3_voltage_fast_pipeline=pipeline)
        self.diagnostic['cadence_source_sha256']=prep.live.cadence_source_hashes()
        self.refresh()
        return {'profile_schema':prep.live.SCHEMA_V3,'scalar_step_manifest':scalar_path,
                'group_settings':self.write('fast-settings.json',settings)}

    def test_fast_formal_cli_pins_exact_backend_and_binary_without_promoting_review(self):
        args=self.args();args.update(self.fast_args())
        originals={path:path.read_bytes() for path in self.base.rglob('*') if path.is_file()}
        argv=[]
        for key,value in args.items():argv.extend(('--'+key.replace('_','-'),str(value)))
        with redirect_stdout(io.StringIO()) as stdout:
            self.assertEqual(prep.main(argv),0)
        result=json.loads(stdout.getvalue());profile=self.result('profile.json')
        hardware=self.result('hardware-review.json');prepared=self.result('preparation.json')
        self.assertEqual(profile['model_backend'],prep.live.SCALAR_BACKEND)
        self.assertTrue(profile['voltage_overlap']);self.assertTrue(profile['voltage_pipeline'])
        self.assertEqual(profile['policy_weight'],1.)
        self.assertFalse(result['output_allowed']);self.assertFalse(result['hardware_opened'])
        self.assertEqual(result['execution_settings'],prep.live.execution_settings(profile))
        self.assertEqual(prepared['execution_settings'],result['execution_settings'])
        self.assertNotIn('local_characterization',profile);self.assertNotIn('watchdog_review_policy',profile)
        self.assertFalse(profile['approved_for_supported_policy_output']);self.assertIsNone(profile['review'])
        for group,ids in prep.GROUPS.items():
            for mid in ids:self.assertEqual(profile['axes'][str(mid)]['kp'],{'calf':4.,'thigh':8.,'hip':6.}[group])
        self.assertEqual(hardware['artifact_sha256'],{key:ref['sha256']
            for key,ref in profile['artifacts'].items() if key!='hardware_review'})
        self.assertEqual(hardware['artifact_sha256']['scalar_step_manifest'],
            hashlib.sha256((self.base/'scalar.json').read_bytes()).hexdigest())
        self.assertEqual(hardware['reviewed_settings_sha256'],prep.live.reviewed_settings_sha256(profile))
        self.assertIsNone(hardware['native_batch_encoder_acceptance']['review'])
        self.assertIsNone(hardware['voltage_pipeline_acceptance']['review'])
        self.assertIsNone(hardware['native_batch_encoder_acceptance']['hard_output_and_freshness_limits_unchanged'])
        self.assertEqual(hardware['native_batch_encoder_acceptance']['binary_sha256'],profile['native_batch_encoder']['sha256'])
        self.assertEqual(hardware['voltage_pipeline_acceptance']['diagnostic_sha256'],
                         profile['artifacts']['pipeline_diagnostic']['sha256'])
        for path,raw in originals.items():self.assertEqual(path.read_bytes(),raw)
        plan=prep.live.load_profile(self.base/'assembled/profile.json',require_approved=False)
        self.assertFalse(plan['output_allowed'])
        with self.assertRaisesRegex(ValueError,'unapproved'):
            prep.live.load_profile(self.base/'assembled/profile.json')

    def test_formal_ten_second_fast_candidate_uses_scalar_timing_without_short_pipeline_cap(self):
        args=self.fast_args(pipeline=False)
        settings=json.loads(args['group_settings'].read_text());settings['run_settings']['duration_s']=10.
        self.write('fast-settings.json',settings)
        result=self.run_prepare(**args)
        timing=self.result('preparation.json')['timing_diagnostic_review_only']
        profile=self.result('profile.json')
        self.assertEqual(profile['duration_s'],10.);self.assertEqual(profile['policy_weight'],1.)
        self.assertTrue(profile['voltage_overlap']);self.assertFalse(profile['voltage_pipeline'])
        self.assertIsNotNone(profile['native_batch_encoder'])
        self.assertNotIn('voltage_pipeline_acceptance',self.result('hardware-review.json'))
        self.assertEqual(timing['kind'],'stop_proxy_diagnostic_only')
        self.assertFalse(timing['actual_policy_output_20ms_verified']);self.assertFalse(result['output_allowed'])

    def test_scalar_selection_and_manifest_must_both_be_explicit_v3_inputs(self):
        args=self.fast_args()
        with self.assertRaisesRegex(ValueError,'Explicit scalar-step manifest'):
            self.run_prepare(**dict(args,scalar_step_manifest=None))
        with self.assertRaisesRegex(ValueError,'V3 profile'):
            self.run_prepare(**dict(args,profile_schema=prep.live.SCHEMA_V2))
        with self.assertRaisesRegex(ValueError,'explicit scalar backend'):
            self.run_prepare(scalar_step_manifest=args['scalar_step_manifest'])
        self.assertFalse((self.base/'assembled').exists())

    def test_scalar_manifest_flags_schema_status_and_exact_baseline_are_required(self):
        args=self.fast_args();original=json.loads(args['scalar_step_manifest'].read_text())
        changes=[{'schema':'other'},{'status':'UNVERIFIED'},{'baseline_manifest_sha256':'f'*64}]
        changes.extend({key:value} for key in ('hardware_opened','output_allowed','approved_for_runtime','live_50hz_verified')
                       for value in (True,0,None))
        for change in changes:
            self.write('scalar.json',dict(original,**change))
            with self.subTest(change=change),self.assertRaisesRegex(ValueError,'file-only manifest'):
                self.run_prepare(**args)
        self.assertFalse((self.base/'assembled').exists())

    def test_scalar_diagnostic_conflicting_model_and_baseline_pins_are_rejected(self):
        args=self.fast_args();original=copy.deepcopy(self.diagnostic)
        for kind in ('selected','baseline'):
            self.diagnostic=copy.deepcopy(original)
            if kind=='selected':self.diagnostic['model_source']['manifest_sha256']='e'*64
            else:self.diagnostic['model_source']['baseline_provenance']['manifest_sha256']='e'*64
            self.write('pipeline_diagnostic.json',self.diagnostic)
            with self.subTest(kind=kind),self.assertRaisesRegex(ValueError,'hash mismatch'):
                self.run_prepare(**args)
        self.assertFalse((self.base/'assembled').exists())

    def test_native_encoder_selection_requires_exact_regular_bundle_member(self):
        args=self.fast_args();settings=json.loads(args['group_settings'].read_text())
        original=copy.deepcopy(settings['run_settings']['native_batch_encoder'])
        cases=[({'path':'../escape.so','sha256':original['sha256']},'bundle-relative'),
               ({'path':'missing.so','sha256':original['sha256']},'binary mismatch'),
               (dict(original,sha256='a'*64),'binary mismatch')]
        link=self.base/'bundle'/'alias.so';link.symlink_to(self.base/'bundle'/original['path'])
        cases.append((dict(original,path=link.name),'binary mismatch'))
        for selection,message in cases:
            settings['run_settings']['native_batch_encoder']=selection;self.write('fast-settings.json',settings)
            with self.subTest(selection=selection),self.assertRaisesRegex(ValueError,message):self.run_prepare(**args)
        settings['run_settings']['native_batch_encoder']=original;self.write('fast-settings.json',settings)
        with self.assertRaisesRegex(ValueError,'explicit pinned bundle'):self.run_prepare(**dict(args,bundle=None))
        self.assertFalse((self.base/'assembled').exists())

    def test_fast_settings_cannot_copy_supported_only_waivers_or_new_stage_scope(self):
        args=self.fast_args();original=json.loads(args['group_settings'].read_text())
        for key,value in [('local_characterization',prep.live.LOCAL_RELATIVE_SUPPORTED),
                          ('watchdog_review_policy',prep.live.COMMAND_LOSS_ONLY_SUPPORTED),
                          ('diagnostic_timing_acceptance',prep.live.SUPPORTED_POLICY_MIX_STEP_10PCT),
                          ('post_reply_deadline_policy',{}),('fixed_catch',{}),('human_supported_hold',{}),
                          ('cadence_source_sha256',{})]:
            settings=copy.deepcopy(original);settings['run_settings'][key]=value;self.write('fast-settings.json',settings)
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'Invalid three-group settings'):
                self.run_prepare(**args)
        self.assertFalse((self.base/'assembled').exists())

    def test_fast_manifest_and_encoder_mutation_before_publication_are_rejected(self):
        args=self.fast_args();serialize=prep._json_bytes
        for path,message in [(self.base/'scalar.json','Source changed during assembly'),
                             (self.base/'bundle'/'synthetic_batch.so','Model bundle source changed')]:
            original=path.read_bytes()
            def mutate(value):
                if value.get('schema')==prep.live.REVIEW_SCHEMA:path.write_bytes(b'SYNTHETIC MUTATION')
                return serialize(value)
            with self.subTest(path=path.name),patch.object(prep,'_json_bytes',side_effect=mutate):
                with self.assertRaisesRegex(ValueError,message):self.run_prepare(**args)
            path.write_bytes(original)
        self.assertFalse((self.base/'assembled').exists())

    def test_unapproved_profile_pins_sources_and_leaves_unknowns_null(self):
        sources={p:p.read_bytes() for p in self.base.rglob('*') if p.is_file()}
        result=self.run_prepare();profile=self.result('profile.json');review=self.result('hardware-review.json')
        self.assertFalse(result['output_allowed']);self.assertFalse(result['hardware_opened'])
        self.assertIsNone(profile['review']);self.assertFalse(profile['approved_for_supported_policy_output'])
        self.assertEqual(profile['schema'],prep.live.SCHEMA_V2)
        self.assertEqual((profile['request_gap_us'],profile['request_window']),(600,3))
        self.assertEqual(result['transport_settings'],prep.live.transport_settings(profile))
        for mid,row in profile['axes'].items():
            self.assertEqual(row['sign'],1);self.assertEqual(row['offset_rad'],0)
            for field in ('uncertainty_rad','physical_lower_rad','physical_upper_rad',*prep.live.LIMIT_CAPS):
                self.assertIsNone(row[field])
            self.assertIsNone(review['angles'][mid]['zero_and_sign_physically_verified'])
            self.assertIsNone(review['type2_dynamic'][mid]['velocity_scale_and_sign_verified'])
            self.assertIsNone(review['device_watchdog'][mid]['actual_command_loss_test_passed'])
            self.assertIn('version_bytes_hex',review['device_watchdog'][mid])
            self.assertIsNone(review['device_watchdog'][mid]['version_bytes_hex'])
        self.assertEqual(profile['boot_id'],self.original['boot_id'])
        self.assertEqual(profile['motor_power_epoch'],self.capture['motor_power_epoch'])
        for key,ref in profile['artifacts'].items():
            self.assertEqual(ref['sha256'],hashlib.sha256(Path(ref['path']).read_bytes()).hexdigest())
        self.assertEqual(review['reviewed_settings_sha256'],prep.live.reviewed_settings_sha256(profile))
        for path,raw in sources.items():self.assertEqual(path.read_bytes(),raw)
        self.assertEqual((self.base/'assembled').stat().st_mode & 0o777,0o700)
        for path in (self.base/'assembled').iterdir():self.assertEqual(path.stat().st_mode & 0o777,0o600)
        self.assertFalse(prep.live.load_profile(self.base/'assembled/profile.json',require_approved=False)['output_allowed'])
        with self.assertRaisesRegex(ValueError,'unapproved'):
            prep.live.load_profile(self.base/'assembled/profile.json')

    def test_explicit_v3_cadence_is_source_pinned_but_remains_unapproved(self):
        result=self.run_prepare(profile_schema=prep.live.SCHEMA_V3)
        profile=self.result('profile.json');preparation=self.result('preparation.json')
        self.assertEqual(profile['schema'],prep.live.SCHEMA_V3)
        self.assertEqual(profile['cadence_source_sha256'],prep.live.cadence_source_hashes())
        self.assertEqual(preparation['telemetry_cadence']['total_requests_per_cycle_including_output'],26)
        self.assertFalse(preparation['telemetry_cadence']['timeout_parameter_drift_monitored_during_cycles'])
        self.assertFalse(result['output_allowed']);self.assertFalse(result['hardware_opened'])
        self.assertIsNone(profile['review'])
        self.assertEqual(self.result('hardware-review.json')['reviewed_settings_sha256'],
                         prep.live.reviewed_settings_sha256(profile))
        with self.assertRaisesRegex(ValueError,'unapproved'):
            prep.live.load_profile(self.base/'assembled/profile.json')

    def test_v3_diagnostic_without_source_pins_is_not_current_source_evidence(self):
        self.run_prepare(profile_schema=prep.live.SCHEMA_V3)
        preparation=self.result('preparation.json')
        self.assertFalse(preparation['diagnostic_binding']['cadence_sources_match'])
        self.assertEqual(preparation['timing_diagnostic_review_only']['status'],'NOT_ELIGIBLE')
        self.assertIn('diagnostic_cadence_sources_unavailable_or_inconsistent',
            [row['code'] for row in preparation['blockers_by_kind']['missing_measurements']])
        self.assertFalse(self.result('profile.json')['approved_for_supported_policy_output'])

    def test_v3_stale_source_diagnostic_does_not_inherit_template_source_pins(self):
        self.diagnostic['cadence_source_sha256']=prep.live.cadence_source_hashes()
        first=next(iter(self.diagnostic['cadence_source_sha256']))
        self.diagnostic['cadence_source_sha256'][first]='a'*64
        self.refresh();self.run_prepare(profile_schema=prep.live.SCHEMA_V3)
        preparation=self.result('preparation.json')
        self.assertFalse(preparation['diagnostic_binding']['cadence_sources_match'])
        self.assertEqual(preparation['timing_diagnostic_review_only']['status'],'NOT_ELIGIBLE')
        self.assertNotEqual(self.result('profile.json')['cadence_source_sha256'],
                            self.diagnostic['cadence_source_sha256'])

    def test_matching_v3_diagnostic_provenance_is_recorded_without_approval(self):
        self.diagnostic['cadence_source_sha256']=prep.live.cadence_source_hashes()
        self.refresh();self.run_prepare(profile_schema=prep.live.SCHEMA_V3)
        preparation=self.result('preparation.json');binding=preparation['diagnostic_binding']
        self.assertTrue(binding['cadence_sources_match'])
        self.assertTrue(binding['boot_matches_capture_and_calibration'])
        self.assertTrue(binding['motor_power_epoch_matches'])
        self.assertFalse(binding['freshness_or_physical_power_transition_verified'])
        self.assertEqual(preparation['timing_diagnostic_review_only']['kind'],'stop_proxy_diagnostic_only')
        self.assertFalse(self.result('profile.json')['approved_for_supported_policy_output'])

    def test_group_settings_cannot_silently_select_v3_cadence(self):
        settings=prep.settings_template()
        settings['run_settings']['telemetry_cadence']=prep.live.CADENCE_PRE_ENABLE
        with self.assertRaisesRegex(ValueError,'Invalid three-group settings schema'):
            self.run_prepare(group_settings=self.write('settings.json',settings))
        self.assertFalse((self.base/'assembled').exists())

    def test_explicit_three_groups_apply_to_correct_ids_only(self):
        settings=prep.settings_template()
        for group,kp in [('calf',4.),('thigh',8.),('hip',6.)]:settings['groups'][group]['kp']=kp
        settings['run_settings']={'imu_accel_norm_min_m_s2':10.4,'imu_accel_norm_max_m_s2':10.9}
        self.run_prepare(group_settings=self.write('settings.json',settings))
        profile=self.result('profile.json')
        for group,ids in prep.GROUPS.items():
            for mid in ids:
                self.assertEqual(profile['axes'][str(mid)]['kp'],settings['groups'][group]['kp'])
                self.assertIsNone(profile['axes'][str(mid)]['kd'])
        self.assertEqual(profile['imu_accel_norm_min_m_s2'],10.4)
        self.assertFalse(profile['approved_for_supported_policy_output'])

    def test_explicit_prepare_cli_pacing_is_bound_but_never_approved(self):
        self.diagnostic['plan'].update(request_gap_us=800,window=2);self.refresh()
        argv=[]
        for key,value in self.args().items():argv.extend(('--'+key.replace('_','-'),str(value)))
        argv.extend(('--request-gap-us','800','--request-window','2'))
        with redirect_stdout(io.StringIO()) as stdout:
            self.assertEqual(prep.main(argv),0)
        result=json.loads(stdout.getvalue());profile=self.result('profile.json')
        self.assertEqual((profile['request_gap_us'],profile['request_window']),(800,2))
        self.assertFalse(profile['approved_for_supported_policy_output']);self.assertIsNone(profile['review'])
        self.assertFalse(result['output_allowed']);self.assertFalse(result['hardware_opened'])
        self.assertEqual(result['transport_settings'],prep.live.transport_settings(profile))
        self.assertEqual(self.result('hardware-review.json')['reviewed_settings_sha256'],
                         prep.live.reviewed_settings_sha256(profile))
        timing=self.result('preparation.json')['timing_diagnostic_review_only']
        self.assertEqual(timing['kind'],'stop_proxy_diagnostic_only')
        self.assertFalse(timing['actual_policy_output_20ms_verified'])
        self.assertEqual(profile['hard_cycle_ms'],20.)

    def test_diagnostic_pacing_is_not_adopted_as_candidate_default(self):
        self.diagnostic['plan'].update(request_gap_us=800,window=2);self.refresh()
        self.run_prepare();profile=self.result('profile.json')
        self.assertEqual((profile['request_gap_us'],profile['request_window']),(600,3))
        timing=self.result('preparation.json')['timing_diagnostic_review_only']
        self.assertEqual(timing['status'],'NOT_ELIGIBLE')
        self.assertIn('Diagnostic pacing',timing['reason'])
        self.assertFalse(profile['approved_for_supported_policy_output'])

    def test_group_pacing_can_be_explicit_and_cannot_conflict_with_prepare_options(self):
        settings=prep.settings_template()
        settings['run_settings']={'request_gap_us':5000,'request_window':1}
        path=self.write('settings.json',settings)
        with self.assertRaisesRegex(ValueError,'conflicts with group settings'):
            self.run_prepare(group_settings=path,request_gap_us=800)
        self.assertFalse((self.base/'assembled').exists())
        self.run_prepare(group_settings=path,request_gap_us=5000,request_window=1)
        profile=self.result('profile.json')
        self.assertEqual((profile['request_gap_us'],profile['request_window']),(5000,1))
        self.assertFalse(profile['approved_for_supported_policy_output'])

    def test_prepare_pacing_types_and_bounds_reject_before_publication(self):
        for key,values in (('request_gap_us',(True,600.,'600',599,5001)),
                           ('request_window',(True,1.,'1',0,4))):
            for value in values:
                with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                    self.run_prepare(**{key:value})
        self.assertFalse((self.base/'assembled').exists())
        argv=[]
        for key,value in self.args().items():argv.extend(('--'+key.replace('_','-'),str(value)))
        for flags in (('--request-gap-us','599'),('--request-gap-us','5001'),
                      ('--request-window','0'),('--request-window','4')):
            with self.subTest(flags=flags),patch.object(prep,'prepare') as run,redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:prep.main([*argv,*flags])
                self.assertEqual(error.exception.code,2);run.assert_not_called()

    def test_bad_group_limits_flags_and_unknown_keys_rejected(self):
        cases=[]
        for key,value in [('kp',31),('kd',True),('max_temperature_c',float('inf'))]:
            value_settings=prep.settings_template();value_settings['groups']['hip'][key]=value;cases.append(value_settings)
        value_settings=prep.settings_template();value_settings['groups']['hip']['skip_faults']=True;cases.append(value_settings)
        value_settings=prep.settings_template();value_settings['run_settings']['approved_for_supported_policy_output']=True;cases.append(value_settings)
        for n,settings in enumerate(cases):
            # JSON infinity is explicitly rejected before preparation too.
            path=self.base/f'invalid-{n}.json';path.write_text(json.dumps(settings))
            with self.subTest(n=n),self.assertRaises(ValueError):self.run_prepare(group_settings=path)
        self.assertFalse((self.base/'assembled').exists())

    def test_reviewed_physical_values_copy_without_creating_hardware_approval(self):
        evidence=self.write('physical-fixture.json',{'synthetic':'review source only'})
        digest=hashlib.sha256(evidence.read_bytes()).hexdigest()
        self.ap['physical_uncertainty_known']=True;self.ap['evidence_files']={digest:str(evidence)}
        row=self.ap['axes'][0]
        row.update(uncertainty_rad=.01,zero_reviewed=True,direction_reviewed=True,physical_limits_reviewed=True,
            zero_evidence_sha256=digest,direction_evidence_sha256=digest,physical_limits_evidence_sha256=digest)
        self.refresh();self.run_prepare()
        self.assertEqual(self.result('profile.json')['axes']['1']['uncertainty_rad'],.01)
        self.assertEqual(self.result('profile.json')['axes']['1']['physical_lower_rad'],row['lower_rad'])
        self.assertIsNone(self.result('hardware-review.json')['angles']['1']['zero_and_sign_physically_verified'])

    def test_changed_reviewed_angle_evidence_is_rejected(self):
        evidence=self.write('physical-fixture.json',{'synthetic':'original'})
        digest=hashlib.sha256(evidence.read_bytes()).hexdigest();self.ap['evidence_files']={digest:str(evidence)}
        self.ap['axes'][0].update(direction_reviewed=True,direction_evidence_sha256=digest)
        self.refresh();evidence.write_text('{}')
        with self.assertRaisesRegex(ValueError,'evidence file changed'):self.run_prepare()

    def test_candidate_identity_sign_and_offset_must_match_angle_profile(self):
        old=copy.deepcopy(self.cal)
        for field,value in [('sign_candidate',-1),('offset_candidate_rad',.1)]:
            self.cal=copy.deepcopy(old);self.cal['candidates'][0][field]=value;self.refresh()
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,'mismatch'):self.run_prepare()
        self.cal=copy.deepcopy(old);self.cal['identities']['1']='f'*16;self.refresh()
        with self.assertRaisesRegex(ValueError,'identities'):self.run_prepare()

    def test_capture_hash_and_raw_are_bound(self):
        self.capture['telemetry']['rows']['1']['median_position_rad']=-1.1
        self.write('capture.json',self.capture)
        with self.assertRaisesRegex(ValueError,'capture hash'):self.run_prepare()
        self.cal['source_capture_sha256']=hashlib.sha256((self.base/'capture.json').read_bytes()).hexdigest()
        self.write('calibration.json',self.cal)
        with self.assertRaisesRegex(ValueError,'Capture raw'):self.run_prepare()

    def test_source_branch_and_source_angle_profile_hash_are_validated(self):
        self.capture['telemetry']['rows']['1']['median_position_rad']+=2*3.141592653589793
        self.refresh()
        with self.assertRaisesRegex(ValueError,'candidate branch'):self.run_prepare()
        self.capture['telemetry']['rows']['1']['median_position_rad']=-1.;self.refresh()
        self.cal['source_calibration_sha256_by_id']['1']='e'*64;self.write('calibration.json',self.cal)
        with self.assertRaisesRegex(ValueError,'Source angle-profile'):self.run_prepare()

    def test_unique_full_turn_offset_is_preserved_without_approving_it(self):
        self.capture['telemetry']['rows']['3']['median_position_rad']=2*3.141592653589793
        self.cal['candidates'][2].update(diagnostic_branch_turns_embedded_in_offset=1,
            offset_candidate_rad=-2*3.141592653589793)
        self.refresh();self.run_prepare()
        row=self.result('profile.json')['axes']['3']
        self.assertEqual(row['offset_rad'],-2*3.141592653589793)
        self.assertIsNone(self.result('hardware-review.json')['angles']['3']['power_cycle_branch_method_verified'])

    def test_complete_settings_use_existing_braking_contract(self):
        evidence=self.write('all-physical-fixture.json',{'synthetic':'twelve-axis review source'})
        digest=hashlib.sha256(evidence.read_bytes()).hexdigest()
        self.ap.update(physical_uncertainty_known=True,evidence_files={digest:str(evidence)})
        for row in self.ap['axes']:
            row.update(uncertainty_rad=.005,zero_reviewed=True,direction_reviewed=True,physical_limits_reviewed=True,
                zero_evidence_sha256=digest,direction_evidence_sha256=digest,physical_limits_evidence_sha256=digest)
        self.refresh();settings=prep.settings_template()
        for group,ids in prep.GROUPS.items():
            settings['groups'][group]={key:self.original['axes'][str(ids[0])][key] for key in prep.live.LIMIT_CAPS}
        settings['run_settings']={'duration_s':.7,'startup_duration_s':.2,'stop_duration_s':.2,'policy_ramp_s':.2}
        with self.assertRaisesRegex(ValueError,'braking'):
            self.run_prepare(group_settings=self.write('complete-settings.json',settings))
        settings['run_settings']['duration_s']=5.
        self.run_prepare(group_settings=self.write('complete-settings.json',settings))
        self.assertFalse(self.result('profile.json')['approved_for_supported_policy_output'])

    def test_diagnostic_input_hash_mismatch_is_not_hidden_as_missing_measurement(self):
        self.diagnostic['input_sha256']['mount']='a'*64;self.write('pipeline_diagnostic.json',self.diagnostic)
        with self.assertRaisesRegex(ValueError,'Diagnostic input hash'):self.run_prepare()

    def test_failed_diagnostic_and_unknown_epoch_remain_explicitly_unresolved(self):
        self.diagnostic.update(status='ABORTED',errors=['synthetic failure'])
        self.cal['source_current_motor_power_epoch_label']='NOT_INFERRED_FROM_JETSON_BOOT'
        self.capture['motor_power_epoch']='NOT_INFERRED_FROM_JETSON_BOOT'
        self.diagnostic['motor_power_epoch']='NOT_INFERRED_FROM_JETSON_BOOT'
        self.refresh();result=self.run_prepare()
        self.assertIsNone(self.result('profile.json')['boot_id'])
        self.assertIsNone(self.result('profile.json')['motor_power_epoch'])
        codes=[x['code'] for x in result['blockers_by_kind']['missing_measurements']]
        self.assertIn('full_pipeline_diagnostic_not_eligible',codes)

    def test_synthetic_diagnostic_does_not_publish_current_boot(self):
        self.diagnostic['simulated']=True;self.refresh();self.run_prepare()
        self.assertIsNone(self.result('profile.json')['boot_id'])

    def test_declared_power_epoch_requires_same_boot_and_remains_unapproved(self):
        self.cal['source_current_motor_power_epoch_label']='NOT_INFERRED_FROM_JETSON_BOOT'
        self.capture['motor_power_epoch']='NOT_INFERRED_FROM_JETSON_BOOT'
        self.diagnostic.pop('motor_power_epoch')
        self.refresh();result=self.run_prepare(power_epoch='motor-power-20260928-r1')
        profile=self.result('profile.json');binding=self.result('preparation.json')['motor_power_epoch_binding']
        self.assertEqual(profile['motor_power_epoch'],'motor-power-20260928-r1')
        self.assertEqual(binding['source'],'operator_declared')
        self.assertEqual(binding['matched_boot_id'],self.original['boot_id'])
        self.assertEqual(binding['source_labels'],[])
        self.assertFalse(binding['physical_power_transition_verified_by_this_tool'])
        self.assertFalse(profile['approved_for_supported_policy_output'])
        self.assertFalse(result['output_allowed'])
        self.assertNotIn('explicit_motor_power_epoch_unavailable_or_inconsistent',
            [x['code'] for x in result['blockers_by_kind']['file_assembly']])
        self.assertEqual(self.result('hardware-review.json')['reviewed_settings_sha256'],
            prep.live.reviewed_settings_sha256(profile))
        preparation=self.result('preparation.json')
        self.assertFalse(preparation['diagnostic_binding']['motor_power_epoch_matches'])
        self.assertEqual(preparation['timing_diagnostic_review_only']['status'],'NOT_ELIGIBLE')
        self.assertIn('diagnostic_motor_power_epoch_unavailable_or_inconsistent',
            [row['code'] for row in preparation['blockers_by_kind']['missing_measurements']])

    def test_stale_diagnostic_power_epoch_is_not_associated_with_new_capture(self):
        self.diagnostic['motor_power_epoch']='SYNTHETIC OLDER POWER EPOCH'
        self.refresh();self.run_prepare()
        preparation=self.result('preparation.json')
        self.assertIsNone(self.result('profile.json')['motor_power_epoch'])
        self.assertFalse(preparation['diagnostic_binding']['motor_power_epoch_matches'])
        self.assertEqual(preparation['timing_diagnostic_review_only']['status'],'NOT_ELIGIBLE')

    def test_declared_power_epoch_invalid_or_conflicting_labels_rejected(self):
        for label in ('',' ',' UNKNOWN','UNKNOWN','NOT_INFERRED_FROM_JETSON_BOOT','x'*129,
                      'abc\nxyz','abc\x00xyz','abc\x7fxyz','abc\u200bxyz',True):
            with self.subTest(label=repr(label)),self.assertRaisesRegex(ValueError,'Invalid operator-declared'):
                self.run_prepare(power_epoch=label)
        with self.assertRaisesRegex(ValueError,'conflicts with source label'):
            self.run_prepare(power_epoch='different-concrete-power-label')
        self.assertFalse((self.base/'assembled').exists())
        # Matching concrete evidence labels can also be explicitly declared.
        self.run_prepare(power_epoch=self.capture['motor_power_epoch'])
        self.assertEqual(self.result('profile.json')['motor_power_epoch'],self.capture['motor_power_epoch'])

    def test_declared_power_epoch_rejects_missing_capture_boot_mismatch_and_simulation(self):
        label=self.capture['motor_power_epoch']
        with self.assertRaisesRegex(ValueError,'same boot'):
            self.run_prepare(power_epoch=label,source_capture=None)
        original=copy.deepcopy(self.diagnostic)
        for change in ({'boot_id':'aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa'},
                       {'status':'ABORTED'},{'simulated':True},{'hardware_opened':False}):
            self.diagnostic=copy.deepcopy(original);self.diagnostic.update(change);self.refresh()
            with self.subTest(change=change),self.assertRaisesRegex(ValueError,'same boot'):
                self.run_prepare(power_epoch=label)
        self.assertFalse((self.base/'assembled').exists())

    def test_missing_capture_and_bundle_are_file_assembly_blockers(self):
        result=self.run_prepare(source_capture=None,bundle=None)
        codes=[x['code'] for x in result['blockers_by_kind']['file_assembly']]
        self.assertIn('source_capture_file_not_pinned',codes)
        self.assertIn('explicit_pinned_bundle_path_required',codes)
        self.assertFalse(self.result('preparation.json')['calibration_source_capture_pinned'])

    def test_unreviewed_imu_fragment_copies_only_observed_numbers(self):
        fragment={'schema':'singularitydog.imu-review-preparation.v1','status':'UNREVIEWED',
            'approved_for_runtime':False,'dependency_eligible':False,'hardware_opened':False,
            'references':{key:{'sha256':hashlib.sha256((self.base/(key+'.json')).read_bytes()).hexdigest()}
                          for key in ('mount','bias')},
            'imu':dict.fromkeys(prep.IMU_PHYSICAL,True)}
        fragment['imu'].update(corrected_static_gyro_max_rad_s=.005,raw_gravity_norm_min_m_s2=10.6,
            raw_gravity_norm_max_m_s2=10.8,gravity_direction_max_error_rad=.01,norm_deviation_rationale='unreviewed')
        self.run_prepare(imu_fragment=self.write('imu-fragment.json',fragment))
        imu=self.result('hardware-review.json')['imu']
        for key in prep.IMU_PHYSICAL:self.assertIsNone(imu[key])
        self.assertIsNone(imu['gravity_direction_max_error_rad']);self.assertEqual(imu['norm_deviation_rationale'],'')
        self.assertEqual(imu['corrected_static_gyro_max_rad_s'],.005)

    def test_symlink_inputs_and_existing_outputs_are_rejected_without_mutation(self):
        original=(self.base/'calibration.json').read_bytes();link=self.base/'linked.json';link.symlink_to(self.base/'calibration.json')
        with self.assertRaisesRegex(ValueError,'nonsymlink'):self.run_prepare(calibration=link)
        self.run_prepare();before={p:p.read_bytes() for p in (self.base/'assembled').iterdir()}
        with self.assertRaises(FileExistsError):self.run_prepare()
        for path,raw in before.items():self.assertEqual(path.read_bytes(),raw)
        self.assertEqual((self.base/'calibration.json').read_bytes(),original)

    def test_private_output_cannot_be_inside_git(self):
        folder=self.base/'repo';folder.mkdir();(folder/'.git').mkdir()
        with self.assertRaisesRegex(ValueError,'outside Git'):self.run_prepare(output=folder/'private')

    def test_bundle_changed_after_initial_verification_is_rejected_before_publication(self):
        member=next((self.base/'bundle').iterdir())
        serialize=prep._json_bytes
        def mutate_after_bundle_check(value):
            if value.get('schema')==prep.live.REVIEW_SCHEMA:
                member.write_bytes(b'SYNTHETIC CHANGED BUNDLE')
            return serialize(value)
        with patch.object(prep,'_json_bytes',side_effect=mutate_after_bundle_check):
            with self.assertRaisesRegex(ValueError,'Model bundle source changed during assembly'):
                self.run_prepare()
        self.assertFalse((self.base/'assembled').exists())


if __name__=='__main__':unittest.main()
