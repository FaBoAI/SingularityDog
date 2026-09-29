"""Synthetic preload admission tests; no robot evidence or transport access."""
import copy
import contextlib
import io
import stat
import hashlib
import json
import math
import unittest

from singularitydog_hw import policy_live_profile as profile
from singularitydog_hw.supported_preload_path import _fraction
import test_policy_rare_jitter_profile as rare_fixture
from test_policy_local_profile import seal_local
from test_policy_live_profile import _write


class SupportedPreloadProfileTests(unittest.TestCase):
    def setUp(self):
        rare_fixture.RareJitterProfileTests.setUp(self)
        self.prior = json.loads(rare_fixture.RareJitterProfileTests.seal(self).read_text())
        self.data.update(diagnostic_timing_acceptance=profile.SUPPORTED_PRELOAD_5S,
            duration_s=5., startup_duration_s=1., policy_ramp_s=.2,
            stop_duration_s=.4, policy_weight=0.)
        self.data.pop('startup_damping_duration_s')
        self.data.pop('post_reply_deadline_policy')
        self.data['cadence_source_sha256'] = profile.cadence_source_hashes(self.data)
        for axis in self.data['axes'].values():
            axis.update(kp=6., max_estimated_pd_torque_nm=.2, max_measured_velocity_rad_s=.25)
        report = self.docs['pipeline_diagnostic']
        report.update(motor_power_epoch=self.data['motor_power_epoch'],
            cadence_source_sha256=copy.deepcopy(self.data['cadence_source_sha256']))
        self.docs['preload_source_profile'] = self.prior
        raw = {i:self.docs['local_reference_capture']['telemetry']['rows'][i]['median_position_rad']
               for i in profile.IDS}
        screen = dict(screen_failures=[], motor_output_allowed=False, rise_mm=.25,
            initial_model_rad_by_id=raw.copy(), initial_raw_rad_by_id=raw.copy(),
            up_direction_body_unit_vector=[0.,0.,1.],
            readiness_blockers=['SYNTHETIC direction not reviewed'])
        path = dict(schema='singularitydog.supported-preload-path-file-only.v1',
            duration_s=5., period_s=.02, source_screen=screen, samples=[],
            motor_output_allowed=False, approved_for_runtime=False,
            learned_model_output=False, box_removal_allowed=False,
            blockers=['SYNTHETIC direction not reviewed'])
        for slot in range(251):
            t=slot*.02; fraction=_fraction(t)
            target={mid:q+.003*fraction for mid,q in raw.items()}
            path['samples'].append(dict(time_s=t, rise_fraction=fraction,
                q_model_rad_by_id=target, q_raw_rad_by_id=target.copy()))
        self.docs['preload_path'] = path
        self.review = dict(schema='singularitydog.supported-preload-review.v1',
            review={**self.data['review'],'decision':'ACCEPT_SUPPORTED_GEOMETRIC_PRELOAD_5S'},
            mode=profile.SUPPORTED_PRELOAD_5S, scope=self.data['scope'],
            boot_id=self.data['boot_id'], motor_power_epoch=self.data['motor_power_epoch'],
            assembly_id=self.data['assembly_id'],
            source_sha256=copy.deepcopy(self.data['cadence_source_sha256']),
            support_must_remain=True, low_catch_must_remain=True,
            immediate_power_cutoff_ready=True, four_foot_contact_observed=True,
            physical_corridor_and_direction_verified=True, encoder_branch_rechecked=True,
            original_candidate_not_promoted=True, absolute_accuracy_not_certified=True,
            load_bearing_not_established=True, standing_allowed=False,
            walking_allowed=False, box_removal_allowed=False,
            verified_up_direction_body_unit_vector=[0.,0.,1.],
            direction_and_contact_evidence='SYNTHETIC ONLY, no real physical observation',
            blocker_dispositions={'SYNTHETIC direction not reviewed':'SYNTHETIC ONLY explicit review'})
        self.validation = dict(schema='singularitydog.supported-preload-source-validation.v1',
            status='PASS_FILE_ONLY_TESTS', hardware_opened=False, errors=[],
            source_sha256=copy.deepcopy(self.data['cadence_source_sha256']),
            checks={k:True for k in ('normal_extend_return_and_stop', 'fault_stop_both_buses',
                'cancellation_stop', 'stale_input_stop', 'path_mutation_and_replay_rejected',
                'return_target_and_measured_confirmation')},
            test_command='SYNTHETIC unittest fixture', test_output_sha256='e'*64, tests_passed=12)
        self.docs['preload_review'] = self.review
        for name in profile.artifact_names(self.data):
            self.data['artifacts'].setdefault(name, {'path':name+'.json','sha256':'0'*64})

    def seal(self):
        for name in ('preload_source_profile','local_reference_capture',
                     'pipeline_diagnostic','command_loss_report'):
            self.data['artifacts'][name] = _write(self.base/(name+'.json'), self.docs[name])
        screen=self.docs['preload_path']['source_screen']
        screen.update(profile_sha256=self.data['artifacts']['preload_source_profile']['sha256'],
                      capture_sha256=self.data['artifacts']['local_reference_capture']['sha256'])
        self.data['artifacts']['preload_path'] = _write(self.base/'preload_path.json',self.docs['preload_path'])
        for key,name in (('path_sha256','preload_path'),('source_profile_sha256','preload_source_profile'),
                         ('capture_sha256','local_reference_capture'),('diagnostic_sha256','pipeline_diagnostic'),
                         ('command_loss_sha256','command_loss_report')):
            self.review[key]=self.data['artifacts'][name]['sha256']
        self.review['software_validation']=_write(self.base/'software-validation.json',self.validation)
        seal_local(self.base,self.data,self.docs)
        return self.base/'profile.json'

    def load(self):
        return profile.load_profile(self.seal())

    def test_supported_finite_path_has_loader_token_and_preserves_candidate(self):
        loaded=self.load();settings=profile.supported_preload_settings(loaded)
        self.assertTrue(loaded['output_allowed'])
        self.assertEqual(settings['return_complete_s'],4.)
        self.assertEqual(loaded['policy_weight'],0.)
        self.assertFalse(profile.current_position_hold_only(loaded))
        self.assertTrue(loaded['support_must_remain'])
        self.assertFalse(settings['path']['approved_for_runtime'])
        self.assertEqual(settings['path']['blockers'],self.docs['preload_path']['blockers'])
        self.assertEqual(loaded['timing_review']['twenty_ms_misses'],0)

    def test_template_plan_only_has_no_proof_and_does_not_open_artifacts(self):
        data=profile.supported_preload_template()
        path=self.base/'unapproved.json';_write(path,data)
        loaded=profile.load_profile(path,require_approved=False)
        self.assertFalse(loaded['output_allowed'])
        with self.assertRaises(profile.ProfileError): profile.supported_preload_settings(loaded)
        with self.assertRaises(profile.ProfileError): profile.load_profile(path)

    def test_cli_preload_template_is_private_unapproved_and_never_overwritten(self):
        path=self.base/'preload-template.json'
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(profile.main(['--write-preload-template',str(path)]),0)
        original=path.read_bytes()
        self.assertEqual(stat.S_IMODE(path.stat().st_mode),0o600)
        self.assertFalse(profile.load_profile(path,require_approved=False)['output_allowed'])
        self.assertEqual(json.loads(original)['diagnostic_timing_acceptance'],profile.SUPPORTED_PRELOAD_5S)
        self.assertEqual(json.loads(original)['request_gap_us'],900)
        self.assertEqual(json.loads(original)['request_window'],3)
        with self.assertRaises(FileExistsError):
            profile.main(['--write-preload-template',str(path)])
        self.assertEqual(path.read_bytes(),original)
        legacy=self.base/'legacy-template.json'
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(profile.main(['--write-template',str(legacy)]),0)
        self.assertEqual(json.loads(legacy.read_text())['schema'],profile.SCHEMA)
        self.assertNotIn('diagnostic_timing_acceptance',json.loads(legacy.read_text()))

    def test_old_source_manifest_remains_exact_and_extra_only_for_opt_in(self):
        self.assertEqual(set(profile.cadence_source_hashes()),set(profile.CADENCE_SOURCE_PATHS))
        self.assertEqual(set(profile.cadence_source_hashes(self.data))-set(profile.CADENCE_SOURCE_PATHS),
                         {'singularitydog_hw/supported_preload_path.py'})
        self.data['cadence_source_sha256']=profile.cadence_source_hashes()
        with self.assertRaisesRegex(profile.ProfileError,'source pins'): self.load()

    def test_mode_cannot_be_repurposed_for_gain_duration_or_timing_escalation(self):
        for key,value in (('duration_s',5.1),('startup_duration_s',.9),('policy_weight',.01),
                          ('stop_duration_s',.401),('hard_cycle_ms',21),
                          ('max_sample_age_ms',21),('max_consecutive_20ms_misses',1)):
            old=self.data[key];self.data[key]=value
            with self.subTest(key=key), self.assertRaises(profile.ProfileError): self.load()
            self.data[key]=old
        for key,value in (('kp',6.001),('kd',.151),('max_estimated_pd_torque_nm',.201),
                          ('max_displacement_from_start_rad',math.radians(1.01))):
            old=self.data['axes']['1'][key];self.data['axes']['1'][key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()
            self.data['axes']['1'][key]=old

    def test_exact_source_power_epoch_and_all500_steady_deadlines_required(self):
        report=self.docs['pipeline_diagnostic']
        for key,value in (('motor_power_epoch','stale'),('cadence_source_sha256',{})):
            old=report[key];report[key]=value
            with self.subTest(key=key),self.assertRaisesRegex(profile.ProfileError,'current power'):self.load()
            report[key]=old
        row=report['measurements'][-1];row['cycle_end_ns']=row['scheduled_release_ns']+20_000_001
        with self.assertRaises(profile.ProfileError):self.load()

    def test_path_screen_failure_or_missing_blocker_review_is_not_waivable(self):
        self.docs['preload_path']['source_screen']['screen_failures']=['SYNTHETIC out-of-range']
        with self.assertRaisesRegex(profile.ProfileError,'screen failed'):self.load()
        self.docs['preload_path']['source_screen']['screen_failures']=[]
        self.review['blocker_dispositions']={}
        with self.assertRaisesRegex(profile.ProfileError,'Every original'):self.load()

    def test_software_fault_stop_and_physical_direction_are_required(self):
        for key,value in (('physical_corridor_and_direction_verified',False),
                          ('box_removal_allowed',True),('walking_allowed',True),
                          ('verified_up_direction_body_unit_vector',[1.,0.,0.])):
            old=self.review[key];self.review[key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):self.load()
            self.review[key]=old
        self.validation['checks']['fault_stop_both_buses']=False
        with self.assertRaisesRegex(profile.ProfileError,'fault/STOP'):self.load()

    def test_token_is_not_reusable_with_changed_targets_gains_epoch_or_artifacts(self):
        loaded=self.load()
        for change in (lambda d:d.update(policy_weight=.5),
                       lambda d:d.update(motor_power_epoch='other'),
                       lambda d:d['axes']['1'].update(kp=10),
                       lambda d:d['artifacts']['preload_path'].update(sha256='f'*64),
                       lambda d:d['_preload_path']['samples'][50]['q_model_rad_by_id'].update({'1':0.})):
            mutated={**loaded,'axes':copy.deepcopy(loaded['axes']),
                     'artifacts':copy.deepcopy(loaded['artifacts']),
                     '_preload_path':copy.deepcopy(loaded['_preload_path'])}
            change(mutated)
            with self.subTest(change=change),self.assertRaises(profile.ProfileError):
                profile.supported_preload_settings(mutated)

    def test_source_and_capture_angles_cannot_be_substituted(self):
        self.docs['preload_source_profile']['axes']['1']['offset_rad']+=.01
        with self.assertRaisesRegex(profile.ProfileError,'historical calibration'):self.load()
        self.docs['preload_source_profile']['axes']['1']['offset_rad']-=.01
        for row in self.docs['preload_path']['samples']:
            row['q_raw_rad_by_id']['1']+=2*math.pi
        with self.assertRaises(profile.ProfileError):self.load()


if __name__=='__main__':unittest.main()
