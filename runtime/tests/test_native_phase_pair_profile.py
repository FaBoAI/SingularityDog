"""Transport opt-in requires its own source-bound admission; no hardware."""
import copy
import unittest

import test_prepared_voltage_publication_profile as fixture
from test_policy_local_profile import seal_local
from singularitydog_hw import policy_live_profile as profile


class NativePairProfileTests(unittest.TestCase):
    select=fixture.PreparedVoltageProfileTests.select

    def setUp(self):
        fixture.PreparedVoltageProfileTests.setUp(self)
        self.data.update(native_phase_pair=True,request_gap_us=900,request_window=3)
        report=self.docs['pipeline_diagnostic']
        report['native_phase_pair']=True
        report['plan'].update(native_phase_pair=True,request_gap_us=900,request_window=3)
        report['native_phase_pair_proof']=dict(mode='persistent_dual_owner.v1',
            request_count_per_cycle=26,all_phases_joined=True,
            owner_placement_verified=True,owner_settings_restored=True,
            coordinator_placement_verified=True,coordinator_settings_restored=True,
            active_deadlines_unchanged=True)
        self.docs['hardware_review']['native_phase_pair_acceptance']=dict(
            schema='singularitydog.native-phase-pair-review.v1',mode='persistent_dual_owner.v1',
            scope=self.data['scope'],request_count_per_cycle=26,active_deadlines_unchanged=True,
            stop_proxy_does_not_certify_active_api_latency=True,
            cadence_source_sha256=copy.deepcopy(self.data['cadence_source_sha256']),
            hard_output_and_freshness_limits_unchanged=True,
            review={**self.data['review'],'decision':'ACCEPT_NATIVE_PHASE_PAIR'})

    def seal(self):
        seal_local(self.base,self.data,self.docs)
        for name in ('rare_jitter_diagnostic_acceptance','voltage_pipeline_acceptance',
                     'prepared_voltage_publication_acceptance','native_phase_pair_acceptance'):
            self.docs['hardware_review'][name]['diagnostic_sha256']=self.data['artifacts']['pipeline_diagnostic']['sha256']
        seal_local(self.base,self.data,self.docs)
        return self.base/'profile.json'

    def test_complete_loader_binds_explicit_selection(self):
        value=profile.load_profile(self.seal())
        self.assertTrue(profile.native_phase_pair_settings(value))
        self.assertEqual(profile.execution_settings(value)['native_phase_pair'],'persistent_dual_owner.v1')
        self.assertFalse(value['actual_policy_output_20ms_verified'])

    def select_gap(self, gap):
        self.data['request_gap_us']=gap
        self.docs['pipeline_diagnostic']['plan']['request_gap_us']=gap

    def assert_selected_gap_retains_limits(self, gap):
        self.select_gap(gap)
        value=profile.load_profile(self.seal())
        self.assertTrue(profile.native_phase_pair_settings(value))
        self.assertEqual(profile.transport_settings(value,request_gap_us=gap,request_window=3)
                         ['request_gap_us'],gap)
        self.assertEqual((value['hard_cycle_ms'],value['max_sample_age_ms'],
                          value['duration_s'],value['policy_weight']),(20.,20.,2.,.005))
        self.assertEqual(value['max_consecutive_20ms_misses'],0)
        self.assertEqual(value['max_sample_gap_ms'],21.)
        self.assertFalse(value['actual_policy_output_20ms_verified'])
        for requested in (880,890,900):
            if requested==gap:continue
            with self.subTest(requested=requested),self.assertRaisesRegex(
                    profile.ProfileError,'differs from reviewed profile'):
                profile.transport_settings(value,request_gap_us=requested)
        value['request_gap_us']=900
        with self.assertRaisesRegex(profile.ProfileError,'complete loader proof'):
            profile.native_phase_pair_settings(value)

    def test_880_requires_its_matching_reviewed_diagnostic_and_retains_limits(self):
        self.assert_selected_gap_retains_limits(880)

    def test_890_requires_its_matching_reviewed_diagnostic_and_retains_limits(self):
        self.assert_selected_gap_retains_limits(890)

    def test_each_selected_gap_rejects_the_other_diagnostic_and_window(self):
        for selected,measured in ((a,b) for a in (880,890,900) for b in (880,890,900) if a!=b):
            self.select_gap(selected)
            plan=self.docs['pipeline_diagnostic']['plan']
            plan['request_gap_us']=measured
            with self.subTest(selected=selected,measured=measured),self.assertRaisesRegex(
                    profile.ProfileError,'Diagnostic pacing differs'):
                profile.load_profile(self.seal())
            plan.update(request_gap_us=selected,window=2)
            with self.subTest(selected=selected,window=2),self.assertRaisesRegex(
                    profile.ProfileError,'Diagnostic pacing differs'):
                profile.load_profile(self.seal())
            plan['window']=3

    def test_880_and_890_keep_fresh_source_boot_power_and_coordinator_proof_required(self):
        original=copy.deepcopy(self.docs['pipeline_diagnostic'])
        for gap in (880,890):
            self.docs['pipeline_diagnostic']=copy.deepcopy(original)
            self.select_gap(gap)
            selected=copy.deepcopy(self.docs['pipeline_diagnostic'])
            for key,bad in (('boot_id','different-boot'),('motor_power_epoch','different-power'),
                            ('cadence_source_sha256',{})):
                self.docs['pipeline_diagnostic']=copy.deepcopy(selected)
                self.docs['pipeline_diagnostic'][key]=bad
                with self.subTest(gap=gap,key=key),self.assertRaisesRegex(profile.ProfileError,'own current diagnostic'):
                    profile.load_profile(self.seal())
            for key in ('all_phases_joined','owner_settings_restored',
                        'coordinator_placement_verified','coordinator_settings_restored'):
                self.docs['pipeline_diagnostic']=copy.deepcopy(selected)
                self.docs['pipeline_diagnostic']['native_phase_pair_proof'][key]=False
                with self.subTest(gap=gap,key=key),self.assertRaisesRegex(profile.ProfileError,'own current diagnostic'):
                    profile.load_profile(self.seal())

    def test_890_does_not_expand_deadlines_motion_or_gain_caps(self):
        self.select_gap(890)
        for key,bad in (('hard_cycle_ms',21),('max_sample_age_ms',21),
                        ('max_consecutive_20ms_misses',1),('request_window',2),
                        ('policy_weight',.0051),('preauthorized_boxed_sequence',True)):
            value=copy.deepcopy(self.data);value[key]=bad
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):
                profile.execution_settings(value)
        for key,bad in (('kp',3.001),('kd',.151),
                        ('max_displacement_from_start_rad',.018),
                        ('max_estimated_pd_torque_nm',.101)):
            value=copy.deepcopy(self.data);value['axes']['1'][key]=bad
            with self.subTest(axis_key=key),self.assertRaises(profile.ProfileError):
                profile.execution_settings(value)

    def test_absent_or_false_keeps_legacy_route(self):
        for selected in (None,False):
            value=profile.template(schema=profile.SCHEMA_V3)
            if selected is False:value['native_phase_pair']=False
            self.assertFalse(profile.native_phase_pair_settings(value))
            self.assertNotIn('native_phase_pair',profile.execution_settings(value))

    def test_raw_selected_profile_is_not_execution_permission(self):
        with self.assertRaisesRegex(profile.ProfileError,'complete loader proof'):
            profile.native_phase_pair_settings(self.data)

    def test_mutation_after_loading_rejects_selection(self):
        value=profile.load_profile(self.seal());value['axes']['1']['kp']=2.
        with self.assertRaisesRegex(profile.ProfileError,'complete loader proof'):
            profile.native_phase_pair_settings(value)

    def test_old_diagnostic_cannot_qualify_new_transport(self):
        self.docs['pipeline_diagnostic'].pop('native_phase_pair')
        with self.assertRaisesRegex(profile.ProfileError,'Diagnostic native phase pair'):
            profile.load_profile(self.seal())

    def test_unrestored_owner_or_changed_deadline_is_rejected(self):
        for key in ('owner_settings_restored','all_phases_joined','active_deadlines_unchanged'):
            proof=self.docs['pipeline_diagnostic']['native_phase_pair_proof']
            proof[key]=False
            with self.subTest(key=key),self.assertRaisesRegex(profile.ProfileError,'own current diagnostic'):
                profile.load_profile(self.seal())
            proof[key]=True

    def test_missing_or_false_coordinator_placement_and_restoration_is_rejected(self):
        proof=self.docs['pipeline_diagnostic']['native_phase_pair_proof']
        for key in ('coordinator_placement_verified','coordinator_settings_restored'):
            for value in (None,False):
                if value is None:proof.pop(key)
                else:proof[key]=value
                with self.subTest(key=key,value=value),self.assertRaisesRegex(
                        profile.ProfileError,'own current diagnostic'):
                    profile.load_profile(self.seal())
                proof[key]=True

    def test_old_or_different_source_review_is_rejected(self):
        review=self.docs['hardware_review']['native_phase_pair_acceptance']
        review['cadence_source_sha256']['singularitydog_hw/policy_output_runtime.py']='a'*64
        with self.assertRaisesRegex(profile.ProfileError,'source-bound native phase pair'):
            profile.load_profile(self.seal())

    def test_nonbaseline_pacing_and_extended_scope_are_rejected(self):
        for key,value in (('request_gap_us',850),('request_gap_us',870),('request_gap_us',879),
                          ('request_gap_us',891),('request_gap_us',880.),('request_gap_us',890.),
                          ('request_gap_us',True),('request_window',2),
                          ('policy_weight',.1),('preauthorized_boxed_sequence',True)):
            data=copy.deepcopy(self.data);data[key]=value
            with self.subTest(key=key),self.assertRaises(profile.ProfileError):
                profile.execution_settings(data)


if __name__=='__main__':unittest.main()
