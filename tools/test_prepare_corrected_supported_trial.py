"""Synthetic file-only drafts; mocked face audits are never hardware evidence."""
import copy
import hashlib
import io
import json
import math
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'runtime'), str(ROOT/'runtime'/'tests'), str(ROOT/'tools')]
from test_policy_local_profile import local_fixture
from singularitydog_hw.policy_post_reply_timing import POST_REPLY_POLICY
import prepare_corrected_supported_trial as tool


class DraftTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name).resolve()
        self.prior, self.docs, model_pins = local_fixture(self.base)
        self.enterContext(patch.object(tool.live.shadow, 'SOURCE_HASHES', model_pins))
        self.prior.update(model_backend=tool.live.SCALAR_BACKEND, voltage_overlap=True,
            voltage_pipeline=True, request_gap_us=900, request_window=3,
            diagnostic_timing_acceptance=tool.live.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
            startup_damping_duration_s=.08, startup_cycle_allowance=tool.live.FIRST_CYCLE_POST_REPLY,
            post_reply_deadline_policy=dict(mode=POST_REPLY_POLICY, max_lateness_ms=1.,
                max_consecutive_misses=1, rolling_window_cycles=100, max_misses_per_window=1))
        for row in self.prior['axes'].values(): row.update(tool.CAPS)
        binary = self.base/'bundle'/'synthetic.so'; binary.write_bytes(b'SYNTHETIC NEVER DLOPEN')
        self.prior['native_batch_encoder'] = {'path': binary.name,
            'sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
        for name in ('scalar_step_manifest',): self.prior['artifacts'][name] = {'path': None, 'sha256': None}
        self.documents = {k: copy.deepcopy(self.docs[k]) for k in ('calibration', 'mount', 'model_manifest')}
        self.documents['gyro_bias'] = copy.deepcopy(self.docs['bias'])
        capture = copy.deepcopy(self.docs['local_reference_capture'])
        capture.update(angle_wrap_applied=False, plan={'allowed_can_types': [0, 17]},
            motor_power_epoch='NOT_INFERRED_FROM_JETSON_BOOT')
        for row in capture['telemetry']['rows'].values(): row.update(current=0, position_span_deg=.01)
        self.documents.update(current_capture=capture,
            expected_uids=self.documents['calibration']['identities'],
            prior_profile=self.prior, prior_report={
                'status': 'COMPLETE_SUPPORTED_OUTPUT', 'errors': [],
                'boot_id': self.prior['boot_id'], 'motor_power_epoch': self.prior['motor_power_epoch'],
                'scope': self.prior['scope'], 'motor_enable_sent': True, 'learned_targets_sent': True,
                'motion_gain_sent': True, 'normal_ramp_completed': True, 'stop_confirmed': True,
                'stop_reports': {scope: {'complete': True, 'confirmed_ids': ids,
                    'unconfirmed_ids': [], 'ambiguous_ids': []} for scope, ids in
                    (('front', list(range(1, 7))), ('rear', list(range(7, 13))))}},
            accel_diagnostic_input={'synthetic_test_only': True},
            accel_diagnostic_report={'synthetic_test_only': True, 'input_bindings': [], 'source_bindings': []})
        cal = self.documents['calibration']
        cal.update(source_current_boot_id=capture['boot_id'],
            source_raw_rad_by_id={mid: row['median_position_rad'] for mid, row in capture['telemetry']['rows'].items()})
        for row in cal['candidates']: row['diagnostic_branch_turns_embedded_in_offset'] = 0
        self.documents['scalar_step_manifest'] = {'schema': 'native-step-scalar-file-only-v1',
            'status': 'PASS_FILE_ONLY_COMPARE', 'hardware_opened': False, 'output_allowed': False,
            'approved_for_runtime': False, 'live_50hz_verified': False}
        self.request = {'schema': tool.INPUT_SCHEMA, 'assembly_id': 'SYNTHETIC-ONLY',
            'expected_boot_id': self.prior['boot_id'], 'motor_power_epoch': None,
            'files': {}, 'bundle': str(self.base/'bundle')}
        self.template = {'schema': tool.hypothesis.SCHEMA, 'scope': tool.hypothesis.SCOPE,
            'manifest': None, 'candidate': None, 'R_body_from_sensor': self.documents['mount']['R_body_from_sensor'],
            'raw_norm_bounds_m_s2': None, 'corrected_norm_bounds_m_s2': None,
            'source_sha256': tool.hypothesis.source_hashes(),
            'hypothesis_review': {'decision': 'UNREVIEWED', 'reviewer': None, 'reviewed_at': None, 'rationale': None},
            'assumptions_acknowledged': dict.fromkeys(tool.hypothesis.ASSUMPTIONS, False),
            'formal_calibration_approved': False, 'absolute_orientation_error_bound_rad': None,
            'grants_motor_output': False}
        self.audit = self.enterContext(patch.object(tool.hypothesis, 'create_template', side_effect=self.fake_template))
        self.refresh()

    def fake_template(self, manifest, candidate, rotation):
        result = copy.deepcopy(self.template)
        result.update(manifest=manifest, candidate=candidate, R_body_from_sensor=rotation)
        return result

    def write(self, name, value):
        path = self.base/(name+'.json'); path.write_bytes(tool.json_bytes(value))
        return tool.reference(tool.read_file(path, tool.FILE_MAX)[1])

    def refresh(self):
        for name in tool.FILE_NAMES:
            if name in ('calibration', 'prior_report', 'scalar_step_manifest'): continue
            self.request['files'][name] = self.write(name, self.documents[name])
        self.documents['calibration']['source_capture_sha256'] = self.request['files']['current_capture']['sha256']
        self.documents['prior_report']['profile_sha256'] = self.request['files']['prior_profile']['sha256']
        self.documents['scalar_step_manifest']['baseline_manifest_sha256'] = self.request['files']['model_manifest']['sha256']
        for name in ('calibration', 'prior_report', 'scalar_step_manifest'):
            self.request['files'][name] = self.write(name, self.documents[name])
        self.write('input', self.request)

    def prepare(self): return tool.prepare(self.base/'input.json', self.base/'draft')
    def read(self, name): return json.loads((self.base/'draft'/name).read_text())

    def test_plan_uses_new_selection_without_transferring_any_old_permission(self):
        before = {p: p.read_bytes() for p in self.base.rglob('*') if p.is_file()}
        result = self.prepare(); profile = self.read('profile.json')
        self.assertEqual(result['status'], 'UNAPPROVED_FILE_ONLY_DRAFT')
        for flag in tool.FLAGS: self.assertIs(result[flag], False)
        self.assertTrue(profile['accel_input_hypothesis'])
        self.assertFalse(profile.get('apply_reviewed_accel_calibration', False))
        self.assertFalse(profile['approved_for_supported_policy_output']); self.assertIsNone(profile['review'])
        self.assertTrue(profile['blockers'])
        for name in ('hardware_review', 'operator_acceptance', 'command_loss_report', 'pipeline_diagnostic'):
            self.assertEqual(profile['artifacts'][name], {'path': None, 'sha256': None})
        self.assertEqual(self.read('historical-profile.json'), self.prior)
        for path, raw in before.items(): self.assertEqual(path.read_bytes(), raw)
        plan = tool.live.load_profile(self.base/'draft'/'profile.json', require_approved=False)
        self.assertFalse(plan['output_allowed'])
        with self.assertRaisesRegex(ValueError, 'unapproved'):
            tool.live.load_profile(self.base/'draft'/'profile.json')

    def test_explicit_fk_candidate_stays_unapproved_and_preserves_old_success(self):
        candidate = {'synthetic_file_only_candidate': True}
        self.request['files']['target_fk_manifest'] = self.write('target-fk', candidate)
        self.write('input', self.request)
        from singularitydog_hw import policy_active_fk
        proof = {'schema': 'singularitydog.active-fk-file-plan.v1',
            'torch_or_native_loaded': False, 'model_sha256': 'a' * 64, 'library_sha256': 'b' * 64,
            'active_binding': {'adapter_source_sha256': hashlib.sha256(Path(policy_active_fk.__file__).read_bytes()).hexdigest()},
            **dict.fromkeys(('output_allowed', 'approved_for_runtime',
                'active_controller_qualification', 'timing_admission_eligible', 'live_50hz_verified'), False)}
        with patch.object(policy_active_fk, 'plan', return_value=proof) as plan:
            self.prepare()
        profile = self.read('profile.json'); report = self.read('preparation.json')
        self.assertTrue(profile['native_target_fk_cache'])
        self.assertEqual(profile['artifacts']['target_fk_manifest'], self.request['files']['target_fk_manifest'])
        self.assertFalse(profile['approved_for_supported_policy_output'])
        self.assertEqual(self.read('historical-profile.json'), self.prior)
        self.assertFalse(report['fk_type1_live_timing_verified'])
        self.assertEqual(len(report['input_bindings']), len(tool.FILE_NAMES) + 2)
        self.assertEqual(plan.call_count, 2)
        self.assertFalse(plan.call_args.args[0]['approved_for_supported_policy_output'])

    def test_fk_plan_failure_does_not_publish_a_draft(self):
        self.request['files']['target_fk_manifest'] = self.write('target-fk', {})
        self.write('input', self.request)
        from singularitydog_hw import policy_active_fk
        with patch.object(policy_active_fk, 'plan', side_effect=ValueError('Wrong candidate provenance')):
            with self.assertRaisesRegex(ValueError, 'provenance'): self.prepare()
        self.assertFalse((self.base / 'draft').exists())

    def test_second_fk_verification_failure_does_not_publish_a_completed_draft(self):
        self.request['files']['target_fk_manifest'] = self.write('target-fk', {})
        self.write('input', self.request)
        from singularitydog_hw import policy_active_fk
        with patch.object(policy_active_fk, 'plan', side_effect=[{}, ValueError('Evidence changed in preflight')]):
            with self.assertRaisesRegex(ValueError, 'preflight'): self.prepare()
        self.assertFalse((self.base / 'draft').exists())

    def test_optional_fk_pin_cannot_be_an_unknown_model_inventory(self):
        self.request['files']['other_model'] = self.write('other', {})
        self.write('input', self.request)
        with self.assertRaisesRegex(ValueError, 'inventory'): self.prepare()

    def test_hypothesis_is_official_unreviewed_template_not_formal_calibration(self):
        self.prepare(); doc = self.read('accel-input-hypothesis-draft.json')
        self.audit.assert_called_once_with(self.request['files']['accel_diagnostic_input'],
            self.request['files']['accel_diagnostic_report'], self.documents['mount']['R_body_from_sensor'])
        self.assertEqual(doc['hypothesis_review']['decision'], 'UNREVIEWED')
        self.assertIsNone(doc['raw_norm_bounds_m_s2']); self.assertIsNone(doc['corrected_norm_bounds_m_s2'])
        self.assertFalse(doc['formal_calibration_approved']); self.assertFalse(doc['grants_motor_output'])
        self.assertTrue(all(v is False for v in doc['assumptions_acknowledged'].values()))

    def test_exact_old_half_percent_caps_and_raw_norm_monitors_preserved(self):
        self.prepare(); profile = self.read('profile.json')
        for key in ('duration_s', 'policy_weight', 'imu_accel_norm_min_m_s2', 'imu_accel_norm_max_m_s2',
                    'post_reply_deadline_policy', 'hard_cycle_ms', 'max_sample_age_ms'):
            self.assertEqual(profile[key], self.prior[key])
        for row in profile['axes'].values():
            for key, value in tool.CAPS.items(): self.assertEqual(row[key], value)
            self.assertIsNone(row['uncertainty_rad'])
        self.assertEqual(profile['policy_weight'], .005)

    def test_changed_duration_mixture_gains_or_hard_deadline_not_accepted_as_prior(self):
        original = copy.deepcopy(self.prior)
        for key, value in [('duration_s', 10.), ('policy_weight', .01), ('hard_cycle_ms', 21.),
                           ('request_gap_us', 600), ('max_sample_age_ms', 21.)]:
            self.documents['prior_profile'] = copy.deepcopy(original); self.documents['prior_profile'][key] = value
            self.refresh()
            with self.subTest(key=key), self.assertRaises(ValueError): self.prepare()
        self.documents['prior_profile'] = copy.deepcopy(original)
        self.documents['prior_profile']['axes']['1']['kp'] = 6.; self.refresh()
        with self.assertRaises(ValueError): self.prepare()
        self.assertFalse((self.base/'draft').exists())

    def test_prior_report_mismatch_or_incomplete_stop_rejected(self):
        original = copy.deepcopy(self.documents['prior_report'])
        for key, value in [('status', 'ABORTED'), ('learned_targets_sent', False), ('normal_ramp_completed', False),
                           ('boot_id', 'other'), ('scope', 'standing')]:
            self.documents['prior_report'] = dict(original, **{key: value}); self.refresh()
            with self.subTest(key=key), self.assertRaises(ValueError): self.prepare()
        self.documents['prior_report'] = copy.deepcopy(original)
        self.documents['prior_report']['stop_reports']['rear']['ambiguous_ids'] = [7]; self.refresh()
        with self.assertRaisesRegex(ValueError, 'STOP'): self.prepare()

    def test_selected_uid_capture_and_calibration_are_all_bound(self):
        self.documents['expected_uids']['1'] = 'f'*16; self.refresh()
        with self.assertRaisesRegex(ValueError, 'UID'): self.prepare()
        self.assertFalse((self.base/'draft').exists())

    def test_wrong_calibration_boot_or_raw_angles_rejected(self):
        for key, value in [('source_current_boot_id', 'other'), ('source_raw_rad_by_id', {})]:
            original = self.documents['calibration'][key]
            self.documents['calibration'][key] = value; self.refresh()
            with self.subTest(key=key), self.assertRaises(ValueError): self.prepare()
            self.documents['calibration'][key] = original

    def test_capture_sha_mismatch_is_not_rebound_automatically(self):
        self.documents['calibration']['source_capture_sha256'] = '0'*64
        self.request['files']['calibration'] = self.write('calibration', self.documents['calibration'])
        self.write('input', self.request)
        with self.assertRaisesRegex(ValueError, 'capture SHA'): self.prepare()

    def test_physical_sign_or_nonintegral_origin_change_rejected(self):
        row = self.documents['calibration']['candidates'][0]
        original = copy.deepcopy(row)
        for key, value in [('sign_candidate', -1), ('offset_candidate_rad', .001)]:
            row.update(original); row[key] = value; self.refresh()
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'sign|zero'): self.prepare()

    def test_saved_integer_turn_is_used_without_rewrapping_raw_capture(self):
        row = self.documents['calibration']['candidates'][0]
        row['offset_candidate_rad'] = -2*math.pi; row['diagnostic_branch_turns_embedded_in_offset'] = 1
        self.documents['current_capture']['telemetry']['rows']['1']['median_position_rad'] += 2*math.pi
        self.documents['calibration']['source_raw_rad_by_id']['1'] += 2*math.pi
        self.refresh(); self.prepare()
        axis = self.read('preparation.json')['axis_diagnostic_descriptions']['1']
        self.assertEqual(axis['offset_difference_integer_turns'], -1)
        self.assertAlmostEqual(axis['model_rad'], -1.)
        self.assertIsNone(axis['physical_clearance_verified'])

    def test_outside_model_and_one_degree_limit_margin_rejected_without_clipping(self):
        for q in (-2.3, -.081):
            self.documents['current_capture']['telemetry']['rows']['1']['median_position_rad'] = q
            self.documents['calibration']['source_raw_rad_by_id']['1'] = q
            self.refresh()
            with self.subTest(q=q), self.assertRaisesRegex(ValueError, 'outside|interval'): self.prepare()

    def test_changed_input_during_face_reaudit_blocks_all_publication(self):
        original = self.fake_template
        def mutate(*args):
            result = original(*args)
            (self.base/'gyro_bias.json').write_bytes(b'{}\n')
            return result
        self.audit.side_effect = mutate
        with self.assertRaisesRegex(ValueError, 'changed'): self.prepare()
        self.assertFalse((self.base/'draft').exists())

    def test_norm_reaudit_failure_preserves_inputs_and_publishes_nothing(self):
        self.audit.side_effect = ValueError('Synthetic norm SHA changed')
        with self.assertRaisesRegex(ValueError, 'norm SHA'): self.prepare()
        self.assertFalse((self.base/'draft').exists())

    def test_model_manifest_and_bundle_bytes_must_match_selected_exact_pins(self):
        name = next(iter(tool.live.shadow.SOURCE_HASHES)); (self.base/'bundle'/name).write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'bundle'): self.prepare()

    def test_missing_encoder_is_explicit_blocker_not_a_build_or_ABI_claim(self):
        (self.base/'bundle'/'synthetic.so').unlink(); self.prepare()
        prepared = self.read('preparation.json')
        self.assertIn('native_batch_encoder_bytes_not_present_in_selected_bundle', prepared['blockers'])
        self.assertEqual(self.read('profile.json')['native_batch_encoder'], self.prior['native_batch_encoder'])

    def test_input_symlink_and_output_inside_git_are_refused(self):
        alias = self.base/'alias.json'; alias.symlink_to(self.base/'mount.json')
        self.request['files']['mount']['path'] = str(alias); self.write('input', self.request)
        with self.assertRaisesRegex(ValueError, 'symlink'): self.prepare()
        self.refresh()
        git_root = self.base/'synthetic-git-root'; git_root.mkdir()
        (git_root/'.git').mkdir()
        with self.assertRaisesRegex(ValueError, 'outside Git'):
            tool.prepare(self.base/'input.json', git_root/'private-draft-must-not-exist')

    def test_explicit_power_label_stays_an_operator_label_not_hardware_proof(self):
        self.request['motor_power_epoch'] = 'SYNTHETIC OPERATOR LABEL'; self.write('input', self.request)
        self.prepare(); doc = self.read('preparation.json')
        self.assertFalse(doc['current_boot_verified_on_target'])
        self.assertEqual(doc['motor_power_epoch_label'], 'SYNTHETIC OPERATOR LABEL')
        self.assertNotIn('current_motor_power_epoch_not_declared', doc['blockers'])
        self.assertIn('fresh_target_boot_power_pose_reconfirmation_pending', doc['blockers'])

    def test_formal_acceleration_extension_not_used_as_new_hypothesis(self):
        self.documents['gyro_bias']['accel_calibration_review'] = {}; self.refresh()
        with self.assertRaisesRegex(ValueError, 'formal'): self.prepare()

    def test_member_manifest_covers_exact_saved_bytes_and_files_are_private(self):
        self.prepare(); manifest = self.read('manifest.json')
        self.assertEqual(set(manifest['files']), {p.name for p in (self.base/'draft').iterdir()}-{'manifest.json'})
        for name, pin in manifest['files'].items():
            path = self.base/'draft'/name
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), pin['sha256'])
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
        self.assertEqual((self.base/'draft').stat().st_mode & 0o777, 0o700)
        with self.assertRaisesRegex(ValueError, 'Fresh'): self.prepare()

    def test_cli_has_no_execute_or_finalize_path(self):
        with redirect_stdout(io.StringIO()) as stdout:
            self.assertEqual(tool.main(['--input', str(self.base/'input.json'), '--output', str(self.base/'draft')]), 0)
        self.assertFalse(json.loads(stdout.getvalue())['output_allowed'])
        for option in ('--execute', '--finalize'):
            with self.subTest(option=option), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                tool.main(['--input', str(self.base/'input.json'), '--output', str(self.base/'other'), option])


if __name__ == '__main__': unittest.main()
