"""Candidate input semantics and negative boundaries; no motor I/O."""
import contextlib
import copy
import io
import json
import math
import unittest
from unittest.mock import patch

from singularitydog_hw import native_pipeline_benchmark as benchmark
from singularitydog_hw import policy_output_model as model_module
from test_policy_observer import make, snapshot
from test_policy_output_model import PolicyOutputModelTests, torch

REFERENCE = {'path': '/synthetic/hypothesis.json', 'sha256': 'a'*64}


class SyntheticCorrection:
    """Loader validation has its own raw-capture tests; this isolates consumers."""
    def correct(self, raw):
        corrected = [raw[0]-.5, raw[1], raw[2]+.1]
        norm = math.hypot(*corrected)
        if not 9.0 <= norm <= 10.5:
            raise ValueError('Synthetic corrected norm exceeded')
        return corrected, norm

    def provenance(self):
        return {'kind': 'synthetic-test-only', 'hypothesis_sha256': REFERENCE['sha256'],
                'formal_calibration_approved': False, 'grants_motor_output': False}


class DiagnosticHypothesisIntegrationTests(unittest.TestCase):
    def test_observer_replaces_only_gravity_and_records_candidate_as_unapproved(self):
        source = snapshot()
        saved = copy.deepcopy(source)
        plain = make()
        with patch('singularitydog_hw.imu_accel_input_hypothesis.load_accel_input_hypothesis',
                   return_value=SyntheticCorrection()) as load:
            corrected = make(accel_input_hypothesis=REFERENCE)
        load.assert_called_once()
        for run in (plain, corrected):
            run.reset_run(source['tick_ns'], warmup_completed=True)
            run.consume(source)
        for index in (0, 2, 3, 4, 5):
            self.assertEqual(plain._policy.calls[-1][index], corrected._policy.calls[-1][index])
        expected = [0., .5/math.hypot(.5, 9.9), -9.9/math.hypot(.5, 9.9)]
        for got, wanted in zip(corrected._policy.calls[-1][1], expected):
            self.assertAlmostEqual(got, wanted)
        summary = corrected.summary()
        self.assertEqual(summary['accel_input_hypothesis'], SyntheticCorrection().provenance())
        self.assertNotIn('accel_input_hypothesis', plain.summary())
        self.assertEqual(saved, source)

    def test_diagnostic_cli_plan_pins_explicit_candidate_without_opening_hardware(self):
        output = io.StringIO()
        args = ['--mode', 'stop-proxy', '--supported-disabled', '--v3-voltage-proxy',
                '--provenance-mode', 'supported-policy-probe-2s-rare-jitter-v1',
                '--power-epoch', 'synthetic-current-epoch',
                '--accel-input-hypothesis', REFERENCE['path'],
                '--accel-input-hypothesis-sha256', REFERENCE['sha256']]
        with contextlib.redirect_stdout(output):
            self.assertEqual(benchmark.main(args), 0)
        plan = json.loads(output.getvalue())
        self.assertEqual(plan['accel_input_hypothesis'], REFERENCE)
        self.assertFalse(plan['enable_available'])
        self.assertFalse(plan['apply_reviewed_accel_calibration'])
        sources = plan['source_provenance']
        self.assertTrue(sources['accel_input_hypothesis'])
        self.assertIn('singularitydog_hw/imu_accel_input_hypothesis.py', sources['cadence_source_sha256'])
        report = {'status':'COMPLETE_DIAGNOSTIC','errors':[]}
        benchmark._finish_source_provenance(report, sources)
        self.assertTrue(sources['source_files_unchanged'])
        self.assertEqual(report['errors'], [])

    def test_cli_rejects_incomplete_or_incompatible_candidate_selection(self):
        base = ['--mode', 'stop-proxy', '--supported-disabled', '--v3-voltage-proxy',
                '--accel-input-hypothesis', REFERENCE['path'],
                '--accel-input-hypothesis-sha256', REFERENCE['sha256']]
        variants = [base[:-2], base+['--acquisition-only'], base+['--compare-feedback'],
                    base+['--apply-reviewed-accel-calibration'],
                    [x for x in base if x != '--supported-disabled'],
                    base[:1]+['type17']+base[2:], base[:-1]+['not-a-hash']]
        for args in variants:
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit):
                benchmark.main(args)


@unittest.skipIf(torch is None, 'CPU PyTorch is unavailable')
class LiveHypothesisIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.fixture = PolicyOutputModelTests('test_warmup_then_one_reset_preserves_fresh_state_across_live_ticks')
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def load(self, correction=None):
        f = self.fixture
        with patch.object(model_module, 'accel_input_hypothesis_settings', return_value=REFERENCE), \
                patch('singularitydog_hw.imu_accel_input_hypothesis.load_accel_input_hypothesis',
                      return_value=correction or SyntheticCorrection()):
            return model_module.LivePolicyModel(f.profile, policy=f.policy, torch_module=torch)

    def test_live_gravity_candidate_keeps_raw_norm_and_provenance(self):
        f = self.fixture
        model = self.load()
        source = f.imu()
        saved = copy.deepcopy(source)
        values = model.validate_inputs(f.sample, source, f.now)
        expected = [0., .5/math.hypot(.5, 9.71), -9.71/math.hypot(.5, 9.71)]
        for got, wanted in zip(values[1], expected):
            self.assertAlmostEqual(got, wanted)
        record = model.last_validation
        self.assertAlmostEqual(record['raw_accel_norm_m_s2'], 9.81)
        self.assertAlmostEqual(record['corrected_accel_norm_m_s2'], math.hypot(.5,9.71))
        self.assertEqual(record['accel_input_hypothesis'], SyntheticCorrection().provenance())
        self.assertNotIn('reviewed_accel_calibration', record)
        self.assertEqual(record['raw_tilt_rad'], 0.)
        self.assertEqual(source, saved)

    def test_corrected_level_cannot_hide_raw_tilt(self):
        class HidesTilt(SyntheticCorrection):
            def correct(self, raw):
                return [0., 0., -9.81], 9.81
        f = self.fixture
        model = self.load(HidesTilt())
        source = f.imu()
        angle = min(math.pi/2, model.profile['imu_tilt_limit_rad']+.1)
        source['accel_m_s2'] = [9.81*math.sin(angle), 0., -9.81*math.cos(angle)]
        with self.assertRaisesRegex(ValueError, 'Raw body tilt exceeded'):
            model.validate_inputs(f.sample, source, f.now)

    def test_raw_level_cannot_hide_corrected_tilt(self):
        class Tilts(SyntheticCorrection):
            def correct(self, raw):
                return [9.81,0.,0.], 9.81
        f = self.fixture
        model = self.load(Tilts())
        with self.assertRaisesRegex(ValueError, 'Body tilt/angular velocity exceeded'):
            model.validate_inputs(f.sample, f.imu(), f.now)

    def test_locomotion_override_is_rejected_even_if_zero(self):
        f = self.fixture
        model = self.load()
        with self.assertRaisesRegex(ValueError, 'boxed-only'):
            model(f.sample, f.imu(), f.now, command_override=[0.,0.,0.])

    def test_existing_raw_norm_and_sample_age_still_reject(self):
        f = self.fixture
        model = self.load()
        source = f.imu()
        source['accel_m_s2'] = [0.,0.,-30.]
        with self.assertRaisesRegex(ValueError, 'gravity-proxy norm'):
            model.validate_inputs(f.sample, source, f.now)
        source = f.imu()
        source['read_started_monotonic_ns'] -= 1_000_000_000
        with self.assertRaisesRegex(ValueError, 'Stale/noncausal'):
            model.validate_inputs(f.sample, source, f.now)


if __name__ == '__main__':
    unittest.main()
