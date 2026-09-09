"""Observable behavior of the public tools; fixtures are synthetic and not robot results."""
import copy
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import artifact_manifest as manifests
import audit_urdf
import check_publication
import evaluate_trace
import make_dashboard


class ManifestTests(unittest.TestCase):
    def test_changed_bytes_and_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'a.txt').write_text('original')
            manifest = manifests.create(root, ['a.txt'])
            self.assertEqual(manifests.verify(root, manifest), 1)
            (root / 'a.txt').write_text('changed')
            with self.assertRaises(ValueError):
                manifests.verify(root, manifest)
            for path in ('../escape', '/absolute', '.git/config'):
                with self.assertRaises(ValueError):
                    manifests.member(root, path)
            (root / 'link').symlink_to(root / 'a.txt')
            with self.assertRaises(ValueError):
                manifests.member(root, 'link')


class URDFTests(unittest.TestCase):
    def test_fixture(self):
        result = audit_urdf.audit(ROOT / 'examples/synthetic_robot.urdf', 1)
        self.assertEqual(result['actuated_joints'], 1)
        self.assertAlmostEqual(result['total_declared_mass_kg'], 1.1)

    def test_invalid_physical_inertia_axis_and_tree(self):
        original = (ROOT / 'examples/synthetic_robot.urdf').read_text()
        changes = [('izz="0.01"', 'izz="0.03"'), ('ixx="0.01"', 'ixx="-0.01"'),
                   ('xyz="0 1 0"', 'xyz="0 0 0"'), ('<child link="limb"/>', '<child link="base"/>')]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.urdf'
            for old, new in changes:
                path.write_text(original.replace(old, new))
                with self.subTest(change=new), self.assertRaises(ValueError):
                    audit_urdf.audit(path)


class TraceTests(unittest.TestCase):
    def setUp(self):
        self.trace = json.loads((ROOT / 'examples/synthetic_trace.json').read_text())

    def test_complete_speed_and_contact_cycles(self):
        result = evaluate_trace.evaluate(self.trace)
        self.assertTrue(result['completed_requested_interval'])
        self.assertAlmostEqual(result['mean_velocity_along_fixed_world_direction_m_s'], .25)
        self.assertAlmostEqual(result['net_translation_over_observed_time_m_s'], .25)
        self.assertEqual(result['feet']['RL']['completed_swings_peak_ge_15mm'], 2)
        self.assertFalse(result['walking_acceptance'])

    def test_first_failure_cannot_pass_speed_target(self):
        self.trace['samples'][2]['terminal'] = True
        result = evaluate_trace.evaluate(self.trace)
        self.assertEqual(result['completed_samples'], 3)
        self.assertEqual(result['discarded_after_first_end'], 7)
        self.assertFalse(result['speed_target_met_in_complete_interval'])

    def test_initial_airborne_and_end_airborne_not_complete(self):
        for i, row in enumerate(self.trace['samples']):
            for foot in row['feet'].values():
                foot['normal_force_N'] = 0 if i < 4 or i >= 8 else 5
                foot['clearance_m'] = .03
        result = evaluate_trace.evaluate(self.trace)
        self.assertEqual(result['feet']['FL']['completed_swings'], 0)
        self.assertTrue(result['feet']['FL']['end_censored_swing'])

    def test_invalid_time_and_nonfinite_rejected(self):
        for field, value in [('time_s', .123), ('heading_rad', float('nan'))]:
            trace = copy.deepcopy(self.trace)
            trace['samples'][3][field] = value
            with self.assertRaises(ValueError):
                evaluate_trace.evaluate(trace)

    def test_world_direction_is_not_current_heading(self):
        self.trace['direction_world_xy'] = [0, 1]
        for row in self.trace['samples']:
            row['heading_rad'] = 1.5707963267948966
        result = evaluate_trace.evaluate(self.trace)
        self.assertEqual(result['mean_velocity_along_fixed_world_direction_m_s'], 0)


class PublicationTests(unittest.TestCase):
    def test_private_data_and_asset_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'note.md').write_text('host=' + '10.' + '1.2.3')
            self.assertTrue(check_publication.check(root, ['note.md']))
            (root / 'note.md').write_text('公開する独自の検討記録')
            self.assertEqual(check_publication.check(root, ['note.md']), [])
            (root / 'part.step').write_text('not for publication')
            self.assertTrue(check_publication.check(root, ['part.step']))


class DashboardTests(unittest.TestCase):
    def test_video_copy_requires_exact_hash(self):
        data = json.loads((ROOT / 'evidence/improvement-loops.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'synthetic.mp4'
            source.write_bytes(b'synthetic copy test only')
            data['loops'][0]['video'] = {'path': 'synthetic.mp4', 'sha256': '0' * 64}
            index = root / 'index.json'
            index.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                make_dashboard.build(index, root / 'bad.html', True)
            data['loops'][0]['video']['sha256'] = manifests.digest(source)
            index.write_text(json.dumps(data))
            result = make_dashboard.build(index, root / 'good.html', True)
            self.assertEqual(result['videos_included'], 1)
            copied = list((root / 'media').glob('*.mp4'))
            self.assertEqual(len(copied), 1)
            self.assertEqual(manifests.digest(copied[0]), manifests.digest(source))

    def test_unmeasured_results_and_duplicate_loop_rejected(self):
        data = json.loads((ROOT / 'evidence/improvement-loops.json').read_text())
        make_dashboard.validate(data)
        data['loops'][-1]['forward_speed_m_s'] = .2
        with self.assertRaises(ValueError):
            make_dashboard.validate(data)
        data['loops'][-1]['forward_speed_m_s'] = None
        data['loops'][-1]['id'] = data['loops'][0]['id']
        with self.assertRaises(ValueError):
            make_dashboard.validate(data)

    def test_public_dashboard_has_no_video_or_script_injection(self):
        data = json.loads((ROOT / 'evidence/improvement-loops.json').read_text())
        data['notes'] = '</script><script>injected()</script>'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'index.json').write_text(json.dumps(data))
            result = make_dashboard.build(root / 'index.json', root / 'index.html')
            self.assertEqual(result['videos_included'], 0)
            self.assertNotIn(data['notes'], (root / 'index.html').read_text())
            with self.assertRaises(FileExistsError):
                make_dashboard.build(root / 'index.json', root / 'index.html')


class JobTests(unittest.TestCase):
    def test_detached_supervisor_finishes_with_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            command = self.request(root, 'from pathlib import Path\nPath("done.json").write_text(\'{"status":"COMPLETED","updates":2}\')\n')
            result = subprocess.run(command + ['--detach'], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(result.stdout)['status'], 'SUPERVISOR_DISPATCHED_NOT_COMPLETION')
            deadline = time.monotonic() + 5
            status_path = root / 'jobs/test-job/status.json'
            while time.monotonic() < deadline:
                state = json.loads(status_path.read_text())
                if state['status'] in ('COMPLETED', 'FAILED'):
                    break
                time.sleep(.02)
            self.assertEqual(state['status'], 'COMPLETED', state)

    def request(self, root, body, expected=None, timeout=5):
        source = root / 'worker.py'
        source.write_text(body)
        manifest = root / 'manifest.json'
        manifest.write_text(json.dumps(manifests.create(root, ['worker.py'])))
        request = root / 'job.json'
        request.write_text(json.dumps({'schema': 'singularitydog.job.v1', 'job_id': 'test-job',
                                      'argv': [sys.executable, str(source)], 'cwd': str(root), 'timeout_s': timeout,
                                      'artifact_root': str(root), 'manifest_file': str(manifest), 'manifest_sha256': manifests.digest(manifest),
                                      'completion_file': str(root / 'done.json'),
                                      'expected_completion': expected or {'status': 'COMPLETED', 'updates': 2}}))
        return [sys.executable, str(ROOT / 'tools/bounded_job.py'), 'run', '--request', str(request),
                '--jobs', str(root / 'jobs'), '--lock', str(root / 'shared.lock')]

    def test_success_and_duplicate_prevented(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            command = self.request(root, 'from pathlib import Path\nPath("done.json").write_text(\'{"status":"COMPLETED","updates":2}\')\n')
            result = subprocess.run(command, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            status = json.loads((root / 'jobs/test-job/status.json').read_text())
            self.assertEqual(status['status'], 'COMPLETED')
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)

    def test_false_success_and_timeout_fail(self):
        for body, timeout in [('from pathlib import Path\nPath("done.json").write_text(\'{"status":"ERROR"}\')\n', 5),
                              ('import time\ntime.sleep(10)\n', .15)]:
            with self.subTest(timeout=timeout), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                command = self.request(root, body, timeout=timeout)
                result = subprocess.run(command, capture_output=True, timeout=10)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertEqual(json.loads((root / 'jobs/test-job/status.json').read_text())['status'], 'FAILED')

    def test_shared_lock_prevents_second_worker(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            command = self.request(root, 'from pathlib import Path\nPath("executed").touch()\n')
            with (root / 'shared.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                result = subprocess.run(command, capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertFalse((root / 'executed').exists())


if __name__ == '__main__':
    unittest.main()
