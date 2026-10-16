"""Synthetic file-only projection replay; no physical observation or grant.

Reuse the raw normalizer's packet fixture to exercise the new consumer's
original-byte chain. These documents never enter an opaque execution proof.
"""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

from . import profile as H

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
import test_normalize_four_bus_type1_30hz_report as fixture


class OriginalObservationChainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.ProjectionTests.setUpClass()
        cls.profile = copy.deepcopy(fixture.ProjectionTests.profile)
        cls.original = copy.deepcopy(fixture.ProjectionTests.report)

    @classmethod
    def tearDownClass(cls):
        fixture.ProjectionTests.tearDownClass()

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.directory = Path(tmp.name).resolve()
        self.profile_ref = self.save('profile.json', self.profile)
        self.original_ref = self.save('original.json', self.original)
        self.projected = fixture.tool.normalize(self.original, self.profile,
            original_reference=self.original_ref, profile_reference=self.profile_ref,
            expected_source_sha256=self.profile['contract']['source_manifest']['sha256'])
        self.manifest = {'files': {name: {'sha256': hashlib.sha256((ROOT/name).read_bytes()).hexdigest()}
            for name in ('tools/analyze_four_bus_type1_failure.py',
                         'tools/normalize_four_bus_type1_30hz_report.py')}}

    def save(self, name, value):
        path = self.directory/name
        path.write_text(json.dumps(value, allow_nan=False)+'\n')
        return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}

    def check(self, projected=None, manifest=None):
        document = self.projected if projected is None else projected
        ref = self.save('projected.json', document)
        pins = H.Pins()
        result = H.verified_predecessor_observation_sha(pins.json(ref), pins.json(self.profile_ref),
            report_ref=ref, profile_ref=self.profile_ref, source_root=str(ROOT),
            manifest=self.manifest if manifest is None else manifest, pins=pins)
        return result, pins

    def test_original_sha_survives_exact_replayed_projection_without_new_observation(self):
        result, pins = self.check()
        self.assertEqual(result, self.original_ref['sha256'])
        self.assertIn(self.original_ref['path'], pins.values)
        self.assertEqual(len(pins.values), 5)
        self.assertNotIn('physical_observation', self.projected)
        with self.assertRaises(ValueError):
            H.runtime_spec({'report_sha256': result})

    def test_tampered_raw_status_metadata_and_copied_deadline_never_replay(self):
        for change in (
            lambda d: d['measurement']['cycles'][0]['output']['port3']['records'][2].update(rx_hex='00'*17),
            lambda d: d.update(status='ABORTED'),
            lambda d: d.update(inference_tx_budget_ns=30_000_000),
            lambda d: d['compatibility_deadline_field_projection']['raw_validation'].update(
                original_Type1_output_records_checked=999),
            lambda d: d['compatibility_deadline_field_projection'].update(output_approval_granted_here=True),
            lambda d: d['compatibility_deadline_field_projection'].update(schema='forged'),
            lambda d: d['compatibility_deadline_field_projection']['original_profile'].update(sha256='0'*64)):
            with self.subTest(change=repr(change)):
                data=copy.deepcopy(self.projected); change(data)
                with self.assertRaises(ValueError):
                    self.check(data)

    def test_missing_or_replaced_original_is_not_replaced_by_derivative(self):
        Path(self.original_ref['path']).unlink()
        with self.assertRaises((ValueError, FileNotFoundError)):
            self.check()
        Path(self.original_ref['path']).write_text('{"fake": true}\n')
        with self.assertRaises(ValueError):
            self.check()

    def test_unfrozen_auditor_and_changed_raw_original_cannot_be_pinned_through(self):
        manifest=copy.deepcopy(self.manifest)
        manifest['files']['tools/normalize_four_bus_type1_30hz_report.py']['sha256']='0'*64
        with self.assertRaises(ValueError):
            self.check(manifest=manifest)
        original=copy.deepcopy(self.original)
        raw=original['measurement']['cycles'][0]['output']['port3']['records'][2]
        raw.update(finish_ns=original['measurement']['cycles'][0]['write_deadline_ns'])
        # Even if an attacker provides new SHA-pinned originals, the actual raw
        # audit rejects them; the derived complete flag cannot repair packets.
        self.original_ref=self.save('original.json',original)
        data=copy.deepcopy(self.projected)
        data['compatibility_deadline_field_projection']['original_report']=self.original_ref
        with self.assertRaises(ValueError):
            self.check(data)

    def test_report_without_projection_keeps_its_own_sha(self):
        ref=self.save('plain.json',self.original)
        self.assertEqual(H.verified_predecessor_observation_sha(self.original,self.profile,
            report_ref=ref,profile_ref=self.profile_ref,source_root=str(ROOT),manifest=self.manifest,pins=H.Pins()),
            ref['sha256'])


if __name__ == '__main__':
    unittest.main()
