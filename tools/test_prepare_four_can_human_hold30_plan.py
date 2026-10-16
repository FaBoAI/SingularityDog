"""Reference-only PLAN tests. Fixtures never qualify output or operator facts."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import prepare_four_can_human_hold30_plan as tool
import test_normalize_four_bus_type1_30hz_report as fixture


class FilePlanBuilderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.ProjectionTests.setUpClass()
        cls.profile=copy.deepcopy(fixture.ProjectionTests.profile)

    @classmethod
    def tearDownClass(cls):
        fixture.ProjectionTests.tearDownClass()

    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        self.root=Path(tmp.name).resolve()
        self.profile_ref=self.save('base.json',self.profile)
        self.evidence_ref=self.save('refs.json',{'schema':tool.REFS_SCHEMA,'evidence':{}})

    def save(self,name,value):
        path=self.root/name;raw=(json.dumps(value,allow_nan=False)+'\n').encode();path.write_bytes(raw)
        return {'path':str(path),'sha256':hashlib.sha256(raw).hexdigest()}

    def test_absent_current_facts_stay_blocked_and_do_not_load_capabilities(self):
        with patch.object(tool.H,'prepare_from_files',side_effect=AssertionError('must not prepare')):
            plan,review=tool.build(self.profile_ref,self.evidence_ref)
        self.assertEqual(plan['status'],'PLAN_ONLY')
        self.assertIsNone(plan['conditions'])
        self.assertIn('CURRENT_TWO_PERSON_FULL_SUPPORT_VIDEO_AUDIO_TTY_FACTS_NOT_SUPPLIED',review['blockers'])
        self.assertTrue(review['assets']['missing'])
        self.assertTrue(all(review[key] is False for key in tool.FALSE_FLAGS))
        self.assertTrue(all(plan[key] is False for key in tool.H.NO_GRANTS))
        with self.assertRaises(ValueError):tool.H.runtime_spec(plan)

    def test_requested_primary_review_without_current_facts_cannot_pass(self):
        with patch.object(tool.H,'prepare_from_files',side_effect=AssertionError('must not prepare')):
            _,review=tool.build(self.profile_ref,self.evidence_ref,review_primary=True)
        self.assertEqual(review['primary_review']['status'],'BLOCKED')
        self.assertIn('conditions are missing',review['primary_review']['error'])
        self.assertFalse(review['output_allowed'])

    def test_inlined_or_unselected_conditions_cannot_become_physical_facts(self):
        for document in ({'schema':tool.REFS_SCHEMA,'evidence':{},'conditions':{'all_good':True}},
            {'schema':tool.REFS_SCHEMA,'evidence':{'operator_count':2}},
            {'schema':tool.REFS_SCHEMA,'evidence':{'current_conditions':{'path':'/fake','sha256':'a'*64}}}):
            with self.subTest(document=document):
                ref=self.save('bad-refs.json',document)
                with self.assertRaises(ValueError):tool.build(self.profile_ref,ref)

    def test_failed_source_asset_review_stays_separate_from_runtime_qualification(self):
        refs={'source_root':str(self.root),'source_count':1,
            'source_manifest':{'path':'/fake/source.json','sha256':'1'*64},
            'source_review':{'path':'/fake/review.json','sha256':'2'*64}}
        ref=self.save('source-refs.json',{'schema':tool.REFS_SCHEMA,'evidence':refs})
        _,review=tool.build(self.profile_ref,ref)
        self.assertTrue(review['assets']['errors'])
        self.assertIsNone(review['assets']['source'])
        self.assertEqual(review['status'],'PLAN_ONLY_NOT_EXECUTABLE')

    def test_changed_inputs_and_reused_publication_directory_are_rejected(self):
        before=self.profile_ref
        Path(before['path']).write_text('{}\n')
        with self.assertRaises(ValueError):tool.build(before,self.evidence_ref)
        profile=self.save('new-base.json',self.profile)
        plan,review=tool.build(profile,self.evidence_ref)
        output=self.root/'fresh-plan'
        refs=tool.publish(output,plan,review)
        self.assertEqual(set(refs),{'plan.json','file-only-review.json'})
        self.assertEqual(json.loads((output/'plan.json').read_text())['status'],'PLAN_ONLY')
        with self.assertRaises(ValueError):tool.publish(output,plan,review)


if __name__=='__main__':unittest.main()
