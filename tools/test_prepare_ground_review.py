import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from prepare_ground_review import prepare, review_template


class GroundReviewPreparationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.base=Path(self.temp.name)
        self.plan=self.write('plan.json',{'stage':'stand'})
        self.profile=self.write('profile.json',{'fixture':'synthetic'})
        self.report=self.write('report.json',{
            'schema':'singularitydog.ground-trial-report.v1','stage':'stand',
            'stage_plan_sha256':self.sha(self.plan),'profile_sha256':self.sha(self.profile),
            'status':'COMPLETE_BOUNDED_GROUND_TRIAL','simulation_only':True,
            'runtime_report':{'cycles':[{'begin_ns':1000}],
                'stop_reports':{'front':{'evidence':{'records':[{'received_ns':2000}]}},
                                'rear':{'evidence':{'records':[{'received_ns':2100}]}}}}})
        self.video=self.base/'fixture-video.bin';self.video.write_bytes(b'SYNTHETIC_NOT_VIDEO'*1000)

    def write(self,name,value):
        path=self.base/name;path.write_text(json.dumps(value));return path

    def sha(self,path):return hashlib.sha256(path.read_bytes()).hexdigest()

    def test_all_sources_bound_but_video_and_physical_facts_remain_unknown(self):
        out=self.base/'review'
        result=prepare(self.report,self.plan,self.profile,self.video,out)
        self.assertFalse(result['hardware_opened']);self.assertFalse(result['dependency_eligible'])
        association=json.loads((out/'association.json').read_text())
        for key,path in (('report',self.report),('plan',self.plan),('profile',self.profile),('video',self.video)):
            self.assertEqual(association['references'][key],{'path':str(path.resolve()),'sha256':self.sha(path)})
        self.assertEqual(association['sync'],{'trial_start_ns':1000,'trial_end_ns':2100,
            'video_start_s':None,'video_end_s':None,'uncertainty_ms':None})
        review=json.loads((out/'physical-review-template.json').read_text())
        self.assertEqual(review['decision'],'UNREVIEWED')
        self.assertTrue(all(v is None for v in review['observations'].values()))
        self.assertEqual((out.stat().st_mode&0o777),0o700)
        self.assertEqual(((out/'association.json').stat().st_mode&0o777),0o600)

    def test_mismatched_report_fails_before_output_creation(self):
        self.plan.write_text('{"stage":"walk"}')
        with self.assertRaisesRegex(ValueError,'do not match'):
            prepare(self.report,self.plan,self.profile,self.video,self.base/'bad')
        self.assertFalse((self.base/'bad').exists())

    def test_aborted_missing_cycles_stays_unknown(self):
        r=json.loads(self.report.read_text());r['runtime_report']={};r['status']='ABORTED'
        self.report.write_text(json.dumps(r))
        prepare(self.report,self.plan,self.profile,self.video,self.base/'aborted')
        a=json.loads((self.base/'aborted/association.json').read_text())
        self.assertIsNone(a['sync']['trial_start_ns']);self.assertIsNone(a['sync']['trial_end_ns'])

    def test_final_association_binding_never_approves_review(self):
        out=self.base/'review';prepare(self.report,self.plan,self.profile,self.video,out)
        association=out/'association.json'
        a=json.loads(association.read_text());a['synchronization_method']='synthetic test edit'
        association.write_text(json.dumps(a))
        final=out/'review-bound.json';result=review_template(association,final)
        value=json.loads(final.read_text())
        self.assertEqual(value['association_sha256'],self.sha(association))
        self.assertEqual(result['status'],value['decision']);self.assertEqual(value['decision'],'UNREVIEWED')
        self.assertIsNone(value['reviewed_by'])
        with self.assertRaisesRegex(ValueError,'new private'):review_template(association,final)

    def test_git_output_and_symlink_input_rejected(self):
        git=self.base/'repo';git.mkdir();(git/'.git').mkdir()
        with self.assertRaisesRegex(ValueError,'outside Git'):
            prepare(self.report,self.plan,self.profile,self.video,git/'review')
        link=self.base/'link';link.symlink_to(self.video)
        with self.assertRaisesRegex(ValueError,'Regular'):
            prepare(self.report,self.plan,self.profile,link,self.base/'review')


if __name__=='__main__':unittest.main()
