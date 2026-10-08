"""End-to-end synthetic file-only sequence generation; no motor observations."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

import prepare_preauthorized_boxed_sequence as tool
import test_policy_preauthorized_boxed_sequence as fixture


class PreparationTests(unittest.TestCase):
    def setUp(self):
        self.fixture=fixture.PreauthorizedBoxedSequenceTests()
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.base=self.fixture.base
        auth=self.fixture.auth
        conditions=auth['current_conditions_source']
        self.context=dict(schema='private.current-r9-preauthorization-source-context.v1',
            sequence_authorization_source=dict(kind='latest_direct_user_message_in_codex',
                exact_text='SYNTHETIC allow automatic boxed2,10,30; confirmation questions waived'),
            observation_reference=conditions,post_trial_physical_observation_inferred=False,
            physical_anomaly_after_new_trials=None,physical_audio_heard_after_new_trials=None,
            box_removal_allowed=False,standing_allowed=False,walking_allowed=False)
        self.context_ref=self.fixture.write(self.base/'context.json',self.context)

    def args(self,duration,profile,permission,capture,report=None,receipt=None):
        values=['--duration',str(duration),'--kit-runtime',str(Path(tool.live.__file__).resolve().parents[1]),
                '--output',str(self.base.resolve()/('prepared-'+str(duration)))]
        for name,ref in (('profile',profile),('preauthorization',permission),('current-capture',capture),
                         ('source-manifest',self.fixture.auth['source_manifest']),
                         ('prior-report',report),('prior-execution-receipt',receipt)):
            if ref is not None:values.extend(['--'+name,ref['path'],'--'+name+'-sha256',ref['sha256']])
        return tool.parser().parse_args(values)

    def base_args(self):
        data,docs,directory=self.fixture.nodes[0]
        profile=dict(path=str(directory/'profile.json'),sha256=hashlib.sha256((directory/'profile.json').read_bytes()).hexdigest())
        return self.args(2,profile,self.context_ref,data['artifacts']['boxed_sequence_current_capture'])

    def extended_args(self,duration,prior_result):
        node=self.fixture.nodes[1 if duration==10 else 2]
        data,docs,directory=node
        report=copy.deepcopy(docs['prior_supported_report'])
        report['profile_sha256']=prior_result['profile']['sha256']
        receipt=json.loads(Path(docs['prior_supported_observation']['execution_receipt']['path']).read_text())
        receipt['profile_sha256']=prior_result['profile']['sha256']
        report_ref=self.fixture.write(self.base/('actual-'+str(duration)+'.json'),report)
        receipt['report_sha256']=report_ref['sha256']
        receipt_ref=self.fixture.write(self.base/('receipt-'+str(duration)+'.json'),receipt)
        return self.args(duration,prior_result['profile'],prior_result['preauthorization'],
                         data['artifacts']['boxed_sequence_current_capture'],report_ref,receipt_ref)

    def test_default_plan_does_not_write_or_claim_success(self):
        args=self.base_args()
        with patch.object(tool,'write') as writer,patch.object(Path,'mkdir') as mkdir:
            plan=tool.prepare(args)
        writer.assert_not_called();mkdir.assert_not_called()
        self.assertEqual(plan['status'],'PLAN_ONLY')
        self.assertFalse(plan['output_allowed']);self.assertFalse(plan['motor_enable_sent'])
        self.assertFalse(Path(args.output).exists())

    def test_original_files_generate_full_current_2_10_30_chain_with_unknown_physical_results(self):
        args=self.base_args();args.prepare=True
        two=tool.prepare(args)
        args=self.extended_args(10,two);args.prepare=True
        ten=tool.prepare(args)
        args=self.extended_args(30,ten);args.prepare=True
        thirty=tool.prepare(args)
        for seconds,result in ((2,two),(10,ten),(30,thirty)):
            self.assertTrue(result['profile_loader_accepted']);self.assertFalse(result['output_allowed'])
            loaded=tool.live.load_profile(result['profile']['path'])
            self.assertEqual(loaded['duration_s'],seconds)
            self.assertEqual(loaded['_post_trial_physical_observation'],
                             dict(audio_heard=None,anomalies=None,support_maintained=None,observed=False))
        self.assertEqual(two['sequence_contract_sha256'],thirty['sequence_contract_sha256'])
        self.assertEqual(two['preauthorization'],ten['preauthorization'])
        self.assertEqual(two['preauthorization'],thirty['preauthorization'])

    def test_explicit_pins_no_overwrite_current_source_and_consecutive_stage_are_required(self):
        for change in (lambda a:setattr(a,'profile_sha256','f'*64),
                       lambda a:setattr(a,'source_manifest_sha256','f'*64),
                       lambda a:setattr(a,'duration',30)):
            args=self.base_args();change(args)
            with self.subTest(change=change),self.assertRaises((ValueError,tool.live.ProfileError)):tool.prepare(args)
        args=self.base_args();args.prepare=True;tool.prepare(args)
        with self.assertRaisesRegex(tool.live.ProfileError,'Fresh private'):tool.prepare(args)

    def test_legacy_or_aborted_predecessor_cannot_create_a_duration_profile(self):
        args=self.base_args();args.prepare=True;two=tool.prepare(args)
        args=self.extended_args(10,two)
        report=json.loads(Path(args.prior_report).read_text());report['status']='ABORTED'
        ref=self.fixture.write(self.base/'aborted-report.json',report)
        args.prior_report=ref['path'];args.prior_report_sha256=ref['sha256'];args.prepare=True
        with self.assertRaises(tool.live.ProfileError):tool.prepare(args)
        if Path(args.output).exists():
            with self.assertRaises(tool.live.ProfileError):tool.live.load_profile(Path(args.output)/'profile.json')

    def test_context_cannot_claim_future_audio_or_replace_direct_power(self):
        for change in (lambda c:c.update(physical_audio_heard_after_new_trials=True),
                       lambda c:c.update(post_trial_physical_observation_inferred=True)):
            context=copy.deepcopy(self.context);change(context)
            ref=self.fixture.write(self.base/'bad-context.json',context)
            args=self.base_args();args.preauthorization=ref['path'];args.preauthorization_sha256=ref['sha256']
            with self.subTest(change=change),self.assertRaises(tool.live.ProfileError):tool.prepare(args)

    def test_failed_loader_or_changed_original_removes_only_its_fresh_approved_draft(self):
        args=self.base_args();args.prepare=True
        baseline=Path(args.profile).read_bytes()
        with patch.object(tool.live,'load_profile',side_effect=tool.live.ProfileError('Injected full-loader rejection')):
            with self.assertRaisesRegex(tool.live.ProfileError,'Injected'):tool.prepare(args)
        self.assertFalse(Path(args.output).exists());self.assertEqual(Path(args.profile).read_bytes(),baseline)
        loader=tool.live.load_profile
        def load_then_change(path):
            value=loader(path)
            Path(args.profile).write_bytes(baseline+b' ')
            return value
        with patch.object(tool.live,'load_profile',side_effect=load_then_change):
            with self.assertRaisesRegex(tool.live.ProfileError,'SHA256 mismatch'):tool.prepare(args)
        self.assertFalse(Path(args.output).exists())
        self.assertTrue(Path(args.profile).exists())

    def test_entry_without_bytecode_disabled_refuses_before_runtime_imports(self):
        result=subprocess.run([sys.executable,'-E',str(Path(tool.__file__).resolve()),'--help'],
                              capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('Launch the file-only entry with python -B',result.stderr)


if __name__=='__main__':unittest.main()
