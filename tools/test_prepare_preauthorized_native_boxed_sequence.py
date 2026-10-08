"""Synthetic saved-file native stage preparation; no hardware or human facts."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

import prepare_preauthorized_native_boxed_sequence as tool
import test_policy_preauthorized_native_boxed_sequence as fixture


class NativePreparationTests(unittest.TestCase):
    FIXTURE_CLASS=fixture.NativeBoxedPreauthorizationTests
    def setUp(self):
        self.fixture=self.FIXTURE_CLASS()
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.base=self.fixture.base
        self.context=dict(schema=tool.live.NATIVE_BOXED_SEQUENCE_CONTEXT_SCHEMA,
            sequence_authorization_source=dict(kind='latest_direct_user_message_in_codex',
                exact_text='SYNTHETIC native890 boxed 2/10/20; automatic continuation and questions waived'),
            authorized_durations_s=[2,10,20],native_phase_pair=True,request_gap_us=890,request_window=3,
            automatic_continuation_explicitly_authorized=True,post_trial_confirmation_questions_waived=True,
            observation_reference=self.fixture.auth['current_conditions_source'],
            post_trial_physical_observation_inferred=False,physical_anomaly_after_new_trials=None,
            physical_audio_heard_after_new_trials=None,box_removal_allowed=False,
            load_transfer_allowed=False,standing_allowed=False,walking_allowed=False)
        self.context_ref=self.fixture.write(self.base/'native-context.json',self.context)

    def args(self,duration,profile,permission,capture,report=None,receipt=None,diagnostic=None):
        values=['--duration',str(duration),'--kit-runtime',str(Path(tool.live.__file__).resolve().parents[1]),
                '--output',str(self.base.resolve()/('prepared-native-'+str(duration)))]
        for name,ref in (('profile',profile),('preauthorization',permission),('current-capture',capture),
                        ('source-manifest',self.fixture.auth['source_manifest']),('prior-report',report),
                        ('prior-execution-receipt',receipt),('pipeline-diagnostic',diagnostic)):
            if ref:values.extend(['--'+name,ref['path'],'--'+name+'-sha256',ref['sha256']])
        return tool.parser().parse_args(values)

    def base_args(self):
        data,docs,directory=self.fixture.nodes[0]
        raw=(directory/'profile.json').read_bytes()
        profile=dict(path=str(directory/'profile.json'),sha256=hashlib.sha256(raw).hexdigest())
        return self.args(2,profile,self.context_ref,data['artifacts']['boxed_sequence_current_capture'])

    def extended_args(self,duration,prior_result):
        data,docs,directory=self.fixture.nodes[1 if duration==10 else 2]
        report=copy.deepcopy(docs['prior_supported_report'])
        report['profile_sha256']=prior_result['profile']['sha256']
        receipt=json.loads(Path(docs['prior_supported_observation']['execution_receipt']['path']).read_text())
        receipt['profile_sha256']=prior_result['profile']['sha256']
        report_ref=self.fixture.write(self.base/('native-actual-'+str(duration)+'.json'),report)
        receipt['report_sha256']=report_ref['sha256']
        receipt_ref=self.fixture.write(self.base/('native-receipt-'+str(duration)+'.json'),receipt)
        return self.args(duration,prior_result['profile'],prior_result['preauthorization'],
            data['artifacts']['boxed_sequence_current_capture'],report_ref,receipt_ref,
            data['artifacts']['pipeline_diagnostic'] if duration==20 else None)

    def test_default_plan_never_writes_or_claims_hardware_success(self):
        args=self.base_args()
        with patch.object(tool,'write') as writer,patch.object(Path,'mkdir') as mkdir:
            plan=tool.prepare(args)
        writer.assert_not_called();mkdir.assert_not_called()
        self.assertEqual(plan['status'],'PLAN_ONLY');self.assertFalse(plan['output_allowed'])
        self.assertFalse(plan['hardware_opened']);self.assertFalse(Path(args.output).exists())

    def test_real_original_toy_records_prepare_full_native_chain_unknown_physical_results(self):
        args=self.base_args();args.prepare=True;two=tool.prepare(args)
        args=self.extended_args(10,two);args.prepare=True;ten=tool.prepare(args)
        args=self.extended_args(20,ten);args.prepare=True;twenty=tool.prepare(args)
        for seconds,result in ((2,two),(10,ten),(20,twenty)):
            self.assertEqual(result['status'],'PREPARED_PREAUTHORIZED_BOXED_STAGE')
            self.assertTrue(result['profile_loader_accepted']);self.assertFalse(result['output_allowed'])
            loaded=tool.live.load_profile(result['profile']['path'])
            self.assertTrue(tool.live.native_phase_pair_settings(loaded))
            self.assertEqual(loaded['duration_s'],seconds)
            self.assertEqual(loaded['_post_trial_physical_observation'],
                dict(audio_heard=None,anomalies=None,support_maintained=None,observed=False))
        self.assertEqual(two['preauthorization'],ten['preauthorization'])
        self.assertEqual(two['preauthorization'],twenty['preauthorization'])
        self.assertEqual(two['sequence_contract_sha256'],twenty['sequence_contract_sha256'])

    def test_twenty_cannot_omit_fresh_post_ten_diagnostic_or_skip_predecessor(self):
        args=self.base_args();args.prepare=True;two=tool.prepare(args)
        args=self.extended_args(20,two);args.prepare=True
        with self.assertRaises(tool.live.ProfileError):tool.prepare(args)
        args=self.extended_args(10,two);args.prepare=True;ten=tool.prepare(args)
        args=self.extended_args(20,ten);args.pipeline_diagnostic=None;args.pipeline_diagnostic_sha256=None;args.prepare=True
        with self.assertRaisesRegex(tool.live.ProfileError,'fresh post-ten-second diagnostic'):tool.prepare(args)

    def test_context_scope_future_facts_and_input_pins_cannot_be_changed(self):
        for change in (lambda c:c.update(physical_audio_heard_after_new_trials=True),
                       lambda c:c.update(post_trial_physical_observation_inferred=True),
                       lambda c:c.update(authorized_durations_s=[2,10,30]),
                       lambda c:c.update(request_gap_us=900),lambda c:c.update(load_transfer_allowed=True),
                       lambda c:c.update(native_phase_pair=False),
                       lambda c:c.update(automatic_continuation_explicitly_authorized=False),
                       lambda c:c.update(post_trial_confirmation_questions_waived=False)):
            context=copy.deepcopy(self.context);change(context)
            ref=self.fixture.write(self.base/'bad-native-context.json',context)
            args=self.base_args();args.preauthorization=ref['path'];args.preauthorization_sha256=ref['sha256']
            with self.subTest(change=change),self.assertRaises(tool.live.ProfileError):tool.prepare(args)
        for name in ('profile_sha256','source_manifest_sha256','current_capture_sha256'):
            args=self.base_args();setattr(args,name,'f'*64)
            with self.subTest(name=name),self.assertRaises(tool.live.ProfileError):tool.prepare(args)

    def test_aborted_predecessor_and_failed_restore_do_not_publish_stage(self):
        args=self.base_args();args.prepare=True;two=tool.prepare(args)
        args=self.extended_args(10,two)
        report=json.loads(Path(args.prior_report).read_text());report['status']='ABORTED'
        ref=self.fixture.write(self.base/'aborted-native.json',report)
        args.prior_report=ref['path'];args.prior_report_sha256=ref['sha256'];args.prepare=True
        with self.assertRaises(tool.live.ProfileError):tool.prepare(args)
        self.assertFalse(Path(args.output).exists())
        args=self.extended_args(10,two)
        receipt=json.loads(Path(args.prior_execution_receipt).read_text());receipt['source_files_unchanged']=False
        ref=self.fixture.write(self.base/'failed-restore-native.json',receipt)
        args.prior_execution_receipt=ref['path'];args.prior_execution_receipt_sha256=ref['sha256'];args.prepare=True
        with self.assertRaises(tool.live.ProfileError):tool.prepare(args)
        self.assertFalse(Path(args.output).exists())

    def test_unconfirmed_current_clearance_cannot_be_created_by_preparation(self):
        original_ref=self.context['observation_reference']
        original_bytes=Path(original_ref['path']).read_bytes()
        conditions=json.loads(original_bytes)
        conditions['current_conditions']['all12_local_plus_minus3deg_clear']=False
        conditions_ref=self.fixture.write(self.base/'unconfirmed-physical-source.json',conditions)
        context=copy.deepcopy(self.context);context['observation_reference']=conditions_ref
        context_ref=self.fixture.write(self.base/'unconfirmed-context.json',context)
        args=self.base_args();args.prepare=True
        args.preauthorization=context_ref['path'];args.preauthorization_sha256=context_ref['sha256']
        with self.assertRaisesRegex(tool.live.ProfileError,'Direct current boxed conditions incomplete'):
            tool.prepare(args)
        self.assertFalse(Path(args.output).exists())
        self.assertEqual(Path(original_ref['path']).read_bytes(),original_bytes)

    def test_fresh_output_and_full_loader_are_mandatory_preserving_originals(self):
        args=self.base_args();args.prepare=True
        baseline=Path(args.profile).read_bytes()
        with patch.object(tool.live,'load_profile',side_effect=tool.live.ProfileError('Full-loader rejection')):
            with self.assertRaises(tool.live.ProfileError):tool.prepare(args)
        self.assertFalse(Path(args.output).exists());self.assertEqual(Path(args.profile).read_bytes(),baseline)
        result=tool.prepare(args)
        self.assertTrue(Path(result['profile']['path']).exists())
        with self.assertRaisesRegex(tool.live.ProfileError,'Fresh private'):tool.prepare(args)

    def test_entry_refuses_bytecode_enabled_before_runtime_imports(self):
        result=subprocess.run([sys.executable,'-E',str(Path(tool.__file__).resolve()),'--help'],capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('Launch the file-only entry with python -B',result.stderr)


class NativeV1PreparationTests(NativePreparationTests):
    FIXTURE_CLASS=fixture.NativeBoxedV1PreauthorizationTests


if __name__=='__main__':unittest.main()
