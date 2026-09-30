import importlib.util
import json
import hashlib
import shlex
from pathlib import Path
import tempfile
import unittest
import contextlib
import io
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('dog_tomorrow',Path(__file__).with_name('dog_tomorrow.py'))
dog=importlib.util.module_from_spec(spec);spec.loader.exec_module(dog)

class TomorrowTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.config=self.root/'config.json'
        self.config.write_text(json.dumps({'front_port':'/dev/serial/by-path/front',
            'rear_port':'/dev/serial/by-path/rear','expected_uids':'uids.json','bundle':'policy',
            'angle_profile':'angles.json','calibration':'calibration.json','mount':'mount.json'}))
        self.work=self.root/'work'
    def args(self,action):return [action,'--config',str(self.config),'--work-dir',str(self.work)]
    def test_all_plans_open_no_process_or_workdir(self):
        with patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')):
            for action in ('build','imu','capture','can','full','compare','diagnostics'):
                self.assertEqual(dog.main(self.args(action)),0)
        self.assertFalse(self.work.exists())

    def assert_request_pacing(self,command,window,gap_us):
        for flag,value in (('--request-window',window),('--request-gap-us',gap_us)):
            self.assertEqual(command.count(flag),1)
            self.assertEqual(command[command.index(flag)+1],str(value))

    def test_native_plans_forward_default_and_custom_request_pacing(self):
        settings=(((),3,600),
                  (('--request-window','1','--request-gap-us','5000'),1,5000),
                  (('--request-window','2','--request-gap-us','1000'),2,1000))
        with patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')) as run:
            for action in ('can','compare','full','diagnostics'):
                for extra,window,gap_us in settings:
                    with self.subTest(action=action,window=window,gap_us=gap_us):
                        stream=io.StringIO()
                        with contextlib.redirect_stdout(stream):
                            self.assertEqual(dog.main(self.args(action)+list(extra)),0)
                        plan=json.loads(stream.getvalue())
                        self.assertEqual((plan['request_window'],plan['request_gap_us']),(window,gap_us))
                        commands=[shlex.split(command) for command in plan['commands']]
                        native_commands=[command for command in commands
                                         if 'singularitydog_hw.native_pipeline_benchmark' in command]
                        self.assertEqual(len(native_commands),3 if action=='diagnostics' else 1)
                        for command in native_commands:self.assert_request_pacing(command,window,gap_us)
                        if action=='diagnostics':
                            self.assertEqual([stage['name'] for stage in plan['stages']],
                                             ['capture','can','compare','full'])
                            for stage in plan['stages'][1:]:
                                self.assertEqual((stage['request_window'],stage['request_gap_us']),(window,gap_us))
                            self.assertNotIn('request_window',plan['stages'][0])
                            self.assertNotIn('request_gap_us',plan['stages'][0])
                            for command in commands[:2]:
                                self.assertNotIn('--request-window',command)
                                self.assertNotIn('--request-gap-us',command)
            run.assert_not_called()
        self.assertFalse(self.work.exists())

    def test_unrelated_plans_ignore_native_request_pacing(self):
        with patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')) as run:
            for action in ('build','imu','capture'):
                for extra in ((),('--request-window','1','--request-gap-us','5000')):
                    with self.subTest(action=action,extra=extra):
                        stream=io.StringIO()
                        with contextlib.redirect_stdout(stream):
                            self.assertEqual(dog.main(self.args(action)+list(extra)),0)
                        plan=json.loads(stream.getvalue())
                        self.assertNotIn('request_window',plan)
                        self.assertNotIn('request_gap_us',plan)
                        for raw in plan['commands']:
                            command=shlex.split(raw)
                            self.assertNotIn('--request-window',command)
                            self.assertNotIn('--request-gap-us',command)
            run.assert_not_called()
        self.assertFalse(self.work.exists())

    def test_invalid_request_pacing_rejected_before_config_workdir_or_process(self):
        missing=self.root/'missing-config.json'
        invalid=(('--request-window','0'),('--request-window','4'),('--request-window','not-an-int'),
                 ('--request-gap-us','599'),('--request-gap-us','5001'),('--request-gap-us','not-an-int'))
        with patch.object(dog.Path,'read_text',side_effect=AssertionError('Config read')) as read,\
             patch.object(dog.Path,'mkdir',side_effect=AssertionError('Work directory created')) as mkdir,\
             patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')) as run,\
             contextlib.redirect_stderr(io.StringIO()):
            for action in ('can','compare','full','diagnostics'):
                for flag,value in invalid:
                    with self.subTest(action=action,flag=flag,value=value):
                        args=[action,'--config',str(missing),'--work-dir',str(self.work),
                              '--execute','--supported-disabled',flag,value]
                        with self.assertRaises(SystemExit) as raised:dog.main(args)
                        self.assertEqual(raised.exception.code,2)
            read.assert_not_called();mkdir.assert_not_called();run.assert_not_called()
        self.assertFalse(self.work.exists())
    def test_full_requires_support_and_successful_preparation(self):
        with patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')):
            with self.assertRaises(SystemExit):dog.main(self.args('full')+['--execute'])
            with self.assertRaises(SystemExit):dog.main(self.args('full')+['--execute','--supported-disabled'])
    def test_imu_requires_power_off_before_process(self):
        with patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')):
            with self.assertRaises(SystemExit):dog.main(self.args('imu')+['--execute'])
    def test_failed_sequence_does_not_publish_state(self):
        with patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=RuntimeError('injected capture failure')):
            with self.assertRaises(RuntimeError):dog.main(self.args('capture')+['--execute'])
        self.assertFalse((self.work/'state.json').exists())

    def test_manifest_rejects_changed_or_unlisted_source(self):
        code=self.root/'tools'/'main.py';code.parent.mkdir();code.write_text('pass\n')
        manifest={'schema':'private-overnight-kit-v1','files':{'tools/main.py':hashlib.sha256(code.read_bytes()).hexdigest()}}
        (self.root/'kit-manifest.json').write_text(json.dumps(manifest))
        dog.verify_kit(self.root)
        code.write_text('changed\n')
        with self.assertRaisesRegex(ValueError,'changed'):dog.verify_kit(self.root)
        code.write_text('pass\n');(code.parent/'extra.py').write_text('pass\n')
        with self.assertRaisesRegex(ValueError,'Unlisted'):dog.verify_kit(self.root)

    def test_kit_manifest_rejects_symlinked_parent_and_unlisted_symlink_tree(self):
        external=self.root/'external';external.mkdir();code=external/'main.py';code.write_text('pass\n')
        link=self.root/'tools';link.symlink_to(external,target_is_directory=True)
        value={'schema':'private-overnight-kit-v1','files':{'tools/main.py':hashlib.sha256(code.read_bytes()).hexdigest()}}
        path=self.root/'kit-manifest.json';path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError,'symbolic links'):dog.verify_kit(self.root)
        link.unlink();link.mkdir();(link/'nested').symlink_to(external,target_is_directory=True)
        value['files']={'external/main.py':hashlib.sha256(code.read_bytes()).hexdigest()};path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError,'Symlinked'):dog.verify_kit(self.root)

    def test_kit_manifest_rejects_path_aliases_bad_shape_and_duplicate_keys(self):
        path=self.root/'kit-manifest.json'
        for value in ([],{'schema':'private-overnight-kit-v1','files':['file']},
                      {'schema':'private-overnight-kit-v1','files':{'tools//main.py':'a'*64}},
                      {'schema':'private-overnight-kit-v1','files':{'tools/main.py':None}}):
            with self.subTest(value=value):
                path.write_text(json.dumps(value))
                with self.assertRaises(ValueError):dog.verify_kit(self.root)
        path.write_text('{"schema":"bad","schema":"private-overnight-kit-v1","files":{}}')
        with self.assertRaisesRegex(ValueError,'Duplicate'):dog.verify_kit(self.root)

    def test_changed_capture_refuses_before_process(self):
        self.work.mkdir();candidate=self.work/'candidate.json';capture=self.work/'capture.json'
        capture.write_text('{}');candidate.write_text(json.dumps({'source_capture_sha256':hashlib.sha256(b'old').hexdigest()}))
        (self.work/'state.json').write_text(json.dumps({'native_policy_manifest':'unused',
            'calibration':str(candidate),'angle_capture':str(capture),
            'calibration_sha256':hashlib.sha256(candidate.read_bytes()).hexdigest()}))
        with patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')):
            with self.assertRaisesRegex(ValueError,'capture changed'):
                dog.main(self.args('full')+['--execute','--supported-disabled'])

    def active_config(self):
        value=json.loads(self.config.read_text())
        value['supported_policy_output']={
            'active_transport_build':'runtime/experiments/native_active_transport/build.py',
            'active_transport_library':'runtime/experiments/native_active_transport/libdog_active_transport.so',
            'build_active_library_on_target_required':True}
        self.config.write_text(json.dumps(value))

    def fake_build(self,command,**kwargs):
        if str(dog.ACTIVE_DIR/'build.py') in command[1]:
            path=self.root/dog.ACTIVE_DIR;path.mkdir(parents=True,exist_ok=True)
            (path/'transport.cpp').write_bytes(b'fixture-source')
            (path/'libdog_active_transport.so').write_bytes(b'fixture-not-a-real-library')
            (path/'build-record.json').write_text(json.dumps({'abi':1,
                'source_sha256':hashlib.sha256(b'fixture-source').hexdigest(),
                'binary_sha256':hashlib.sha256(b'fixture-not-a-real-library').hexdigest()}))
        elif str(dog.DIAGNOSTIC_DIR/'build.py') in command[1]:
            path=self.root/dog.DIAGNOSTIC_DIR;path.mkdir(parents=True,exist_ok=True)
            (path/'libdog_transport.so').write_bytes(b'fixture-diagnostic-not-a-real-library')
            (path/'build-record.json').write_text(json.dumps({'fixture':'diagnostic-build'}))
        elif 'native_policy_overnight' in command:
            out=Path(command[command.index('--output')+1]);out.mkdir(parents=True)
            raw=b'{"fixture":"synthetic-build-only"}';(out/'manifest.json').write_bytes(raw)
            (out/'build-report.json').write_text(json.dumps({'manifest_sha256':hashlib.sha256(raw).hexdigest()}))

    def artifact_paths(self):
        return [self.root/directory/name for directory,names in (
            (dog.DIAGNOSTIC_DIR,('libdog_transport.so','build-record.json')),
            (dog.ACTIVE_DIR,('libdog_active_transport.so','build-record.json')))
            for name in names]

    def seed_artifacts(self):
        saved={}
        for index,path in enumerate(self.artifact_paths()):
            path.parent.mkdir(parents=True,exist_ok=True)
            raw=('previous-artifact-'+str(index)).encode();mode=(0o600,0o640,0o750,0o660)[index]
            path.write_bytes(raw);path.chmod(mode);saved[path]=(raw,mode)
        return saved

    def assert_artifacts_restored(self,saved):
        for path,(raw,mode) in saved.items():
            self.assertFalse(path.is_symlink())
            self.assertEqual(path.read_bytes(),raw)
            self.assertEqual(path.stat().st_mode & 0o7777,mode)

    def test_new_kit_plan_includes_all_three_builds_without_processes(self):
        self.active_config();out=io.StringIO()
        with patch.object(dog,'ROOT',self.root),patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')),contextlib.redirect_stdout(out):
            dog.main(self.args('build'))
        plan=json.loads(out.getvalue());self.assertEqual(len(plan['commands']),3)
        self.assertIn('native_active_transport/build.py',plan['commands'][1])
        self.assertFalse(plan['motor_enable_available']);self.assertFalse(self.work.exists())

    def test_combined_build_publishes_hashes_only_after_all_success(self):
        self.active_config();previous=self.seed_artifacts()
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=self.fake_build) as run:
            dog.main(self.args('build')+['--execute'])
        state=json.loads((self.work/'state.json').read_text())
        self.assertEqual(run.call_count,3)
        for key in ('native_policy_manifest','active_transport_library','active_transport_build_record'):
            self.assertEqual(state[key+'_sha256'],hashlib.sha256(Path(state[key]).read_bytes()).hexdigest())
        for path,(raw,_) in previous.items():self.assertNotEqual(path.read_bytes(),raw)

    def test_late_model_failure_restores_all_artifact_bytes_modes_and_state(self):
        self.active_config();saved=self.seed_artifacts();self.work.mkdir()
        previous=b'{"previous":"successful-state"}\n';(self.work/'state.json').write_bytes(previous)
        def failure(command,**kwargs):
            if 'native_policy_overnight' in command:
                out=Path(command[command.index('--output')+1]);out.mkdir()
                (out/'build-report.json').write_text('{"status":"FAILED"}')
                raise RuntimeError('model compile failure')
            self.fake_build(command,**kwargs)
            for path in self.artifact_paths():path.chmod(0o644)
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=failure) as run:
            with self.assertRaisesRegex(RuntimeError,'model compile failure'):
                dog.main(self.args('build')+['--execute'])
        self.assertEqual(run.call_count,3);self.assert_artifacts_restored(saved)
        self.assertEqual((self.work/'state.json').read_bytes(),previous)
        self.assertEqual(len(list(self.work.glob('native-policy-*/build-report.json'))),1)

    def test_failed_first_build_removes_new_canonical_artifacts(self):
        self.active_config()
        def failure(command,**kwargs):
            if 'native_policy_overnight' in command:raise RuntimeError('model failure')
            self.fake_build(command,**kwargs)
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=failure):
            with self.assertRaisesRegex(RuntimeError,'model failure'):dog.main(self.args('build')+['--execute'])
        self.assertFalse((self.work/'state.json').exists())
        for path in self.artifact_paths():self.assertFalse(path.exists())

    def test_state_publication_failure_restores_artifacts(self):
        self.active_config();saved=self.seed_artifacts();self.work.mkdir()
        previous=b'{"previous":"intact"}';(self.work/'state.json').write_bytes(previous)
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=self.fake_build),patch.object(dog,'write_state',side_effect=OSError('state failure')):
            with self.assertRaisesRegex(OSError,'state failure'):dog.main(self.args('build')+['--execute'])
        self.assert_artifacts_restored(saved)
        self.assertEqual((self.work/'state.json').read_bytes(),previous)

    def test_exception_after_state_replacement_also_restores_previous_state(self):
        self.active_config();saved=self.seed_artifacts();self.work.mkdir()
        previous=b'{"previous":"intact"}';state_path=self.work/'state.json'
        state_path.write_bytes(previous);state_path.chmod(0o640)
        original_write=dog.write_state
        def failure(path,data):
            original_write(path,data)
            raise OSError('interrupted publication')
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=self.fake_build),patch.object(dog,'write_state',side_effect=failure):
            with self.assertRaisesRegex(OSError,'interrupted publication'):dog.main(self.args('build')+['--execute'])
        self.assert_artifacts_restored(saved)
        self.assertEqual(state_path.read_bytes(),previous);self.assertEqual(state_path.stat().st_mode & 0o7777,0o640)

    def test_failed_state_serialization_does_not_leave_blocking_temporary(self):
        state_path=self.root/'state.json';state_path.write_bytes(b'previous')
        with self.assertRaises(TypeError):dog.write_state(state_path,{'invalid':object()})
        self.assertFalse(state_path.with_suffix('.new').exists());self.assertEqual(state_path.read_bytes(),b'previous')

    def test_first_compiler_partial_failure_restores_previous_artifacts(self):
        self.active_config();saved=self.seed_artifacts()
        def failure(command,**kwargs):
            path=self.artifact_paths()[0];path.write_bytes(b'partial-object');path.chmod(0o644)
            raise RuntimeError('diagnostic compiler failure')
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=failure) as run:
            with self.assertRaisesRegex(RuntimeError,'diagnostic compiler failure'):dog.main(self.args('build')+['--execute'])
        self.assertEqual(run.call_count,1);self.assert_artifacts_restored(saved)

    def test_existing_artifact_symlink_rejected_before_builder(self):
        self.active_config();target=self.root/'outside-artifact';target.write_bytes(b'untouched')
        path=self.artifact_paths()[0];path.parent.mkdir(parents=True);path.symlink_to(target)
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=AssertionError('Builder started')):
            with self.assertRaisesRegex(ValueError,'safely snapshot'):dog.main(self.args('build')+['--execute'])
        self.assertTrue(path.is_symlink());self.assertEqual(target.read_bytes(),b'untouched')

    def test_artifact_parent_symlink_rejected_before_builder(self):
        self.active_config();target=self.root/'outside-directory';target.mkdir()
        path=self.root/dog.DIAGNOSTIC_DIR;path.parent.mkdir(parents=True);path.symlink_to(target,target_is_directory=True)
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=AssertionError('Builder started')):
            with self.assertRaisesRegex(ValueError,'artifact parent'):dog.main(self.args('build')+['--execute'])
        self.assertEqual(list(target.iterdir()),[])

    def test_rollback_replaces_changed_symlink_without_touching_target(self):
        self.active_config();saved=self.seed_artifacts();target=self.root/'outside-artifact';target.write_bytes(b'untouched')
        def failure(command,**kwargs):
            if 'native_policy_overnight' in command:
                path=self.artifact_paths()[2];path.unlink();path.symlink_to(target)
                raise RuntimeError('late failure with replaced artifact')
            self.fake_build(command,**kwargs)
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=failure):
            with self.assertRaisesRegex(RuntimeError,'late failure'):dog.main(self.args('build')+['--execute'])
        self.assert_artifacts_restored(saved);self.assertEqual(target.read_bytes(),b'untouched')

    def test_legacy_diagnostic_build_does_not_modify_active_artifacts(self):
        saved=self.seed_artifacts()
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=self.fake_build) as run:
            dog.main(self.args('build')+['--execute'])
        self.assertEqual(run.call_count,2)
        self.assert_artifacts_restored({path:value for path,value in saved.items() if path.parent==self.root/dog.ACTIVE_DIR})
        self.assertNotIn('active_transport_library',json.loads((self.work/'state.json').read_text()))

    def test_failed_active_build_preserves_previous_state_and_does_not_build_model(self):
        self.active_config();self.work.mkdir();old=b'{"previous":"intact"}'
        (self.work/'state.json').write_bytes(old)
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=[None,RuntimeError('compile failure')]) as run:
            with self.assertRaisesRegex(RuntimeError,'compile failure'):dog.main(self.args('build')+['--execute'])
        self.assertEqual(run.call_count,2);self.assertEqual((self.work/'state.json').read_bytes(),old)

    def test_mismatched_build_record_does_not_publish_success(self):
        self.active_config();saved=self.seed_artifacts()
        def corrupted(command,**kwargs):
            self.fake_build(command,**kwargs)
            if 'native_policy_overnight' in command:
                (self.root/dog.ACTIVE_DIR/'libdog_active_transport.so').write_bytes(b'changed')
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),patch.object(dog.subprocess,'run',side_effect=corrupted):
            with self.assertRaisesRegex(ValueError,'mismatch'):dog.main(self.args('build')+['--execute'])
        self.assertFalse((self.work/'state.json').exists())
        self.assert_artifacts_restored(saved)

    def test_build_path_cannot_be_redirected_by_config(self):
        self.active_config();value=json.loads(self.config.read_text())
        value['supported_policy_output']['active_transport_build']='some-other-program.py'
        self.config.write_text(json.dumps(value))
        with patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')):
            with self.assertRaisesRegex(ValueError,'declaration'):dog.main(self.args('build')+['--execute'])

    def diagnostic_state(self):
        self.work.mkdir(exist_ok=True)
        manifest=self.work/'model-manifest.json'
        manifest.write_text(json.dumps({'schema':'native-policy-overnight-v1','status':'VALIDATED_FILE_ONLY'}))
        state={'native_policy_manifest':str(manifest),
            'native_policy_manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest(),
            'calibration':'OLD_CAPTURE_MUST_NOT_BE_USED','angle_capture':'OLD_CAPTURE_MUST_NOT_BE_USED',
            'last_diagnostics':{'status':'COMPLETE_DIAGNOSTICS','summary':'old-success'}}
        (self.work/'state.json').write_text(json.dumps(state))
        return state

    def diagnostic_child(self,command,**kwargs):
        """Synthetic files only: this fixture never imports/open serial or torch."""
        out=Path(command[command.index('--output')+1]);boot='00000000-0000-0000-0000-000000000001'
        if 'singularitydog_hw.motor_epoch_readonly_capture' in command:
            out.write_text(json.dumps({'status':'RECORDED_REVIEW_REQUIRED','errors':[],
                'motor_output_allowed':False,'approved_for_runtime':False,'boot_id':boot}))
        elif command[1].endswith('/audit_angle_calibration.py'):
            capture=Path(command[command.index('--capture')+1]);sha=hashlib.sha256(capture.read_bytes()).hexdigest()
            candidate=Path(command[command.index('--policy-candidate-output')+1])
            candidate.write_text(json.dumps({'status':'MANUAL_NOMINAL_CANDIDATES_ONLY',
                'source_capture_sha256':sha,'source_current_boot_id':boot}))
            out.write_text(json.dumps({'status':'INCOMPLETE_PHYSICAL_EVIDENCE',
                'current_capture_sha256':sha,'current_boot_id':boot}))
        else:
            out.mkdir()
            cycles=int(command[command.index('--cycles')+1])
            report={'status':'COMPLETE_DIAGNOSTIC','errors':[],'boot_id':boot,'cycles_completed':cycles,
                'motor_enable_sent':False,'learned_targets_sent':False,'approved_for_runtime':False,
                'full_controller_50Hz_verified':False,'mode':command[command.index('--mode')+1],
                'distributions_ms':{'whole_iteration_ms':{'median':18.,'max':22.}}}
            if '--compare-feedback' in command:
                report.update(kind='native_feedback_comparison_report',phase_timings=[{'synthetic_ms':1.}],
                    per_motor={str(i):{'all_direct_static_comparisons_agree':True} for i in range(1,13)})
            elif '--acquisition-only' not in command:
                manifest=Path(command[command.index('--native-policy-manifest')+1])
                calibration=Path(command[command.index('--calibration')+1])
                report.update(model_source={'manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest()},
                    input_sha256={'calibration':hashlib.sha256(calibration.read_bytes()).hexdigest()},
                    observer={'status':'COMPLETE_NO_OUTPUT_DIAGNOSTIC','ticks_completed':cycles,
                              'ticks_requested':cycles,'failure':None,'incomplete':False,'output_allowed':False})
            (out/'report.json').write_text(json.dumps(report))
        return dog.subprocess.CompletedProcess(command,0)

    def run_diagnostics(self,runner=None,*,extra_args=()):
        stream=io.StringIO()
        with patch.object(dog,'ROOT',self.root),patch.object(dog,'verify_kit'),\
             patch.object(dog.subprocess,'run',side_effect=runner or self.diagnostic_child) as run,\
             contextlib.redirect_stdout(stream):
            status=dog.main(self.args('diagnostics')+['--execute','--supported-disabled']+list(extra_args))
        state=json.loads((self.work/'state.json').read_text())
        summary=json.loads(Path(state['last_diagnostics']['summary']).read_text())
        return status,state,summary,run

    def test_diagnostics_plan_lists_whole_sequence_without_opening_work_or_process(self):
        stream=io.StringIO()
        with patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')),contextlib.redirect_stdout(stream):
            self.assertEqual(dog.main(self.args('diagnostics')),0)
        plan=json.loads(stream.getvalue())
        self.assertEqual(plan['status'],'PLAN')
        self.assertEqual([s['name'] for s in plan['stages']],['capture','can','compare','full'])
        self.assertEqual(len(plan['commands']),5)
        self.assertEqual(plan['stages'][2]['cycles'],3)
        self.assertEqual(plan['stages'][3]['cycles'],20)
        self.assertFalse(self.work.exists())

    def test_diagnostics_needs_explicit_disabled_support_and_verified_build(self):
        with patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')):
            with self.assertRaises(SystemExit):dog.main(self.args('diagnostics')+['--execute'])
        status,state,summary,run=self.run_diagnostics()
        self.assertEqual(status,2);self.assertEqual(run.call_count,0)
        self.assertEqual(summary['failed_stage'],'preparation')
        self.assertEqual(state['last_diagnostics']['status'],'ABORTED')
        self.assertTrue(all(s['status']=='NOT_RUN' for s in summary['stages']))

    def test_diagnostics_promotes_only_its_new_complete_capture(self):
        self.diagnostic_state();status,state,summary,run=self.run_diagnostics()
        self.assertEqual(status,0);self.assertEqual(run.call_count,5)
        self.assertEqual(summary['status'],'COMPLETE_DIAGNOSTICS')
        self.assertTrue(summary['fresh_capture_promoted'])
        self.assertTrue(all(s['status']=='COMPLETE' for s in summary['stages']))
        self.assertEqual(Path(state['calibration']).parent,Path(summary['output']))
        self.assertEqual(state['calibration_sha256'],hashlib.sha256(Path(state['calibration']).read_bytes()).hexdigest())
        commands=[call.args[0] for call in run.call_args_list]
        self.assertEqual(commands[-1][commands[-1].index('--calibration')+1],state['calibration'])
        self.assertTrue(all('OLD_CAPTURE_MUST_NOT_BE_USED' not in command for command in commands))
        self.assertTrue(all('singularitydog_hw.policy_output' not in command for command in commands))
        self.assertFalse(summary['approved_for_runtime']);self.assertFalse(summary['automatic_retry'])

    def test_diagnostics_custom_request_pacing_reaches_all_native_children_and_saved_stages(self):
        self.diagnostic_state()
        status,state,summary,run=self.run_diagnostics(
            extra_args=('--request-window','2','--request-gap-us','1000'))
        self.assertEqual(status,0);self.assertEqual(run.call_count,5)
        self.assertEqual(summary['status'],'COMPLETE_DIAGNOSTICS')
        self.assertEqual(state['last_diagnostics']['status'],'COMPLETE_DIAGNOSTICS')
        self.assertTrue(summary['fresh_capture_promoted'])
        self.assertTrue(all(stage['status']=='COMPLETE' for stage in summary['stages']))
        self.assertEqual(Path(state['calibration']).parent,Path(summary['output']))
        commands=[call.args[0] for call in run.call_args_list]
        for command in commands[:2]:
            self.assertNotIn('singularitydog_hw.native_pipeline_benchmark',command)
            self.assertNotIn('--request-window',command)
            self.assertNotIn('--request-gap-us',command)
        for command in commands[2:]:
            self.assertIn('singularitydog_hw.native_pipeline_benchmark',command)
            self.assert_request_pacing(command,2,1000)
        self.assertEqual([stage['name'] for stage in summary['stages']],['capture','can','compare','full'])
        self.assertNotIn('request_window',summary['stages'][0])
        self.assertNotIn('request_gap_us',summary['stages'][0])
        for stage in summary['stages'][1:]:
            self.assertEqual((stage['request_window'],stage['request_gap_us']),(2,1000))
        self.assertFalse(summary['automatic_retry']);self.assertFalse(summary['approved_for_runtime'])

    def test_failed_capture_invalidates_old_success_and_never_runs_later_stages(self):
        previous=self.diagnostic_state()
        def fail(command,**kwargs):
            self.diagnostic_child(command,**kwargs)
            out=Path(command[command.index('--output')+1]);out.write_text(json.dumps({'status':'INCOMPLETE','errors':['ID11 no response']}))
            return dog.subprocess.CompletedProcess(command,1)
        status,state,summary,run=self.run_diagnostics(fail)
        self.assertEqual(status,2);self.assertEqual(run.call_count,1)
        self.assertEqual(state['calibration'],previous['calibration'])
        self.assertEqual(state['last_diagnostics']['status'],'ABORTED')
        self.assertEqual(summary['stages'][0]['report_errors'],['ID11 no response'])
        self.assertTrue(all(s['status']=='NOT_RUN' for s in summary['stages'][1:]))

    def test_zero_exit_with_capture_binding_error_stops_before_can(self):
        self.diagnostic_state()
        def wrong(command,**kwargs):
            result=self.diagnostic_child(command,**kwargs)
            if '--policy-candidate-output' in command:
                path=Path(command[command.index('--policy-candidate-output')+1]);value=json.loads(path.read_text())
                value['source_capture_sha256']='f'*64;path.write_text(json.dumps(value))
            return result
        status,state,summary,run=self.run_diagnostics(wrong)
        self.assertEqual(status,2);self.assertEqual(run.call_count,2)
        self.assertIn('does not bind',str(summary['errors']))
        self.assertFalse(state['last_diagnostics']['fresh_capture_promoted'])

    def test_compare_child_failure_is_retained_and_full_never_starts(self):
        self.diagnostic_state()
        def fail(command,**kwargs):
            result=self.diagnostic_child(command,**kwargs)
            if '--compare-feedback' in command:
                path=Path(command[command.index('--output')+1])/'report.json';value=json.loads(path.read_text())
                value.update(status='ABORTED',errors=['Rear ID10 reply deadline']);path.write_text(json.dumps(value))
                return dog.subprocess.CompletedProcess(command,2)
            return result
        status,state,summary,run=self.run_diagnostics(fail)
        self.assertEqual(status,2);self.assertEqual(run.call_count,4)
        self.assertEqual(summary['failed_stage'],'compare')
        self.assertEqual(summary['stages'][2]['report_errors'],['Rear ID10 reply deadline'])
        self.assertEqual(summary['stages'][3]['status'],'NOT_RUN')

    def test_static_comparison_disagreement_is_not_silently_promoted_to_full(self):
        self.diagnostic_state()
        def differ(command,**kwargs):
            result=self.diagnostic_child(command,**kwargs)
            if '--compare-feedback' in command:
                path=Path(command[command.index('--output')+1])/'report.json';value=json.loads(path.read_text())
                value['per_motor']['3']['all_direct_static_comparisons_agree']=False;path.write_text(json.dumps(value))
            return result
        status,state,summary,run=self.run_diagnostics(differ)
        self.assertEqual(status,2);self.assertEqual(run.call_count,4)
        self.assertEqual(summary['status'],'DIAGNOSTIC_BLOCKED')
        self.assertEqual(state['last_diagnostics']['blocked_stage'],'compare')
        self.assertIn('inconclusive for IDs 3',str(summary['errors']))
        self.assertFalse(summary['stages'][2]['comparison_by_id']['3']['all_direct_static_comparisons_agree'])
        self.assertEqual(summary['pending_checks'][0]['motor_ids'],[3])
        self.assertFalse(summary['physical_failure_inferred']);self.assertFalse(summary['dynamic_scale_validated'])

    def test_full_failure_preserves_exact_reason_and_does_not_publish_capture(self):
        previous=self.diagnostic_state()
        def fail(command,**kwargs):
            result=self.diagnostic_child(command,**kwargs)
            if '--native-policy-manifest' in command:
                path=Path(command[command.index('--output')+1])/'report.json';value=json.loads(path.read_text())
                value.update(status='ABORTED',errors=['Calibrated position outside registered joint range'])
                path.write_text(json.dumps(value));return dog.subprocess.CompletedProcess(command,2)
            return result
        status,state,summary,run=self.run_diagnostics(fail)
        self.assertEqual(status,2);self.assertEqual(run.call_count,5)
        self.assertEqual(summary['failed_stage'],'full')
        self.assertEqual(summary['stages'][3]['report_errors'],['Calibrated position outside registered joint range'])
        self.assertEqual(state['calibration'],previous['calibration'])

    def test_reboot_between_diagnostics_stops_before_compare(self):
        self.diagnostic_state()
        def reboot(command,**kwargs):
            result=self.diagnostic_child(command,**kwargs)
            if '--mode' in command and command[command.index('--mode')+1]=='type17':
                path=Path(command[command.index('--output')+1])/'report.json';value=json.loads(path.read_text())
                value['boot_id']='different-boot';path.write_text(json.dumps(value))
            return result
        status,state,summary,run=self.run_diagnostics(reboot)
        self.assertEqual(status,2);self.assertEqual(run.call_count,3)
        self.assertIn('boot changed',str(summary['errors']))

    def test_manifest_mutation_after_capture_prevents_can_and_no_automatic_build(self):
        state=self.diagnostic_state()
        def mutate(command,**kwargs):
            result=self.diagnostic_child(command,**kwargs)
            if '--policy-candidate-output' in command:Path(state['native_policy_manifest']).write_text('{}')
            return result
        status,state,summary,run=self.run_diagnostics(mutate)
        self.assertEqual(status,2);self.assertEqual(run.call_count,2)
        self.assertIn('Build manifest changed',str(summary['errors']))
        self.assertTrue(all('build' not in call.args[0] for call in run.call_args_list))

    def test_capture_mutation_during_compare_stops_before_full(self):
        self.diagnostic_state()
        def mutate(command,**kwargs):
            result=self.diagnostic_child(command,**kwargs)
            if '--compare-feedback' in command:
                run_dir=Path(command[command.index('--output')+1]).parent
                candidate=run_dir/'calibration.json';data=json.loads(candidate.read_text())
                data['unreviewed_extra']=True;candidate.write_text(json.dumps(data))
            return result
        status,state,summary,run=self.run_diagnostics(mutate)
        self.assertEqual(status,2);self.assertEqual(run.call_count,4)
        self.assertIn('changed during diagnostic child',str(summary['errors']))
        self.assertEqual(summary['stages'][3]['status'],'NOT_RUN')

    def test_next_explicit_invocation_starts_a_new_capture_not_a_failed_stage_resume(self):
        self.diagnostic_state()
        def fail(command,**kwargs):
            if '--compare-feedback' in command:return dog.subprocess.CompletedProcess(command,2)
            return self.diagnostic_child(command,**kwargs)
        status,_,first,first_run=self.run_diagnostics(fail)
        self.assertEqual(status,2);self.assertEqual(first_run.call_count,4)
        status,state,second,second_run=self.run_diagnostics()
        self.assertEqual(status,0);self.assertEqual(second_run.call_count,5)
        self.assertNotEqual(first['output'],second['output'])
        self.assertIn('singularitydog_hw.motor_epoch_readonly_capture',second_run.call_args_list[0].args[0])
        self.assertEqual(Path(state['calibration']).parent,Path(second['output']))
        self.assertEqual(json.loads(Path(first['summary']).read_text())['status'],'ABORTED')

    def test_final_state_write_failure_does_not_claim_success_or_retry_children(self):
        self.diagnostic_state();original=dog.write_state
        def fail(path,data):
            if path.resolve()==(self.work/'state.json').resolve() and data.get('last_diagnostics',{}).get('status')=='COMPLETE_DIAGNOSTICS':
                raise OSError('Synthetic final publication failure')
            return original(path,data)
        with patch.object(dog,'write_state',side_effect=fail):
            status,state,summary,run=self.run_diagnostics()
        self.assertEqual(status,2);self.assertEqual(run.call_count,5)
        self.assertEqual(summary['status'],'ABORTED')
        self.assertEqual(summary['failed_stage'],'state_publication')
        self.assertFalse(summary['fresh_capture_promoted'])
        self.assertEqual(state['last_diagnostics']['status'],'ABORTED')

    def test_full_zero_exit_with_incomplete_observer_is_not_success(self):
        self.diagnostic_state()
        def incomplete(command,**kwargs):
            result=self.diagnostic_child(command,**kwargs)
            if '--native-policy-manifest' in command:
                path=Path(command[command.index('--output')+1])/'report.json';value=json.loads(path.read_text())
                value['observer']['ticks_completed']=1;path.write_text(json.dumps(value))
            return result
        status,state,summary,run=self.run_diagnostics(incomplete)
        self.assertEqual(status,2);self.assertEqual(run.call_count,5)
        self.assertIn('complete real-inference evidence',str(summary['errors']))
        self.assertFalse(summary['fresh_capture_promoted'])

    def test_concurrent_diagnostics_cannot_overwrite_shared_state_or_start_a_child(self):
        previous=self.diagnostic_state();before=(self.work/'state.json').read_bytes()
        with dog.diagnostics_work_lock(self.work),patch.object(dog,'verify_kit'),\
             patch.object(dog.subprocess,'run',side_effect=AssertionError('Process started')):
            with self.assertRaisesRegex(ValueError,'Another diagnostics sequence'):
                dog.main(self.args('diagnostics')+['--execute','--supported-disabled'])
        self.assertEqual((self.work/'state.json').read_bytes(),before)
        self.assertEqual(list(self.work.glob('diagnostics-*/summary.json')),[])

    def test_unexpected_child_output_scope_cannot_be_hidden_by_summary_flags(self):
        self.diagnostic_state()
        def invalid(command,**kwargs):
            result=self.diagnostic_child(command,**kwargs)
            if '--mode' in command:
                path=Path(command[command.index('--output')+1])/'report.json';value=json.loads(path.read_text())
                value['motor_enable_sent']=True;path.write_text(json.dumps(value))
            return result
        status,_,summary,run=self.run_diagnostics(invalid)
        self.assertEqual(status,2);self.assertEqual(run.call_count,3)
        self.assertTrue(summary['motor_enable_sent'])
        self.assertIn('scope invalid',str(summary['errors']))

if __name__=='__main__':unittest.main()
