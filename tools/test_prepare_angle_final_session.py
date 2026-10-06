"""Synthetic file-only planner/reviewer tests; no motor, serial or SSH calls."""
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('prepare_angle_final_session', Path(__file__).with_name('prepare_angle_final_session.py'))
tool = importlib.util.module_from_spec(spec); spec.loader.exec_module(tool)
SIGNS = (1, 1, -1, -1, -1, -1, 1, 1, 1, -1, -1, 1)


def encoded(value): return json.dumps(value, sort_keys=True, allow_nan=False).encode()
def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def paired(value): return value, hashlib.sha256(encoded(value)).hexdigest()


class FinalSessionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(); self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve(); self.kit = self.root/'frozen-kit'
        (self.kit/'inputs').mkdir(parents=True)
        self.config = {'front_port':'/dev/serial/by-path/front', 'rear_port':'/dev/serial/by-path/rear',
            'expected_uids':'inputs/uids.json', 'angle_profile':'inputs/profile.json',
            'calibration_approved_for_runtime':False}
        self.uids = {str(mid):f'{mid:016x}' for mid in tool.IDS}
        (self.kit/'inputs/uids.json').write_bytes(encoded(self.uids))
        self.uid_sha = digest(self.kit/'inputs/uids.json')
        axes = []
        for mid in tool.IDS:
            calf, thigh = (mid-1)%3 == 0, (mid-1)%3 == 1
            nominal = -math.pi/2 if calf else 0.
            axes.append({'motor_id':mid, 'uid':self.uids[str(mid)], 'sign':SIGNS[mid-1],
                'offset_rad':nominal-SIGNS[mid-1]*.1, 'lower_rad':-2.2 if calf else -.9 if thigh else -.5,
                'upper_rad':-.08 if calf else 1.2 if thigh else .5, 'uncertainty_rad':0.,
                'zero_reviewed':False, 'direction_reviewed':False, 'physical_limits_reviewed':False,
                'zero_evidence_sha256':None, 'direction_evidence_sha256':None, 'physical_limits_evidence_sha256':None})
        self.profile = {'schema':'singularitydog.angle-calibration-review-profile.v1', 'assembly_revision':'synthetic',
            'axes':axes, 'evidence_files':{}, 'physical_uncertainty_known':False,
            'approved_for_runtime':False, 'motor_output_available':False}
        self.profile_path = self.kit/'inputs/profile.json'; self.profile_path.write_bytes(encoded(self.profile))
        (self.kit/'kit-config.json').write_bytes(encoded(self.config))
        files = {str(p.relative_to(self.kit)):digest(p) for p in self.kit.rglob('*') if p.is_file()}
        self.manifest = self.kit/'kit-manifest.json'
        self.manifest.write_bytes(encoded({'schema':'private-overnight-kit-v1','files':files}))
        self.seed = self.capture(0); self.seed_path = self.root/'seed.json'; self.seed_path.write_bytes(encoded(self.seed))

    def capture(self, second, changes=None):
        result = {'schema':tool.CAPTURE_SCHEMA, 'status':'RECORDED_REVIEW_REQUIRED', 'errors':[],
            'boot_id':'synthetic-current-boot', 'motor_power_epoch':'NOT_INFERRED_FROM_JETSON_BOOT',
            'started_at':f'2026-10-05T09:00:{second:02d}+09:00',
            'completed_at':f'2026-10-05T09:00:{second:02d}.500000+09:00',
            'expected_uids_sha256':self.uid_sha, 'motor_output_allowed':False, 'approved_for_runtime':False,
            'angle_wrap_applied':False, 'plan':{'allowed_can_types':[0,17],'automatic_retry':False,
                'motor_output_available':False, 'ports':{'front':self.config['front_port'],'rear':self.config['rear_port']},
                'ids_by_bus':{'front':list(range(1,7)),'rear':list(range(7,13))}},
            'identities':{str(mid):{'mcu_uid_hex':self.uids[str(mid)]} for mid in tool.IDS},
            'telemetry':{'rows':{str(mid):{'run_mode':0,'current':0.,'position_span_deg':.01,
                'median_position_rad':.1} for mid in tool.IDS}}}
        for mid, delta_deg in (changes or {}).items():
            result['telemetry']['rows'][str(mid)]['median_position_rad'] += math.radians(delta_deg)
        return result

    def prepare(self, ids=None, l_legs=None, trace_events=False):
        return tool.make_plan(kit_root=self.kit, kit_manifest_sha256=digest(self.manifest),
            angle_profile=self.profile_path, angle_profile_sha256=digest(self.profile_path),
            capture=self.seed_path, capture_sha256=digest(self.seed_path), ids=ids, l_legs=l_legs,
            session_dir=self.root/'future-captures', target_kit_root=self.kit, target_python='/usr/bin/python3',
            trace_events=trace_events)

    def enable_trace_collector(self, code=None):
        path = self.kit/tool.CAPTURE_SOURCE; path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(code or "def main(argv=None):\n    ap.add_argument('--trace-events', type=Path)\n", encoding='utf-8')
        manifest = json.loads(self.manifest.read_bytes()); manifest['files'][tool.CAPTURE_SOURCE] = digest(path)
        self.manifest.write_bytes(encoded(manifest))
        return path

    def traced(self, plan, name, capture):
        stage = next(item for item in plan['stages'] if item['name'] == name)
        path = Path(stage['capture_trace_output']); path.parent.mkdir(parents=True, exist_ok=True)
        raw = b'{"bus":"front","kind":"can_tx"}\n{"bus":"rear","kind":"can_rx_bytes"}\n'
        path.write_bytes(raw)
        capture = copy.deepcopy(capture)
        capture['plan']['trace_events'] = {'path':str(path),'max_events':tool.TRACE_MAX_EVENTS,'max_bytes':tool.TRACE_MAX_BYTES}
        capture['trace_events'] = {'path':str(path),'sha256':digest(path),'event_count':2,
            'byte_count':len(raw),'attempted_event_count':2,'complete':True,'status':'COMPLETE_EVENT_TRACE','errors':[]}
        return capture

    def observation(self, plan, pins, mid=None):
        direction = tool.axis_direction(mid) if mid else None
        return {'schema':tool.OBSERVATION_SCHEMA, 'operator_id':'synthetic-observer',
            'observed_at':'2026-10-05T09:01:00+09:00', 'physical_observation_note':'Explicit relative output observation fixture',
            'power_event':{'label':'operator-observed-uninterrupted-40v-event', 'unchanged_since_seed':True,
                'inferred_from_boot':False, 'boot_id':plan['boot_id'],
                'capture_sha256':{'seed':digest(self.seed_path), **pins}},
            'l_pose_observed':True, 'relative_output_shaft_observed':True,
            'only_selected_relative_joint_moved':True, 'returned_to_baseline':True,
            'physical_direction_observed':direction['physical_direction'] if direction else None,
            'model_delta_sign':direction['model_delta_sign'] if direction else None}

    def direction_inputs(self, mid=4, changed=None):
        plan = self.prepare([mid]); expected = tool.axis_direction(mid)['model_delta_sign']*SIGNS[mid-1]*10
        base, moved, returned = paired(self.capture(1)), paired(self.capture(2,{mid:expected, **(changed or {})})), paired(self.capture(3))
        observation = self.observation(plan, {'baseline':base[1],'moved':moved[1],'return':returned[1]}, mid)
        return plan, base, moved, returned, observation

    def test_plan_keeps_old_axes_and_builds_all_dual_port_capture_arrays_without_execution(self):
        before = self.profile_path.read_bytes()
        with patch('subprocess.run', side_effect=AssertionError('A process was started')):
            plan = self.prepare()
        self.assertEqual(len(plan['stages']),40); self.assertEqual(plan['l_legs'],list(tool.LEGS))
        self.assertFalse((self.root/'future-captures').exists())
        self.assertEqual(self.profile_path.read_bytes(),before)
        self.assertEqual(list(plan['axes'].values()),self.profile['axes'])
        self.assertIsNone(plan['absolute_zero_error_rad']); self.assertFalse(plan['zero_verified'])
        for stage in plan['stages']:
            argv = stage['capture_argv']
            self.assertEqual(argv[:4],['/usr/bin/python3','-B','-m',tool.CAPTURE_MODULE])
            self.assertEqual(argv[argv.index('--front-port')+1],self.config['front_port'])
            self.assertEqual(argv[argv.index('--rear-port')+1],self.config['rear_port'])
            self.assertIn('--execute-readonly',argv); self.assertNotIn('--execute',argv)
            self.assertEqual(stage['capture_env'],{'PYTHONPATH':str(self.kit/'runtime')})
            self.assertFalse(plan['output_allowed']); self.assertTrue(stage['all12_UIDs_required'])
        self.assertEqual([tool.axis_direction(i)['model_delta_sign'] for i in tool.IDS],
                         [1,-1,-1,1,-1,1,1,-1,-1,1,-1,1])

    def test_priority_ids_and_independent_optional_l_legs_are_not_all_remeasurements(self):
        plan = self.prepare([4,5,10]); self.assertEqual(plan['l_legs'],['FL','RL'])
        self.assertEqual(len(plan['stages']),11)
        extended = self.prepare([4,5,10],['FR','RR'])
        self.assertEqual(extended['l_ids_by_leg'],{'FR':[1,2,3],'RR':[7,8,9]})
        self.assertEqual([s['motor_id'] for s in extended['stages'] if s.get('phase')=='moved'],[4,5,10])
        for ids in ([],[4,4],[True],[13]):
            with self.subTest(ids=ids),self.assertRaises(ValueError):self.prepare(ids)

    def test_optional_trace_plans_fresh_per_stage_files_and_pins_collector_without_import(self):
        path = self.enable_trace_collector()
        with patch('subprocess.run', side_effect=AssertionError('Process started')):
            plan = self.prepare([5,10], trace_events=True); tool.validate_plan(plan)
        self.assertEqual(plan['readonly_event_trace']['collector_source'], {'path':str(path),'sha256':digest(path)})
        self.assertTrue(plan['readonly_event_trace']['file_integrity_only'])
        paths = [stage['capture_trace_output'] for stage in plan['stages']]
        self.assertEqual(len(paths),8); self.assertEqual(len(set(paths)),8)
        self.assertFalse((self.root/'future-captures').exists())
        for stage in plan['stages']:
            self.assertEqual(stage['capture_argv'][-2:],['--trace-events',stage['capture_trace_output']])
            self.assertNotEqual(stage['capture_output'],stage['capture_trace_output'])
        self.assertFalse(plan['output_allowed'])

    def test_trace_option_rejects_old_collector_nonboolean_and_frozen_source_drift(self):
        with self.assertRaisesRegex(ValueError,'Frozen collector'):self.prepare([5],trace_events=True)
        path = self.enable_trace_collector("# --trace-events is unsupported\nap.add_argument('--output')\n")
        with self.assertRaisesRegex(ValueError,'does not support'):self.prepare([5],trace_events=True)
        self.enable_trace_collector("def main(argv=None):\n    if False:\n        ap.add_argument('--trace-events')\n")
        with self.assertRaisesRegex(ValueError,'does not support'):self.prepare([5],trace_events=True)
        self.enable_trace_collector(); plan = self.prepare([5],trace_events=True)
        path.write_text("ap.add_argument('--output')\n")
        with self.assertRaises(ValueError):tool.validate_plan(plan)
        for value in (1,'yes',None):
            with self.subTest(value=value),self.assertRaises(ValueError):self.prepare([5],trace_events=value)

    def test_optional_trace_review_binds_actual_files_and_keeps_candidate_scope(self):
        self.enable_trace_collector(); plan = self.prepare([4],trace_events=True)
        base = paired(self.traced(plan,'id4-baseline',self.capture(1)))
        moved = paired(self.traced(plan,'id4-moved',self.capture(2,{4:-10})))
        returned = paired(self.traced(plan,'id4-return',self.capture(3)))
        observation = self.observation(plan, {'baseline':base[1],'moved':moved[1],'return':returned[1]},4)
        result = tool.review_direction(plan,4,base,moved,returned,observation)
        self.assertEqual(result['status'],'DIRECTION_ONLY_CANDIDATE')
        self.assertEqual(set(result['event_trace_sources']),{'baseline','moved','return'})
        self.assertFalse(result['output_allowed']); self.assertIsNone(result['absolute_zero_error_rad'])
        captured = paired(self.traced(plan,'fl-l',self.capture(4)))
        result = tool.review_l(plan,'FL',captured,self.observation(plan,{'l':captured[1]}))
        self.assertEqual(result['event_trace_sources']['l']['event_count'],2)

    def test_trace_failure_missing_bytes_tampering_and_symlink_block_review(self):
        self.enable_trace_collector(); plan = self.prepare([4],trace_events=True)
        original = self.traced(plan,'id4-return',self.capture(3))
        path = Path(original['trace_events']['path']); raw = path.read_bytes()
        for case in ('missing','partial','hash','count','bus','path','declaration','failed','symlink'):
            capture = copy.deepcopy(original); path.unlink(missing_ok=True); path.write_bytes(raw)
            if case=='missing':path.unlink()
            elif case=='partial':path.write_bytes(raw[:-1])
            elif case=='hash':capture['trace_events']['sha256']='0'*64
            elif case=='count':capture['trace_events']['attempted_event_count']=3
            elif case=='bus':
                path.write_bytes(raw.replace(b'front',b'wrong'))
                capture['trace_events']['sha256']=digest(path)
            elif case=='path':capture['trace_events']['path']=str(self.root/'other.jsonl')
            elif case=='declaration':capture['plan']['trace_events']['max_events']=1
            elif case=='failed':capture['trace_events']['complete']=False
            else:
                alternate=self.root/'alternate.jsonl'; alternate.write_bytes(raw); path.unlink(); path.symlink_to(alternate)
            with self.subTest(case=case),self.assertRaises((ValueError,OSError)):
                tool.review_event_trace(plan,'id4-return',capture)
        path.unlink(missing_ok=True); path.write_bytes(raw)

    def test_traced_cli_rehashes_raw_events_before_writing_and_never_executes(self):
        self.enable_trace_collector(); plan = self.prepare([4],trace_events=True)
        values = {'baseline':self.traced(plan,'id4-baseline',self.capture(1)),
                  'moved':self.traced(plan,'id4-moved',self.capture(2,{4:-10})),
                  'return-capture':self.traced(plan,'id4-return',self.capture(3))}
        observation = self.observation(plan, {'baseline':paired(values['baseline'])[1],
            'moved':paired(values['moved'])[1],'return':paired(values['return-capture'])[1]},4)
        inputs = {'plan':plan,'observation':observation,**values}; argv=['review-direction','--id','4']
        for key,value in inputs.items():
            path=self.root/(key+'.json');path.write_bytes(encoded(value))
            argv += ['--'+key,str(path),'--'+key+'-sha256',digest(path)]
        output=self.root/'trace-review.json'
        with patch('subprocess.run',side_effect=AssertionError('Process started')),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(tool.main(argv+['--output',str(output)]),0)
        self.assertFalse(json.loads(output.read_bytes())['output_allowed'])
        real_review=tool.review_direction
        def change_trace(*args):
            result=real_review(*args)
            Path(values['moved']['trace_events']['path']).write_bytes(b'{"bus":"front"}\n')
            return result
        late=self.root/'late-trace-review.json'
        with patch.object(tool,'review_direction',side_effect=change_trace),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            tool.main(argv+['--output',str(late)])
        self.assertFalse(late.exists())

    def test_trace_rejects_rehashed_nonfinite_duplicate_json_and_event_byte_budgets(self):
        self.enable_trace_collector(); plan=self.prepare([4],trace_events=True)
        original=self.traced(plan,'id4-return',self.capture(3))
        path=Path(original['trace_events']['path'])
        for raw in (b'{"bus":"front","value":NaN}\n',
                    b'{"bus":"front","bus":"rear"}\n',
                    b'{"bus":"front","value":1e999}\n',
                    b'{"bus":"front"}\n'*(tool.TRACE_MAX_EVENTS+1),
                    b'{"bus":"front","padding":"'+b'x'*tool.TRACE_MAX_BYTES+b'"}\n'):
            capture=copy.deepcopy(original);path.write_bytes(raw)
            capture['trace_events'].update(sha256=digest(path),byte_count=len(raw),
                event_count=raw.count(b'\n'),attempted_event_count=raw.count(b'\n'))
            with self.subTest(byte_count=len(raw)),self.assertRaises(ValueError):
                tool.review_event_trace(plan,'id4-return',capture)

    def test_trace_supports_actual_collector_source_without_import_and_refuses_parent_symlink(self):
        actual=Path(__file__).resolve().parents[1]/tool.CAPTURE_SOURCE
        self.enable_trace_collector(actual.read_text())
        plan=self.prepare([4],trace_events=True);tool.validate_plan(plan)
        capture=self.traced(plan,'id4-return',self.capture(3))
        parent=Path(capture['trace_events']['path']).parent
        moved=self.root/'moved-trace-dir';parent.rename(moved);parent.symlink_to(moved,target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'without symlinks'):
            tool.review_event_trace(plan,'id4-return',capture)

    def test_all12_relative_directions_are_candidates_and_do_not_update_zero(self):
        for mid in tool.IDS:
            with self.subTest(mid=mid):
                plan, base, moved, returned, observation = self.direction_inputs(mid)
                result = tool.review_direction(plan,mid,base,moved,returned,observation)
                self.assertEqual(result['sign_candidate'],SIGNS[mid-1]); self.assertTrue(result['profile_sign_matches'])
                self.assertEqual(result['retained_axis'],self.profile['axes'][mid-1])
                for key in ('profile_changed','zero_verified','direction_verified_for_runtime','dynamic_scale_verified','approved_for_runtime','output_allowed'):
                    self.assertFalse(result[key])
                self.assertIsNone(result['physical_uncertainty_rad']); self.assertIsNone(result['measured_model_angle_rad'])

    def test_opposite_raw_sign_is_a_visible_conflict_without_changing_existing_axis(self):
        plan, base, moved, returned, observation = self.direction_inputs(4,{4:10})
        result = tool.review_direction(plan,4,base,moved,returned,observation)
        self.assertEqual(result['status'],'SIGN_CONFLICT_REVIEW_REQUIRED'); self.assertFalse(result['profile_sign_matches'])
        self.assertEqual(result['retained_axis']['offset_rad'],self.profile['axes'][3]['offset_rad'])

    def test_incomplete_physical_observation_and_unknown_or_unbound_power_block(self):
        plan, base, moved, returned, original = self.direction_inputs()
        changes = [('relative_output_shaft_observed',False),('only_selected_relative_joint_moved',None),
            ('returned_to_baseline',False),('l_pose_observed',None),('physical_observation_note',''),
            ('model_delta_sign',True),('physical_direction_observed','face')]
        for key, value in changes:
            observation = copy.deepcopy(original); observation[key] = value
            with self.subTest(key=key),self.assertRaises(ValueError):tool.review_direction(plan,4,base,moved,returned,observation)
        for key, value in (('label',None),('label','UNKNOWN'),('label',plan['boot_id']),
                ('unchanged_since_seed',False),('inferred_from_boot',True),('boot_id','other-boot'),
                ('capture_sha256',{'baseline':base[1]})):
            observation = copy.deepcopy(original); observation['power_event'][key] = value
            with self.subTest(power=key,value=value),self.assertRaises(ValueError):
                tool.review_direction(plan,4,base,moved,returned,observation)

    def test_review_json_requires_a_physical_note_beyond_confirmation_or_cancellation(self):
        plan, base, moved, returned, original = self.direction_inputs()
        captured = paired(self.capture(4))
        l_original = self.observation(plan, {'l':captured[1]})
        tokens = tool.PHYSICAL_NOTE_CONFIRMATION_TOKENS | tool.PHYSICAL_NOTE_CANCEL
        for token in sorted(tokens):
            for note in (token, token.upper()):
                direction = copy.deepcopy(original); direction['physical_observation_note'] = note
                l_observation = copy.deepcopy(l_original); l_observation['physical_observation_note'] = note
                with self.subTest(note=note, kind='direction'),self.assertRaisesRegex(ValueError,'standalone'):
                    tool.review_direction(plan,4,base,moved,returned,direction)
                with self.subTest(note=note, kind='l'),self.assertRaisesRegex(ValueError,'standalone'):
                    tool.review_l(plan,'FL',captured,l_observation)

    def test_cli_review_confirmation_only_note_does_not_create_review(self):
        plan, base, moved, returned, observation = self.direction_inputs()
        observation['physical_observation_note'] = 'y'
        inputs = {'plan':plan,'observation':observation,'baseline':base[0],
                  'moved':moved[0],'return-capture':returned[0]}
        argv = ['review-direction','--id','4']
        for key,value in inputs.items():
            path = self.root/(key+'.json'); path.write_bytes(encoded(value))
            argv += ['--'+key,str(path),'--'+key+'-sha256',digest(path)]
        output = self.root/'unwritten-review.json'
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as raised:
            tool.main(argv+['--output',str(output)])
        self.assertEqual(raised.exception.code,2)
        self.assertFalse(output.exists())

    def test_motion_return_discontinuity_and_incomplete_capture_fail_closed(self):
        for changes in ({4:-2},{4:-21},{4:-370},{5:4},{12:4}):
            plan, base, moved, returned, observation = self.direction_inputs(4,changes)
            with self.subTest(changes=changes),self.assertRaises(ValueError):tool.review_direction(plan,4,base,moved,returned,observation)
        plan, base, moved, returned, observation = self.direction_inputs()
        for change in ('return','boot','uid','static','current','boolean_mode','time','epoch','wrap','missing'):
            capture = copy.deepcopy(returned[0])
            if change=='return':capture['telemetry']['rows']['4']['median_position_rad'] += math.radians(4)
            elif change=='boot':capture['boot_id']='other'
            elif change=='uid':capture['identities']['12']['mcu_uid_hex']='f'*16
            elif change=='static':capture['telemetry']['rows']['7']['position_span_deg']=.11
            elif change=='current':capture['telemetry']['rows']['1']['current']=.01
            elif change=='boolean_mode':capture['telemetry']['rows']['1']['run_mode']=False
            elif change=='time':capture['started_at']='2026-10-05T09:00:00+09:00'
            elif change=='epoch':capture['motor_power_epoch']='different-event'
            elif change=='wrap':capture['angle_wrap_applied']=True
            else:del capture['telemetry']['rows']['12']
            new = paired(capture); updated = copy.deepcopy(observation); updated['power_event']['capture_sha256']['return']=new[1]
            with self.subTest(change=change),self.assertRaises(ValueError):tool.review_direction(plan,4,base,moved,new,updated)

    def test_l_reports_old_zero_nominal_difference_with_unknown_absolute_error(self):
        plan = self.prepare([4,5,10]); captured = paired(self.capture(1,{4:2}))
        observation = self.observation(plan,{'l':captured[1]})
        result = tool.review_l(plan,'FL',captured,observation)
        row = result['rows_by_id']['4']
        self.assertAlmostEqual(row['difference_from_nominal_deg'],-2.)
        self.assertEqual(row['nominal_model_deg_diagnostic_only'],-90.)
        self.assertEqual(row['retained_axis'],self.profile['axes'][3]); self.assertFalse(row['zero_verified'])
        self.assertIsNone(row['absolute_zero_error_rad']); self.assertFalse(result['output_allowed'])
        observation['l_pose_observed']=False
        with self.assertRaises(ValueError):tool.review_l(plan,'FL',captured,observation)

    def test_l_rejects_missing_numeric_branch_and_does_not_hide_a_whole_turn(self):
        plan = self.prepare([6]); captured = paired(self.capture(1,{6:50}))
        with self.assertRaisesRegex(ValueError,'Unique existing numeric branch'):
            tool.review_l(plan,'FL',captured,self.observation(plan,{'l':captured[1]}))
        captured = paired(self.capture(1,{6:360}))
        with self.assertRaisesRegex(ValueError,'discontinuity'):
            tool.review_l(plan,'FL',captured,self.observation(plan,{'l':captured[1]}))

    def test_plan_pins_reject_frozen_kit_drift_source_hash_and_typed_plan_changes(self):
        plan = self.prepare([4]); tool.validate_plan(plan)
        changed = copy.deepcopy(plan); changed['axes']['4']['uncertainty_rad']=False
        with self.assertRaisesRegex(ValueError,'differs'):tool.validate_plan(changed)
        changed = copy.deepcopy(plan); changed['stages'][0]['capture_argv'][2]='-c'
        with self.assertRaisesRegex(ValueError,'differs'):tool.validate_plan(changed)
        self.profile_path.write_bytes(self.profile_path.read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError,'changed'):tool.validate_plan(plan)

    def test_capture_can_id_alias_and_seed_epoch_conflict_are_rejected(self):
        for value in (True, 1.0):
            seed = self.capture(0); seed['plan']['ids_by_bus']['front'][0] = value
            self.seed_path.write_bytes(encoded(seed))
            with self.subTest(alias=value),self.assertRaisesRegex(ValueError,'Type0/17'):
                self.prepare([4])
        self.seed = self.capture(0); self.seed['motor_power_epoch']='a-different-explicit-power-event'
        self.seed_path.write_bytes(encoded(self.seed))
        plan, base, moved, returned, observation = self.direction_inputs()
        with self.assertRaisesRegex(ValueError,'conflicts with attestation'):
            tool.review_direction(plan,4,base,moved,returned,observation)

    def test_observation_cannot_predate_seed_or_last_capture_and_handles_timezone(self):
        plan, base, moved, returned, observation = self.direction_inputs()
        for timestamp in ('2026-10-05T08:59:59+09:00','2026-10-05T09:00:03.499999+09:00'):
            observation['observed_at'] = timestamp
            with self.subTest(timestamp=timestamp),self.assertRaisesRegex(ValueError,'after the final capture'):
                tool.review_direction(plan,4,base,moved,returned,observation)
        observation['observed_at']='2026-10-05T00:00:03.500000+00:00'
        self.assertEqual(tool.review_direction(plan,4,base,moved,returned,observation)['status'],'DIRECTION_ONLY_CANDIDATE')
        captured = paired(self.capture(4)); l_observation=self.observation(plan,{'l':captured[1]})
        l_observation['observed_at']='2026-10-05T09:00:04+09:00'
        with self.assertRaisesRegex(ValueError,'after the final capture'):
            tool.review_l(plan,'FL',captured,l_observation)

    def test_profile_transient_change_during_loader_cannot_be_hidden_by_restore(self):
        original = self.profile_path.read_bytes(); real_loader = tool.load_profile
        def transient(path):
            changed = copy.deepcopy(self.profile); changed['axes'][3]['offset_rad'] += .01
            self.profile_path.write_bytes(encoded(changed))
            try: return real_loader(path)
            finally: self.profile_path.write_bytes(original)
        with patch.object(tool,'load_profile',side_effect=transient),self.assertRaisesRegex(ValueError,'changed during validation'):
            self.prepare([4])
        self.assertEqual(self.profile_path.read_bytes(),original)

    def test_cli_plan_is_private_and_does_not_execute_generated_commands(self):
        output = self.root/'plan.json'
        argv = ['plan','--kit-root',str(self.kit),'--kit-manifest-sha256',digest(self.manifest),
            '--angle-profile',str(self.profile_path),'--angle-profile-sha256',digest(self.profile_path),
            '--capture',str(self.seed_path),'--capture-sha256',digest(self.seed_path),
            '--session-dir',str(self.root/'captures'),'--target-kit-root',str(self.kit),
            '--target-python','/usr/bin/python3','--id','4','--id','5','--id','10',
            '--l-leg','FR','--output',str(output)]
        with patch('subprocess.run',side_effect=AssertionError('Process started')),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(tool.main(argv),0)
        plan = json.loads(output.read_bytes()); tool.validate_plan(plan)
        self.assertEqual(plan['selected_ids'],[4,5,10]); self.assertEqual(plan['l_legs'],['FR'])
        self.assertEqual(len(plan['stages']),10); self.assertEqual(output.stat().st_mode&0o777,0o600)
        self.assertFalse((self.root/'captures').exists())

    def test_cli_review_writes_only_new_private_file_and_preserves_pinned_inputs(self):
        plan, base, moved, returned, observation = self.direction_inputs()
        inputs = {'plan':plan,'observation':observation,'baseline':base[0],'moved':moved[0],'return-capture':returned[0]}
        argv = ['review-direction','--id','4']; snapshots = {}
        for key,value in inputs.items():
            path = self.root/(key+'.json'); path.write_bytes(encoded(value)); snapshots[path]=path.read_bytes()
            argv += ['--'+key,str(path),'--'+key+'-sha256',digest(path)]
        output = self.root/'review.json'; argv += ['--output',str(output)]
        with patch('subprocess.run',side_effect=AssertionError('Process started')),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(tool.main(argv),0)
        self.assertEqual(output.stat().st_mode&0o777,0o600)
        result = json.loads(output.read_bytes()); self.assertFalse(result['approved_for_runtime'])
        self.assertEqual(result['status'],'DIRECTION_ONLY_CANDIDATE')
        for path,raw in snapshots.items():self.assertEqual(path.read_bytes(),raw)
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):tool.main(argv)

    def test_cli_final_capture_rehash_failure_never_writes_a_review(self):
        plan, base, moved, returned, observation = self.direction_inputs()
        inputs = {'plan':plan,'observation':observation,'baseline':base[0],'moved':moved[0],'return-capture':returned[0]}
        argv = ['review-direction','--id','4']
        for key,value in inputs.items():
            path = self.root/(key+'.json'); path.write_bytes(encoded(value))
            argv += ['--'+key,str(path),'--'+key+'-sha256',digest(path)]
        output = self.root/'never-written.json'; argv += ['--output',str(output)]
        real_review = tool.review_direction
        def change_after_review(*args):
            result = real_review(*args)
            moved_path = self.root/'moved.json'; moved_path.write_bytes(moved_path.read_bytes()+b' ')
            return result
        with patch.object(tool,'review_direction',side_effect=change_after_review),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            tool.main(argv)
        self.assertFalse(output.exists())


if __name__ == '__main__': unittest.main()
