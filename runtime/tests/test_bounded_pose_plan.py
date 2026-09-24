"""Synthetic positions/timestamps only; no hardware or transport imports."""
import ast
import copy
from dataclasses import FrozenInstanceError, replace
import math
from pathlib import Path
import unittest

from singularitydog_hw import bounded_pose_plan as pose

START = 10_000_000_000


def fixture(leg='FR'):
    ids=pose.LEGS[leg]
    centers={i:{'position_rad':0.,'request_ns':START-1_000_000,'received_ns':START} for i in ids}
    targets={i:d for i,d in zip(ids,(math.radians(5),math.radians(-2),0.))}
    plan=pose.build_plan(leg,centers,targets,now_ns=START)
    samples=[]
    for index in range(20):
        when=START+pose.RAMP_NS+index*pose.CYCLE_NS
        samples.append({'sample_index':index,'checked_ns':when+14_000_000,
            'joints':{i:{'position_rad':targets[i],'request_ns':when+k*5_000_000,
                          'received_ns':when+k*5_000_000+2_000_000} for k,i in enumerate(ids)}})
    return plan,centers,targets,samples


def evaluate(plan,samples,**kwargs):
    return pose.evaluate_hold(plan,samples,run_start_ns=kwargs.get('run_start_ns',START),
                              ended_ns=kwargs.get('ended_ns',START+pose.DURATION_NS))


class PlanTests(unittest.TestCase):
    def test_matched_start_all_axes_fixed_tolerance_without_wrap(self):
        for leg in pose.LEGS:
            plan,_,_,_=fixture(leg)
            references={i:0. for i in plan.ids}
            report=pose.evaluate_matched_start(plan,references,now_ns=START)
            self.assertTrue(report['passed'])
            self.assertFalse(report['automatic_repositioning'])
            for flag in pose.FLAGS:self.assertFalse(report[flag])
            for mid in plan.ids:
                for offset in (.501,-.501,360.,-360.):
                    bad=dict(references);bad[mid]=math.radians(offset)
                    result=pose.evaluate_matched_start(plan,bad,now_ns=START)
                    self.assertFalse(result['passed'])
                    self.assertFalse(result['motors'][str(mid)]['within_tolerance'])

    def test_matched_start_exact_endpoint_no_epsilon_and_fresh_request_required(self):
        _,centers,_,_=fixture()
        references={1:1.,2:1.,3:1.}
        for mid,sign in ((1,-1),(2,1)):
            centers[mid]['position_rad']=1.+sign*pose.MATCHED_START_TOLERANCE_RAD
        centers[3]['position_rad']=1.
        targets={i:c['position_rad'] for i,c in centers.items()}
        plan=pose.build_plan('FR',centers,targets,now_ns=START)
        self.assertTrue(pose.evaluate_matched_start(plan,references,now_ns=START)['passed'])
        for mid,toward in ((1,-math.inf),(2,math.inf)):
            bad=copy.deepcopy(centers);bad[mid]['position_rad']=math.nextafter(bad[mid]['position_rad'],toward)
            changed=pose.build_plan('FR',bad,targets,now_ns=START)
            self.assertFalse(pose.evaluate_matched_start(changed,references,now_ns=START)['passed'])
        for when in (START-1,START+100_000_000):
            self.assertFalse(pose.evaluate_matched_start(plan,references,now_ns=when)['passed'])

    def test_matched_start_shape_and_tolerance_are_strict_and_inputs_copied(self):
        plan,_,_,_=fixture()
        for bad in (None,{}, {1:0.,2:0.},{1:0.,2:0.,4:0.},{1:0.,'1':0.,2:0.,3:0.},
                    {True:0.,2:0.,3:0.},{1:True,2:0.,3:0.},{1:math.nan,2:0.,3:0.},
                    {1:math.inf,2:0.,3:0.},{1:13.,2:0.,3:0.}):
            with self.assertRaises(ValueError):pose.evaluate_matched_start(plan,bad,now_ns=START)
        for bad in (0.,math.radians(.4),math.radians(.6),True,None,math.nan,'0.5',
                    math.nextafter(pose.MATCHED_START_TOLERANCE_RAD,math.inf)):
            with self.assertRaises(ValueError):
                pose.evaluate_matched_start(plan,{1:0.,2:0.,3:0.},now_ns=START,tolerance_rad=bad)
        references={1:0.,2:0.,3:0.}
        report=pose.evaluate_matched_start(plan,references,now_ns=START)
        references[1]=4.
        self.assertEqual(report['reference_positions_rad']['1'],0.)

    def test_all_leg_exact_scope_and_fixed_trajectory(self):
        for leg in pose.LEGS:
            plan,centers,targets,_=fixture(leg)
            self.assertEqual(plan.ids,pose.LEGS[leg])
            self.assertEqual(pose.target_at(plan,0),{i:0. for i in plan.ids})
            for i,t in pose.target_at(plan,2*pose.NS).items():self.assertAlmostEqual(t,targets[i]/2)
            self.assertEqual(pose.target_at(plan,4*pose.NS),targets)
            self.assertEqual(pose.target_at(plan,5*pose.NS),targets)
            data=plan.as_dict()
            self.assertEqual((data['ramp_ns'],data['hold_ns'],data['duration_ns'],data['active_budget_ns']),
                             (4*pose.NS,pose.NS,5*pose.NS,6*pose.NS))
            for key in pose.FLAGS:self.assertFalse(data[key])
            with self.assertRaises(ValueError):pose.target_at(plan,5*pose.NS+1)

    def test_target_shape_wrong_leg_duplicate_bool_and_missing_rejected(self):
        _,centers,targets,_=fixture()
        for bad in ({1:0.,2:0.},{1:0.,2:0.,4:0.},{1:0.,'1':0.,2:0.,3:0.},{True:0.,2:0.,3:0.}):
            with self.assertRaises(ValueError):pose.build_plan('FR',centers,bad,now_ns=START)
        with self.assertRaises(ValueError):pose.build_plan('fr',centers,targets,now_ns=START)

    def test_no_turn_wrapping_automatic_segmentation_or_overrange(self):
        _,centers,targets,_=fixture()
        for value in (2*math.pi,-2*math.pi,pose.MAX_DELTA_RAD+1e-10,12.58):
            bad=dict(targets);bad[1]=value
            with self.assertRaises(ValueError):pose.build_plan('FR',centers,bad,now_ns=START)

    def test_five_degree_endpoint_cancellation_is_handled_without_epsilon(self):
        _,centers,targets,_=fixture()
        for i in centers:centers[i]['position_rad']=1.
        targets={1:1.-pose.MAX_DELTA_RAD,2:1.+pose.MAX_DELTA_RAD,3:1.}
        plan=pose.build_plan('FR',centers,targets,now_ns=START)
        self.assertEqual(pose.offsets_at(plan,4*pose.NS),{1:-pose.MAX_DELTA_RAD,2:pose.MAX_DELTA_RAD,3:0.})
        self.assertEqual(pose.target_at(plan,4*pose.NS),targets)
        for i,direction in ((1,-math.inf),(2,math.inf)):
            bad=dict(targets);bad[i]=math.nextafter(bad[i],direction)
            with self.assertRaises(ValueError):pose.build_plan('FR',centers,bad,now_ns=START)

    def test_stale_future_or_reversed_center_times_rejected(self):
        _,centers,targets,_=fixture()
        for changes in ({'received_ns':START-100_000_001,'request_ns':START-100_000_002},
                        {'received_ns':START+1},{'request_ns':START+1},{'received_ns':float(START)},
                        {'request_ns':True},{'request_ns':START-100_000_001}):
            bad=copy.deepcopy(centers);bad[1].update(changes)
            with self.assertRaises(ValueError):pose.build_plan('FR',bad,targets,now_ns=START)

    def test_nonnumeric_nonfinite_and_existing_codec_headroom_rejected(self):
        _,centers,targets,_=fixture()
        for value in (float('nan'),float('inf'),True,'0',10**1000):
            bad=dict(targets);bad[1]=value
            with self.assertRaises(ValueError):pose.build_plan('FR',centers,bad,now_ns=START)
        bad=copy.deepcopy(centers);bad[1]['position_rad']=12.56
        with self.assertRaises(ValueError):pose.build_plan('FR',bad,{1:12.56,2:0.,3:0.},now_ns=START)

    def test_plan_does_not_retain_mutable_inputs_or_accept_forged_id_order(self):
        plan,centers,targets,_=fixture()
        centers[1]['position_rad']=9.;targets[1]=8.
        self.assertEqual(plan.centers[0].position_rad,0.)
        self.assertAlmostEqual(plan.targets_rad[0],math.radians(5))
        with self.assertRaises(FrozenInstanceError):plan.leg='FL'
        with self.assertRaises(ValueError):pose.target_at(replace(plan,ids=(3,2,1)),0)
        with self.assertRaises(ValueError):pose.target_at(plan,True)


class SettlingObservationTests(unittest.TestCase):
    def fixture(self):
        _,centers,_,rows=fixture('RR')
        plan=pose.build_plan('RR',centers,{7:0.,8:0.,9:0.},now_ns=START)
        for row in rows:
            for item in row['joints'].values():item['position_rad']=0.
        return plan,rows

    def evaluate(self,plan,rows,**kwargs):
        return pose.evaluate_settling_observation(plan,rows,run_start_ns=START,
            ended_ns=kwargs.get('ended_ns',START+pose.DURATION_NS))

    def test_unreached_is_complete_data_only_and_strict_one_degree_flags_stay_false(self):
        plan,rows=self.fixture()
        for row in rows:row['joints'][9]['position_rad']=math.radians(1.4)
        original=copy.deepcopy(rows);result=self.evaluate(plan,rows)
        self.assertTrue(result['data_complete'])
        self.assertEqual(result['motors']['9']['accepted_samples'],20)
        self.assertEqual(result['first_hold_abs_error_rad']['9'],math.radians(1.4))
        self.assertFalse(evaluate(plan,rows)['hold_candidate_met'])
        self.assertFalse(evaluate(plan,rows)['arrival_candidate_met'])
        self.assertNotIn('hold_candidate_met',result)
        self.assertNotIn('arrival_candidate_met',result)
        self.assertEqual(rows,original)
        for flag in pose.FLAGS:self.assertFalse(result[flag])

    def test_error_limit_endpoints_ulp_overflow_and_nonfinite_per_axis(self):
        for mid in (7,8,9):
            for sign in (-1,1):
                plan,rows=self.fixture()
                for row in rows:row['joints'][mid]['position_rad']=sign*pose.SETTLING_MAX_ERROR_RAD
                self.assertTrue(self.evaluate(plan,rows)['data_complete'])
                rows[2]['joints'][mid]['position_rad']=math.nextafter(sign*pose.SETTLING_MAX_ERROR_RAD,sign*math.inf)
                self.assertFalse(self.evaluate(plan,rows)['data_complete'])
            for bad in (math.nan,math.inf,-math.inf,True):
                plan,rows=self.fixture();rows[1]['joints'][mid]['position_rad']=bad
                self.assertFalse(self.evaluate(plan,rows)['data_complete'])

    def test_first_baseline_worsening_is_absolute_fixed_and_not_rolling(self):
        for mid in (7,8,9):
            for sign in (-1,1):
                plan,rows=self.fixture()
                rows[2]['joints'][mid]['position_rad']=sign*pose.SETTLING_MAX_WORSENING_RAD
                self.assertTrue(self.evaluate(plan,rows)['data_complete'])
                rows[2]['joints'][mid]['position_rad']=math.nextafter(sign*pose.SETTLING_MAX_WORSENING_RAD,sign*math.inf)
                self.assertFalse(self.evaluate(plan,rows)['data_complete'])
            plan,rows=self.fixture()
            for index,row in enumerate(rows):row['joints'][mid]['position_rad']=math.radians(1.4 if index==0 else .5)
            rows[-1]['joints'][mid]['position_rad']=-math.radians(1.6)
            self.assertTrue(self.evaluate(plan,rows)['data_complete'])
            rows[-1]['joints'][mid]['position_rad']=-math.radians(1.651)
            self.assertFalse(self.evaluate(plan,rows)['data_complete'])

    def test_time_shape_coverage_and_elapsed_cannot_be_substituted_by_twenty_rows(self):
        for failure in ('missing','extra','duplicate','missing_id','reordered','equal_time','late_request',
                        'after_5s','stale','gap','short_span','too_early'):
            plan,rows=self.fixture()
            if failure=='missing':rows.pop()
            if failure=='extra':rows.append(copy.deepcopy(rows[-1]))
            if failure=='duplicate':rows[5]=copy.deepcopy(rows[4])
            if failure=='missing_id':rows[4]['joints'].pop(8)
            if failure=='reordered':rows[4],rows[5]=rows[5],rows[4]
            if failure=='equal_time':rows[3]['joints'][8]['received_ns']=rows[3]['joints'][8]['request_ns']
            if failure=='late_request':rows[3]['joints'][8]['request_ns']=START+pose.RAMP_NS+3*pose.CYCLE_NS+20_000_001
            if failure=='after_5s':rows[-1]['joints'][8]['received_ns']=START+pose.DURATION_NS+1
            if failure=='stale':rows[3]['checked_ns']+=101_000_000
            if failure=='gap':rows[3]['joints'][8]['received_ns']+=71_000_000
            if failure=='short_span':rows[-1]['joints'][8]['received_ns']=rows[0]['joints'][8]['received_ns']+899_999_999
            result=self.evaluate(plan,rows,ended_ns=START+pose.DURATION_NS-(1 if failure=='too_early' else 0))
            self.assertFalse(result['data_complete'],failure)

    def test_only_explicit_rr_plan_is_observable(self):
        for leg in ('FR','FL','RL'):
            plan,_,_,rows=fixture(leg)
            with self.assertRaises(ValueError):self.evaluate(plan,rows)


class HoldTests(unittest.TestCase):
    def test_positive_per_axis_request_delay_hold_is_diagnostic_only(self):
        plan,_,_,samples=fixture('RR')
        result=evaluate(plan,samples)
        self.assertEqual(result['status'],'DIAGNOSTIC_TARGET_HOLD_CANDIDATE_MET')
        self.assertTrue(result['elapsed_completed'] and result['arrival_candidate_met'] and result['hold_candidate_met'])
        for key in pose.FLAGS:self.assertFalse(result[key])
        for row in result['motors'].values():
            self.assertEqual(row['accepted_samples'],20);self.assertEqual(row['observed_span_ns'],950_000_000)
        self.assertTrue(all(j['received_ns']>j['request_ns'] for s in samples for j in s['joints'].values()))
        self.assertNotEqual(samples[0]['joints'][7]['received_ns'],samples[0]['joints'][8]['received_ns'])

    def test_equal_timestamps_across_different_axes_are_allowed(self):
        plan,_,_,samples=fixture()
        for row in samples:
            requested=START+pose.RAMP_NS+row['sample_index']*pose.CYCLE_NS
            for item in row['joints'].values():
                item['request_ns']=requested;item['received_ns']=requested+2_000_000
        self.assertTrue(evaluate(plan,samples)['hold_candidate_met'])

    def test_elapsed_time_does_not_turn_failed_arrival_into_success(self):
        plan,_,_,samples=fixture('FL')
        # Historical5-degree command with only0.66degree motion would fail here.
        for row in samples:row['joints'][4]['position_rad']=math.radians(.66)
        result=evaluate(plan,samples)
        self.assertTrue(result['elapsed_completed'])
        self.assertFalse(result['arrival_candidate_met'] or result['hold_candidate_met'])

    def test_arrival_then_drift_is_not_hold(self):
        plan,_,_,samples=fixture()
        samples[10]['joints'][1]['position_rad']+=math.radians(1.1)
        result=evaluate(plan,samples)
        self.assertTrue(result['arrival_candidate_met'])
        self.assertFalse(result['hold_candidate_met'])
        self.assertTrue(any('target error' in e for e in result['errors']))

    def test_no_missing_zero_fill_and_no_clock_only_endpoint_completion(self):
        plan,_,_,samples=fixture()
        for bad in (samples[:-1],[],samples+[samples[-1]]):
            result=evaluate(plan,bad)
            self.assertFalse(result['hold_candidate_met'])
            self.assertTrue(any('20 hold' in e for e in result['errors']))
        result=evaluate(plan,samples[:-1])
        self.assertTrue(result['elapsed_completed'])
        self.assertEqual(result['motors']['1']['observed_span_ns'],900_000_000)

    def test_twenty_samples_with_short_actual_coverage_still_fail(self):
        plan,_,_,samples=fixture()
        for row,delay in zip(samples[:2],(75_000_000,35_000_000)):
            for item in row['joints'].values():item['received_ns']=item['request_ns']+delay
            row['checked_ns']=max(item['received_ns'] for item in row['joints'].values())+2_000_000
        result=evaluate(plan,samples)
        self.assertEqual(result['motors']['1']['accepted_samples'],20)
        self.assertEqual(result['motors']['1']['observed_span_ns'],877_000_000)
        self.assertFalse(result['hold_candidate_met'])

    def test_twenty_three_ms_observation_completion_is_not_start_lateness(self):
        plan,_,_,samples=fixture()
        for row in samples:
            due=START+pose.RAMP_NS+row['sample_index']*pose.CYCLE_NS
            row['checked_ns']=due+23_000_000
            for k,item in enumerate(row['joints'].values()):
                item['received_ns']=due+17_000_000+k*2_000_000
        self.assertTrue(evaluate(plan,samples)['hold_candidate_met'])
        bad=copy.deepcopy(samples);bad[5]['joints'][1]['request_ns']+=21_000_000
        bad[5]['joints'][1]['received_ns']=bad[5]['joints'][1]['request_ns']+1_000_000
        self.assertFalse(evaluate(plan,bad)['hold_candidate_met'])

    def test_zero_latency_center_or_sample_is_rejected(self):
        plan,centers,targets,samples=fixture()
        centers[1]['request_ns']=centers[1]['received_ns']
        with self.assertRaises(ValueError):pose.build_plan('FR',centers,targets,now_ns=START)
        samples[5]['joints'][1]['received_ns']=samples[5]['joints'][1]['request_ns']
        self.assertFalse(evaluate(plan,samples)['hold_candidate_met'])

    def test_duplicate_reordered_index_or_source_and_missing_axis_fail(self):
        plan,_,_,samples=fixture()
        changes=[lambda s:s[4].update(sample_index=3),
                 lambda s:s[4]['joints'][1].update(request_ns=s[3]['joints'][1]['request_ns']),
                 lambda s:s[4]['joints'][1].update(received_ns=s[3]['joints'][1]['received_ns']),
                 lambda s:s[4]['joints'].pop(1),
                 lambda s:s[4].update(checked_ns=s[3]['checked_ns'])]
        for change in changes:
            bad=copy.deepcopy(samples);change(bad)
            result=evaluate(plan,bad);self.assertFalse(result['hold_candidate_met'])

    def test_future_stale_late_sources_and_excessive_gap_fail(self):
        plan,_,_,samples=fixture()
        changes=[lambda s:s[4]['joints'][1].update(received_ns=s[4]['checked_ns']+1),
                 lambda s:s[4].update(checked_ns=s[4]['checked_ns']+100_000_001),
                 lambda s:s[4]['joints'][1].update(request_ns=s[4]['checked_ns']-101_000_000,received_ns=s[4]['checked_ns']-100_000_001),
                 lambda s:s[0]['joints'][1].update(request_ns=START+pose.RAMP_NS-1),
                 lambda s:s[-1]['joints'][1].update(received_ns=START+pose.DURATION_NS+1)]
        for change in changes:
            bad=copy.deepcopy(samples);change(bad)
            result=evaluate(plan,bad);self.assertFalse(result['hold_candidate_met'])
        bad=copy.deepcopy(samples)
        for item in bad[5]['joints'].values():item['request_ns']-=25_000_000;item['received_ns']-=25_000_000
        result=evaluate(plan,bad)
        self.assertTrue(any('gap exceeds' in e for e in result['errors']))

    def test_nonfinite_bool_timestamp_and_range_failure_are_blocked(self):
        plan,_,_,samples=fixture()
        for field,value in [('position_rad',float('nan')),('position_rad',True),('position_rad',8.),
                            ('received_ns',float(START)),('request_ns',-1)]:
            bad=copy.deepcopy(samples);bad[10]['joints'][1][field]=value
            result=evaluate(plan,bad);self.assertFalse(result['hold_candidate_met'])

    def test_clock_budget_freshness_and_malformed_plan_fail_closed(self):
        plan,_,_,samples=fixture()
        for kwargs in ({'ended_ns':START+pose.DURATION_NS-1},{'ended_ns':START+pose.ACTIVE_BUDGET_NS+1},
                       {'run_start_ns':START-1},{'run_start_ns':START+pose.MAX_AGE_NS+1}):
            self.assertFalse(evaluate(plan,samples,**kwargs)['hold_candidate_met'])
        self.assertFalse(evaluate(None,samples)['hold_candidate_met'])
        self.assertFalse(evaluate(plan,None)['hold_candidate_met'])

    def test_pure_module_imports_have_no_io_can_clock_or_network_modules(self):
        tree=ast.parse(Path(pose.__file__).read_text())
        modules={n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
        modules.update(alias.name for n in ast.walk(tree) if isinstance(n,ast.Import) for alias in n.names)
        self.assertEqual(modules,{'dataclasses','math'})


if __name__=='__main__':unittest.main()
