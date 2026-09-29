"""Synthetic timing/review contracts; no hardware approval or timing claim."""
import copy
import unittest
from unittest.mock import patch

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import policy_output_runtime as runtime
from singularitydog_hw.policy_post_reply_timing import POST_REPLY_POLICY, PostReplyDeadlineBudget
import test_policy_live_profile as fixtures
import test_policy_output_runtime as runtime_fixtures
import test_ground_trial_plan as ground_fixtures


def settings():
    return dict(mode=POST_REPLY_POLICY, max_lateness_ms=1., max_consecutive_misses=1,
                rolling_window_cycles=100, max_misses_per_window=1)


class BudgetTests(unittest.TestCase):
    def setUp(self):
        self.budget=PostReplyDeadlineBudget(settings())

    def admit(self,index=0,elapsed=20_000_000,**changes):
        begin=1_000_000_000+index*25_000_000
        args=dict(index=index,begin_ns=begin,oldest_input_ns=begin+1_000_000,
            final_write_ns=begin+16_000_000,last_reply_ns=begin+19_000_000,
            output_sample_start_ns=begin+14_000_000,checked_ns=begin+elapsed,
            sample_age_ns=20_000_000)
        args.update(changes)
        return self.budget.admit(**args)

    def test_exact_period_is_on_time_and_one_ms_is_maximum(self):
        self.assertFalse(self.admit()['allowance_used'])
        self.assertTrue(self.admit(1,21_000_000)['allowance_used'])
        self.assertEqual(self.budget.accepted_misses,1)

    def test_lateness_does_not_extend_sample_age(self):
        with self.assertRaisesRegex(RuntimeError,'sample-age'):
            self.admit(elapsed=20_001_000,oldest_input_ns=1_000_000_001)

    def test_lateness_cap_is_independent_of_age(self):
        with self.assertRaisesRegex(RuntimeError,'lateness'):
            self.admit(elapsed=21_000_001,oldest_input_ns=1_002_000_000)

    def test_late_write_or_reply_never_uses_allowance(self):
        for changes in (dict(last_reply_ns=1_020_000_001),
                        dict(final_write_ns=1_020_000_001,last_reply_ns=1_020_000_002)):
            with self.subTest(changes=changes),self.assertRaisesRegex(RuntimeError,'write/reply'):
                self.admit(elapsed=20_100_000,**changes)

    def test_consecutive_and_nonconsecutive_misses_both_stop(self):
        self.admit(elapsed=20_100_000)
        with self.assertRaisesRegex(RuntimeError,'consecutive'):
            self.admit(1,20_100_000)
        self.admit(1)
        with self.assertRaisesRegex(RuntimeError,'rolling'):
            self.admit(2,20_100_000)

    def test_rolling_window_counts_exactly_100_cycles(self):
        self.admit(elapsed=20_100_000)
        for index in range(1,100):self.admit(index)
        self.assertTrue(self.admit(100,20_100_000)['allowance_used'])

    def test_reviewed_startup_and_one_steady_miss_have_separate_budgets(self):
        startup=self.admit(elapsed=20_069_000,startup_allowed=True)
        self.assertTrue(startup['startup_allowance_used'])
        self.assertFalse(startup['allowance_used'])
        self.assertEqual(self.budget.accepted_misses,0)
        for index in range(1,22):self.admit(index)
        steady=self.admit(22,20_181_000)
        self.assertTrue(steady['allowance_used'])
        self.assertFalse(steady['startup_allowance_used'])
        self.assertEqual(self.budget.accepted_misses,1)
        self.admit(23)
        with self.assertRaisesRegex(RuntimeError,'rolling'):
            self.admit(24,20_100_000)

    def test_startup_exception_keeps_all_hard_gates(self):
        for changes,reason in (
                (dict(elapsed=21_000_001,oldest_input_ns=1_002_000_000),'lateness'),
                (dict(elapsed=20_100_000,last_reply_ns=1_020_000_001),'write/reply'),
                (dict(elapsed=20_100_000,oldest_input_ns=1_000_000_001),'sample-age')):
            with self.subTest(reason=reason),self.assertRaisesRegex(RuntimeError,reason):
                self.admit(startup_allowed=True,**changes)
        self.assertEqual(self.budget.accepted_misses,0)
        self.admit(startup_allowed=True)
        with self.assertRaisesRegex(RuntimeError,'invalid startup allowance'):
            self.admit(1,startup_allowed=True)

    def test_first_miss_without_startup_selection_consumes_steady_budget(self):
        self.assertTrue(self.admit(elapsed=20_069_000)['allowance_used'])
        self.assertEqual(self.budget.accepted_misses,1)

    def test_noncausal_or_skipped_cycles_do_not_reset_budget(self):
        with self.assertRaisesRegex(RuntimeError,'nonsequential'):self.admit(1)
        with self.assertRaisesRegex(RuntimeError,'noncausal'):
            self.admit(last_reply_ns=1_020_000_001)
        self.assertEqual(self.budget.accepted_misses,0)


class SavedR5TimingTests(unittest.TestCase):
    def test_all_21_recorded_cycles_admit_one_post_reply_miss(self):
        # Sanitized scalar offsets from morning r5. Each tuple is oldest input,
        # final host write, final reply, output sample start, coordinator end;
        # all are ns from that cycle's actual start. No wire/UID data included.
        # This reclassifies saved timestamps, not a new hardware execution.
        offsets=[
            (199812,16566092,19346276,12128819,19985297),
            (205380,16237222,19023262,11895279,19627082),
            (198564,16216005,18999453,11833453,19608041),
            (200452,16257158,19013245,11852206,19600297),
            (204772,16212902,19071263,11860782,19618026),
            (198020,16282982,19175232,11847533,19640682),
            (198660,16212645,19006941,11782668,19430342),
            (197252,16303367,19169856,11924815,19761484),
            (204932,16181444,19004029,11851854,19543752),
            (202244,16114723,19011837,11758188,19567017),
            (196740,16215621,19190400,11837229,19664202),
            (198372,16357096,19255394,11906959,19786957),
            (198340,16090787,18996605,11730091,19556104),
            (209508,16106787,18991580,11759659,19501447),
            (231909,16135940,19055294,11819533,19493287),
            (201764,16203941,19011165,11841678,19617578),
            (202756,16189700,18996925,11820237,19564008),
            (196164,16139300,19007901,11774156,19519335),
            (239141,16180101,19022430,11804845,19603497),
            (242981,16188036,19001757,11830349,19610537),
            (212004,16352840,19165600,11970704,20002033),
        ]
        budget=PostReplyDeadlineBudget(settings())
        decisions=[]
        for index,offset in enumerate(offsets):
            begin=1_000_000_000+index*25_000_000
            first,written,replied,sampled,end=(begin+n for n in offset)
            decisions.append(budget.admit(index=index,begin_ns=begin,oldest_input_ns=first,
                final_write_ns=written,last_reply_ns=replied,output_sample_start_ns=sampled,
                checked_ns=end,sample_age_ns=20_000_000))
        self.assertEqual([i for i,r in enumerate(decisions) if r['allowance_used']],[20])
        self.assertEqual(decisions[-1]['lateness_ms'],.002033)
        self.assertEqual(budget.accepted_misses,1)


class ProfileGateTests(unittest.TestCase):
    setUp=fixtures.ProfileTests.setUp
    save=fixtures.ProfileTests.save

    def select(self):
        self.data.update(schema=live.SCHEMA_V3,telemetry_cadence=live.CADENCE_PRE_ENABLE,
            cadence_source_sha256=live.cadence_source_hashes(),duration_s=3.,
            startup_duration_s=.4,policy_ramp_s=.4,stop_duration_s=.4,
            post_reply_deadline_policy=settings())
        self.docs['hardware_review']['post_reply_deadline_acceptance']={
            'settings':settings(),'scope':self.data['scope'],
            'strict_50hz_not_established':True,'hard_output_and_freshness_limits_unchanged':True,
            'review':{**copy.deepcopy(self.data['review']),'decision':'ACCEPT_BOUNDED_POST_REPLY_DEADLINE'}}

    def test_default_is_strict_and_changed_setting_requires_new_review(self):
        self.assertIsNone(live.post_reply_deadline_settings(live.load_profile(self.base/'profile.json')))
        self.select()
        with self.assertRaisesRegex(live.ProfileError,'exact gains, limits'):
            live.load_profile(self.save())
        loaded=live.load_profile(self.save(bind_review=True))
        self.assertEqual(live.post_reply_deadline_settings(loaded),settings())
        self.assertFalse(loaded['actual_policy_output_20ms_verified'])

    def test_separate_acceptance_and_loader_proof_are_required(self):
        self.select()
        with self.assertRaisesRegex(live.ProfileError,'loader proof'):
            live.post_reply_deadline_settings({**self.data,'_post_reply_validation_token':True})
        del self.docs['hardware_review']['post_reply_deadline_acceptance']
        with self.assertRaisesRegex(live.ProfileError,'matching post-reply'):
            live.load_profile(self.save(bind_review=True))

    def test_caps_and_stale_review_cannot_be_relaxed(self):
        self.select()
        for key,value in (('max_lateness_ms',1.001),('max_lateness_ms',True),
                          ('max_consecutive_misses',2),('rolling_window_cycles',99),
                          ('max_misses_per_window',2)):
            old=self.data['post_reply_deadline_policy'][key]
            self.data['post_reply_deadline_policy'][key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):
                live.load_profile(self.save(bind_review=True))
            self.data['post_reply_deadline_policy'][key]=old
        self.data['post_reply_deadline_policy']['max_lateness_ms']=.5
        with self.assertRaisesRegex(live.ProfileError,'matching post-reply'):
            live.load_profile(self.save(bind_review=True))

    def test_hard_limits_and_supported_only_scope_remain_required(self):
        self.select()
        for key,value in (('hard_cycle_ms',21),('max_sample_age_ms',21),
                          ('max_sample_gap_ms',22),('duration_s',3.01),
                          ('max_consecutive_20ms_misses',1),('scope','ground_trial')):
            old=self.data[key];self.data[key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):
                live.load_profile(self.save(bind_review=True))
            self.data[key]=old

    def test_ground_plan_rejects_tolerance_before_hardware(self):
        plan,base,records=ground_fixtures.fixture()
        base['post_reply_deadline_policy']=settings()
        with self.assertRaisesRegex(ground_fixtures.GroundPlanError,'post-reply'):
            ground_fixtures.validate_ground_plan(plan,base,records)


class CoordinatorTests(unittest.TestCase):
    run_case=runtime_fixtures.OutputRuntimeTests.run_case

    def run_timing(self, *, delayed=None, selected=True, startup_allowed=False, front_options=None,
                   wake_late=False, late_reply=False, missing_reply=False, late_encoding=False):
        data=runtime_fixtures.measured_startup_profile()
        data.update(scope='supported_characterization_only',
            diagnostic_timing_acceptance=(live.MEASURED_R17_STARTUP_TIMING if startup_allowed else None))
        if selected:
            data.update(post_reply_deadline_policy=settings(),
                _post_reply_validation_token=live._POST_REPLY_VALIDATION_TOKEN)
        clock=runtime_fixtures.SimulatedClock()
        state={'index':None,'begin':None,'injected':set(),'wake':None}
        original_begin=runtime._PendingCycleTiming.begin
        def begin(pending,index,release,begun,previous_candidate,previous_sample):
            state.update(index=index,begin=begun)
            original_begin(pending,index,release,begun,previous_candidate,previous_sample)
            # Deterministic owner dispatch delay, within unchanged freshness.
            clock.advance(200_000)
        class Policy:
            def __call__(self,*args):return (.04,)*12
            @property
            def last_validation(self):
                index=state['index']
                if index in (delayed or {}) and index not in state['injected']:
                    clock.advance(max(0,state['begin']+delayed[index]-clock()))
                    state['injected'].add(index)
                return None
        class ReplySession(runtime_fixtures.FakeSession):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs);self.type1_batches=0
            def _exchange(self,wires,timeout_ns,send_only):
                clock.advance(500_000)
                rows,stats=super()._exchange(wires,timeout_ns,send_only)
                if self.ids[0]==1 and state['index']==1 and len(rows)==6 and all(
                        runtime.codec.ATParser().feed(bytes(r.tx))[0].kind==1 for r in rows):
                    self.type1_batches+=1
                    if late_reply and self.type1_batches==2:
                        late=state['begin']+20_050_000
                        rows[-1].received_ns=late;rows[-1].deadline_ns=late+1_000_000
                        clock.advance(max(0,late+10_000-clock()))
                    if missing_reply and self.type1_batches==2:
                        rows[-1].received=0
                return rows,stats
        front=ReplySession(1,clock=clock,**(front_options or {}))
        rear=ReplySession(7,clock=clock)
        sleeps=[0]
        def sleep(seconds):
            clock.sleep(seconds);sleeps[0]+=1
            if wake_late and sleeps[0]==3:
                clock.advance(15_000_000);state['wake']=clock()
        encoded_late=[False]
        original_encode=runtime_fixtures.encode_motion
        def encode(*args):
            result=original_encode(*args)
            if late_encoding and state['index']==1 and not encoded_late[0]:
                clock.advance(21_000_000);encoded_late[0]=True
            return result
        with patch.object(runtime._PendingCycleTiming,'begin',begin), \
             patch.object(runtime_fixtures,'encode_motion',encode):
            report,sessions=self.run_case(profile_data=data,front=front,rear=rear,
                imu=runtime_fixtures.FakeIMU(clock=clock),policy=Policy(),clock=clock,sleep=sleep)
        return report,sessions,state

    def test_rare_post_reply_miss_continues_without_catchup_or_50hz_claim(self):
        report,_,state=self.run_timing(delayed={2:20_010_000})
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertEqual(state['injected'],{2})
        self.assertEqual(report['post_reply_deadline_allowance_uses'],1)
        self.assertEqual(report['deadline20ms_misses'],1)
        self.assertFalse(report['startup_20ms_allowance_enabled'])
        self.assertFalse(report['full_controller_50Hz_verified'])
        self.assertTrue(report['stop_confirmed'])
        for left,right in zip(report['cycles'],report['cycles'][1:]):
            self.assertGreaterEqual(right['begin_ns'],max(left['begin_ns']+20_000_000,left['end_ns']))

    def test_startup_miss_does_not_spend_steady_allowance(self):
        report,_,state=self.run_timing(
            delayed={0:20_069_000,22:20_181_000},startup_allowed=True)
        self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
        self.assertEqual(state['injected'],{0,22})
        self.assertTrue(report['startup_20ms_allowance_enabled'])
        self.assertEqual(report['startup_20ms_allowance_uses'],1)
        self.assertEqual(report['post_reply_deadline_allowance_uses'],1)
        self.assertEqual(report['startup_20ms_misses'],1)
        self.assertEqual(report['steady_deadline20ms_misses'],1)
        self.assertEqual(report['deadline20ms_misses'],2)
        self.assertFalse(report['cycles'][0]['steady_deadline20ms_missed'])
        self.assertTrue(report['cycles'][22]['steady_deadline20ms_missed'])
        self.assertFalse(report['full_controller_50Hz_verified'])
        self.assertTrue(report['stop_confirmed'])

    def test_second_steady_miss_after_startup_rejects(self):
        report,_,state=self.run_timing(
            delayed={0:20_069_000,22:20_181_000,24:20_100_000},startup_allowed=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertEqual(state['injected'],{0,22,24})
        self.assertIn('rolling',str(report['errors']))
        self.assertEqual(report['startup_20ms_allowance_uses'],1)
        self.assertEqual(report['post_reply_deadline_allowance_uses'],1)
        self.assertEqual(len(report['post_reply_deadline_rejections']),1)
        self.assertTrue(report['stop_confirmed'])

    def test_strict_default_still_aborts_same_overrun(self):
        report,_,_=self.run_timing(delayed={2:20_010_000},selected=False)
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('Output cycle exceeded hard deadline',str(report['errors']))
        self.assertEqual(report['post_reply_deadline_allowance_uses'],0)

    def test_repeated_misses_stop_and_startup_cannot_add_another_waiver(self):
        for delayed,reason in (({0:20_010_000,1:20_010_000},'consecutive'),
                               ({0:20_010_000,2:20_010_000},'rolling')):
            with self.subTest(delayed=delayed):
                report,_,_=self.run_timing(delayed=delayed)
                self.assertEqual(report['status'],'ABORTED')
                self.assertIn(reason,str(report['errors']))
                self.assertEqual(report['post_reply_deadline_allowance_uses'],1)
                self.assertEqual(len(report['post_reply_deadline_rejections']),1)
                self.assertFalse(report['post_reply_deadline_rejections'][0]['accepted'])
                self.assertTrue(report['stop_confirmed'])

    def test_sample_age_and_late_reply_are_not_waived(self):
        for kwargs,reason in (({'delayed':{1:22_000_000}},'sample-age'),
                              ({'late_reply':True},'Incomplete or noncausal'),
                              ({'missing_reply':True},'Incomplete')):
            with self.subTest(kwargs=kwargs):
                report,_,_=self.run_timing(**kwargs)
                self.assertEqual(report['status'],'ABORTED')
                self.assertIn(reason,str(report['errors']))
                self.assertEqual(report['post_reply_deadline_allowance_uses'],0)
                self.assertTrue(report['stop_confirmed'])

    def test_late_encoding_stops_before_output_submission(self):
        report,_,_=self.run_timing(late_encoding=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('Encoded command exceeded',str(report['errors']))
        trace=report['failed_cycle_timing']
        self.assertEqual(trace['index'],1)
        self.assertIsNone(trace['output_submit_ns'])
        self.assertEqual(report['post_reply_deadline_allowance_uses'],0)
        self.assertTrue(report['stop_confirmed'])

    def test_old_hold_is_not_resent_after_delayed_wakeup(self):
        report,sessions,state=self.run_timing(wake_late=True)
        self.assertEqual(report['status'],'ABORTED')
        self.assertIn('gap exceeded before feedback hold',str(report['errors']))
        self.assertTrue(all(not any(t>=state['wake'] and kind==1 for t,kind,_,_ in s.calls)
                            for s in sessions.values()))

    def test_comm_fault_and_torque_violation_still_stop(self):
        for options,reason in (({'fail_motion':True},'USB lost'),
                                ({'high_returned_torque':True},'torque')):
            with self.subTest(options=options):
                report,_,_=self.run_timing(front_options=options)
                self.assertEqual(report['status'],'ABORTED')
                self.assertIn(reason,str(report['errors']))
                self.assertTrue(report['stop_confirmed'])


if __name__=='__main__':unittest.main()
