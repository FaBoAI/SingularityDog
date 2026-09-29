"""Explicit startup review; synthetic timing is not robot evidence."""
import copy
import unittest
from unittest.mock import patch
from singularitydog_hw import policy_live_profile as live
from singularitydog_hw.policy_post_reply_timing import PostReplyDeadlineBudget, POST_REPLY_POLICY
import test_policy_rare_jitter_profile as rare
import test_policy_supported_extension_profile as extension
import test_policy_output_runtime as runtime_fixture


class FirstCycleProfileTests(unittest.TestCase):
    def setUp(self):
        rare.RareJitterProfileTests.setUp(self)
        self.data['startup_cycle_allowance'] = live.FIRST_CYCLE_POST_REPLY
        self.docs['hardware_review']['startup_cycle_acceptance'] = dict(
            mode=live.FIRST_CYCLE_POST_REPLY, scope=self.data['scope'], first_cycle_only=True,
            hard_output_and_freshness_limits_unchanged=True, steady_miss_budget_unchanged=True,
            review={**self.data['review'], 'decision':'ACCEPT_FIRST_CYCLE_POST_REPLY'})

    def seal(self):
        return rare.RareJitterProfileTests.seal(self)

    def test_loader_proof_and_named_review_are_required(self):
        with self.assertRaisesRegex(live.ProfileError, 'loader review proof'):
            live.reviewed_startup_cycle_allowance(self.data)
        loaded=live.load_profile(self.seal())
        self.assertTrue(live.reviewed_startup_cycle_allowance(loaded))
        accepted=self.docs['hardware_review']['startup_cycle_acceptance']
        for key in ('first_cycle_only','hard_output_and_freshness_limits_unchanged','steady_miss_budget_unchanged'):
            accepted[key]=False
            with self.subTest(key=key),self.assertRaises(live.ProfileError):live.load_profile(self.seal())
            accepted[key]=True
        del self.docs['hardware_review']['startup_cycle_acceptance']
        with self.assertRaises(live.ProfileError):live.load_profile(self.seal())

    def test_wrong_route_or_hard_limit_cannot_select_exception(self):
        for key,value in (('startup_cycle_allowance',True),('hard_cycle_ms',21.),
                          ('max_sample_age_ms',20.001),('diagnostic_timing_acceptance',live.SUPPORTED_POLICY_PROBE_5S)):
            old=self.data[key];self.data[key]=value
            with self.subTest(key=key),self.assertRaises(live.ProfileError):live.load_profile(self.seal())
            self.data[key]=old

    def test_recorded_r50_timestamps_keep_hard_limits_and_one_steady_miss(self):
        budget=PostReplyDeadlineBudget(self.data['post_reply_deadline_policy'])
        rows=(dict(index=0,begin_ns=4533775212788,oldest_input_ns=4533775437723,
                   final_write_ns=4533791938340,last_reply_ns=4533794850715,
                   output_sample_start_ns=4533787100000,checked_ns=4533795323465),
              dict(index=1,begin_ns=4533795383786,oldest_input_ns=4533795612593,
                   final_write_ns=4533811870740,last_reply_ns=4533814744777,
                   output_sample_start_ns=4533807100000,checked_ns=4533815451678))
        # Intermediate output sample stamps are conservative causal test values;
        # begin/oldest/final-write/last-reply/checked are the recorded evidence.
        for row in rows:
            result=budget.admit(**row,sample_age_ns=20_000_000,startup_allowed=row['index']==0)
            self.assertEqual(result['startup_allowance_used'],row['index']==0)
        row={**rows[1],'index':2}
        for k in tuple(row):
            if k.endswith('_ns'):row[k]+=20_000_000
        with self.assertRaisesRegex(RuntimeError,'consecutive miss'):
            budget.admit(**row,sample_age_ns=20_000_000)
        for key,value in (('last_reply_ns',rows[0]['begin_ns']+20_000_001),
                          ('checked_ns',rows[0]['oldest_input_ns']+20_000_001)):
            b=PostReplyDeadlineBudget(self.data['post_reply_deadline_policy'])
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                b.admit(**{**rows[0],key:value},sample_age_ns=20_000_000,startup_allowed=True)

    def test_runtime_uses_reviewed_startup_choice_but_rejects_late_reply(self):
        original=runtime_fixture.measured_startup_profile
        def synthetic_reviewed():
            p=original()
            p.update(scope='supported_characterization_only',
                diagnostic_timing_acceptance=live.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
                startup_cycle_allowance=live.FIRST_CYCLE_POST_REPLY,
                post_reply_deadline_policy=copy.deepcopy(self.data['post_reply_deadline_policy']),
                _startup_cycle_token=live._STARTUP_CYCLE_TOKEN,
                _post_reply_validation_token=live._POST_REPLY_VALIDATION_TOKEN)
            return p
        case=runtime_fixture.OutputRuntimeTests()
        with patch.object(runtime_fixture,'measured_startup_profile',side_effect=synthetic_reviewed):
            report,_,_=case.run_measured_startup_timing_case(delayed_cycle=0)
            self.assertEqual(report['status'],'COMPLETE_SUPPORTED_OUTPUT',report['errors'])
            self.assertTrue(report['startup_20ms_allowance_enabled'])
            self.assertTrue(report['cycles'][0]['startup_20ms_allowance_used'])
            self.assertEqual(report['post_reply_deadline_allowance_uses'],0)
            failed,_,_=case.run_measured_startup_timing_case(late_reply=True)
            self.assertEqual(failed['status'],'ABORTED')
            self.assertTrue(failed['stop_confirmed'])


class FirstCycleExtensionTests(extension.SupportedExtensionProfileTests):
    def test_only_first_startup_exception_can_qualify_for_extension(self):
        prior=self.docs['prior_supported_profile']
        self.data['startup_cycle_allowance']=prior['startup_cycle_allowance']=live.FIRST_CYCLE_POST_REPLY
        self.docs['hardware_review']['startup_cycle_acceptance']=dict(
            mode=live.FIRST_CYCLE_POST_REPLY,scope=self.data['scope'],first_cycle_only=True,
            hard_output_and_freshness_limits_unchanged=True,steady_miss_budget_unchanged=True,
            review={**self.data['review'],'decision':'ACCEPT_FIRST_CYCLE_POST_REPLY'})
        report=self.docs['prior_supported_report'];row=report['cycles'][0]
        report.update(startup_20ms_allowance_enabled=True,startup_20ms_allowance_uses=1,
                      deadline20ms_misses=1,steady_deadline20ms_misses=0)
        row['end_ns']=row['begin_ns']+20_100_000
        row['post_reply_deadline'].update(checked_ns=row['end_ns'],startup_allowance_used=True)
        # Keep the following cycle causal after first-cycle overrun.
        for k in ('begin_ns','output_reply_end_ns','end_ns'):report['cycles'][1][k]+=200_000
        report['cycles'][1]['post_reply_deadline']['checked_ns']=report['cycles'][1]['end_ns']
        self.assertTrue(self.load()['output_allowed'])
        report['steady_deadline20ms_misses']=1
        with self.assertRaises(live.ProfileError):self.load()


if __name__=='__main__':unittest.main()
