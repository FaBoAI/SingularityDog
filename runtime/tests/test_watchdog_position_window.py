"""Zero-gain selectable position window tests; no real actuator, audio, SSH or approval."""
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from singularitydog_hw import watchdog_commissioning as original
from test_watchdog_commissioning import Clock, FakeChannel

proposed = original

class DiagnosticProposalTests(unittest.TestCase):
    def run_case(self, *, mid=3, stage='active', overrides=None, stop_missing=False, position_window_deg=3):
        clock = Clock(); target = mid; phase = stage; values = overrides or {}
        class Injected(FakeChannel):
            def exchange(self, motor, step, center=0.):
                reply = super().exchange(motor, step, center)
                if motor == target and step == 'zero' and self.zero_counts[motor] == (2 if phase == 'active' else 3):
                    reply.update(values)
                return reply
            def stop_all(self):
                reply = super().stop_all()
                if stop_missing and target in self.ids:
                    reply.update(complete=False,confirmed_ids=[],unconfirmed_ids=list(self.ids),
                                 errors=['SYNTHETIC STOP missing'])
                return reply
        channels = {name:Injected(ids, clock) for name,ids in proposed.BUSES.items()}
        expected = {i:(bytes([i])*8).hex() for i in proposed.IDS}
        result = proposed.run(channels,expected,clock=clock,wait=clock.wait,position_window_deg=position_window_deg)
        return result,channels

    def test_active_failure_each_metric_saves_id_stage_signed_value_and_failed_checks(self):
        cases = ({'protocol_position_rad':math.radians(-3)-1e-6},
                 {'velocity_rad_s':-.50001}, {'temperature_c':60.}, {'temperature_c':-10.01})
        failed = ('position_delta_within_window','velocity_abs_within_0_5rad_s',
                  'temperature_below_60c','temperature_at_least_minus10c')
        for values,key in zip(cases,failed):
            with self.subTest(key=key):
                report,channels=self.run_case(overrides=values)
                self.assertEqual(report['status'],'ABORTED');self.assertTrue(report['stop_confirmed'])
                observation=report['axes']['3']['zero_gain_observations'][-1]
                self.assertEqual(observation['motor_id'],3)
                self.assertEqual(observation['phase'],'active_zero_before_silence')
                self.assertEqual(observation['failed_checks'],[key])
                self.assertIn('ID3 phase=active_zero_before_silence',report['errors'][0])
                self.assertIn(key,report['errors'][0])
                for field,value in values.items(): self.assertEqual(observation['reply'][field],value)
                self.assertFalse(report['axes']['3']['command_loss_tested'])
                self.assertTrue(all(c.stop_calls==1 for c in channels.values()))
                enabled=[i for c in channels.values() for _,i,step in c.calls if step=='enable']
                self.assertEqual(enabled,[1,2,3])
                self.assertFalse(report['positive_gain_sent']);self.assertFalse(report['learned_targets_sent'])
                json.dumps(report,allow_nan=False)

    def test_multiple_failed_checks_late_axis_and_no_next_enable(self):
        report,channels=self.run_case(mid=9,overrides={'protocol_position_rad':math.radians(5),
            'velocity_rad_s':-.8,'temperature_c':60.})
        observation=report['axes']['9']['zero_gain_observations'][-1]
        self.assertEqual(observation['failed_checks'],['position_delta_within_window',
            'velocity_abs_within_0_5rad_s','temperature_below_60c'])
        enabled=[i for c in channels.values() for _,i,step in c.calls if step=='enable']
        self.assertEqual(enabled,list(range(1,10)))
        self.assertTrue(all(c.stop_calls==1 for c in channels.values()))

    def test_phase_differentiates_disabled_silence_probe_and_preserves_old_stop_probe(self):
        report,channels=self.run_case(stage='silent',overrides={'velocity_rad_s':10.1312275883})
        axis=report['axes']['3'];observations=axis['zero_gain_observations']
        self.assertEqual([x['phase'] for x in observations],['active_zero_before_silence','disabled_zero_after_silence'])
        self.assertEqual(observations[0]['failed_checks'],[])
        self.assertEqual(observations[1]['failed_checks'],['velocity_abs_within_0_5rad_s'])
        self.assertEqual(axis['stop_probe'],observations[1]['reply'])
        self.assertEqual(axis['stop_probe']['mode_state'],0)
        self.assertEqual(axis['configured_timeout_ms'],200)
        self.assertIn('after command silence',report['errors'][0])
        self.assertFalse(axis['command_loss_tested']);self.assertFalse(axis['disabled_on_command_loss'])
        self.assertTrue(report['stop_confirmed'])

    def test_mode_failure_reply_is_preserved_before_early_guard_raises(self):
        report,_=self.run_case(overrides={'mode_state':0})
        observation=report['axes']['3']['zero_gain_observations'][-1]
        self.assertEqual(observation['reply']['mode_state'],0)
        self.assertEqual(observation['failed_checks'],['mode_state_expected'])
        self.assertIn('Zero-gain active reply not confirmed',report['errors'][0])
        self.assertTrue(report['stop_confirmed'])

    def test_inclusive_boundary_and_just_outside_keep_prior_acceptance(self):
        for stage in ('active','silent'):
            for temperature in (-10.,59.999):
                for speed in (-.5,.5):
                    report,_=self.run_case(stage=stage,overrides={'protocol_position_rad':math.radians(3),
                        'velocity_rad_s':speed,'temperature_c':temperature})
                    self.assertEqual(report['status'],'COMPLETE_COMMAND_LOSS_DIAGNOSTIC',report['errors'])
        for values in ({'protocol_position_rad':math.nextafter(math.radians(3),math.inf)},
                       {'velocity_rad_s':math.nextafter(.5,math.inf)},{'temperature_c':60.}):
            report,_=self.run_case(overrides=values)
            self.assertEqual(report['status'],'ABORTED')

    def test_stop_failure_keeps_observation_and_physical_poweroff_requirement(self):
        report,channels=self.run_case(overrides={'velocity_rad_s':.50001},stop_missing=True)
        self.assertEqual(report['status'],'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
        self.assertFalse(report['stop_confirmed'])
        self.assertEqual(report['axes']['3']['zero_gain_observations'][-1]['velocity_rad_s'],.50001)
        self.assertIn('Physically switch motor power Off',report['errors'][-1])
        self.assertTrue(all(c.stop_calls==1 for c in channels.values()))

    def test_delta_is_not_360_wrapped_or_certified_physical_motion(self):
        report,_=self.run_case(overrides={'protocol_position_rad':2*math.pi+.001})
        observation=report['axes']['3']['zero_gain_observations'][-1]
        self.assertAlmostEqual(observation['position_delta_deg'],360+math.degrees(.001))
        self.assertEqual(observation['position_basis'],'raw_protocol_reply_minus_initial_protocol_center')
        self.assertEqual(observation['failed_checks'],['position_delta_within_window'])
        self.assertFalse(report['approved_for_runtime'])


    def test_only_explicit_3_or_6_numeric_windows_nonfinite_bool_and_other_rejected(self):
        for value in (3,3.,6,6.):
            plan=proposed.plan(position_window_deg=value)
            self.assertEqual(plan['zero_gain_position_window_deg'],value)
            self.assertFalse(plan['positive_gains_available']);self.assertFalse(plan['learned_targets_available'])
        for value in (True,False,0,2,4,5,7,3.01,'6',None,float('nan'),float('inf'),-float('inf')):
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'position window'):
                proposed.plan(position_window_deg=value)
            with self.subTest(value=value),self.assertRaises(ValueError):
                self.run_case(position_window_deg=value)

    def test_5_54_deg_accepted_only_when_explicit6_and_recorded(self):
        for stage in ('active','silent'):
            for window,expected in ((3,'ABORTED'),(6,'COMPLETE_COMMAND_LOSS_DIAGNOSTIC')):
                report,_=self.run_case(stage=stage,position_window_deg=window,
                    overrides={'protocol_position_rad':math.radians(5.54)})
                self.assertEqual(report['status'],expected,report['errors'])
                self.assertEqual(report['zero_gain_position_window_deg'],window)
                obs=report['axes']['3']['zero_gain_observations'][0 if stage=='active' else -1]
                self.assertEqual(obs['limits']['position_delta_abs_max_deg'],window)
                self.assertAlmostEqual(obs['position_delta_deg'],5.54)
                self.assertTrue(report['stop_confirmed'])
                self.assertFalse(report['positive_gain_sent']);self.assertFalse(report['learned_targets_sent'])

    def test_6_deg_boundaries_inclusive_but_just_above6_both_signs_abort(self):
        for stage in ('active','silent'):
            for sign in (-1,1):
                limit=sign*math.radians(6)
                report,_=self.run_case(stage=stage,position_window_deg=6,
                    overrides={'protocol_position_rad':limit})
                self.assertEqual(report['status'],'COMPLETE_COMMAND_LOSS_DIAGNOSTIC',report['errors'])
                outside=math.nextafter(limit,sign*math.inf)
                report,channels=self.run_case(stage=stage,position_window_deg=6,
                    overrides={'protocol_position_rad':outside})
                self.assertEqual(report['status'],'ABORTED');self.assertTrue(report['stop_confirmed'])
                self.assertIn('position_delta_within_window',report['axes']['3']['zero_gain_observations'][-1]['failed_checks'])
                self.assertEqual([i for c in channels.values() for _,i,step in c.calls if step=='enable'],[1,2,3])

    def test_explicit6_never_changes_velocity_temperature_mode_fault_or_stop_failure(self):
        for values in ({'velocity_rad_s':.50001},{'temperature_c':60.},
                       {'temperature_c':-10.01},{'mode_state':0},{'fault_bits':1}):
            report,channels=self.run_case(position_window_deg=6,overrides=values)
            self.assertEqual(report['status'],'ABORTED');self.assertTrue(report['stop_confirmed'])
            self.assertEqual([i for c in channels.values() for _,i,step in c.calls if step=='enable'],[1,2,3])
            self.assertTrue(all(c.stop_calls==1 for c in channels.values()))
        report,_=self.run_case(position_window_deg=6,overrides={'velocity_rad_s':.50001},stop_missing=True)
        self.assertEqual(report['status'],'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')

    def test_explicit6_preserves_enable_timeout_and_watchdog_disable_time_guards(self):
        for bad in ('enable_timeout','late','late_write_completion','no_disable'):
            clock=Clock();channels={name:FakeChannel(ids,clock,bad=bad if name=='front' else None)
                                   for name,ids in proposed.BUSES.items()}
            expected={i:(bytes([i])*8).hex() for i in proposed.IDS}
            report=proposed.run(channels,expected,clock=clock,wait=clock.wait,position_window_deg=6)
            self.assertEqual(report['status'],'ABORTED');self.assertTrue(report['stop_confirmed'])
            self.assertEqual([i for c in channels.values() for _,i,step in c.calls if step=='enable'],[1])
            self.assertTrue(all(c.stop_calls==1 for c in channels.values()))

    def test_default_and_explicit6_cli_plan_only_never_open_channels(self):
        import io,tempfile
        from contextlib import redirect_stdout
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'uids.json';p.write_text(json.dumps({str(i):(bytes([i])*8).hex() for i in proposed.IDS}))
            for flags,window in (([],3),(['--position-window-deg','6'],6)):
                out=io.StringIO()
                with redirect_stdout(out),patch.object(proposed,'Channel') as channel:
                    self.assertEqual(proposed.main(['--expected-uids',str(p),*flags]),0)
                    channel.assert_not_called()
                plan=json.loads(out.getvalue());self.assertEqual(plan['zero_gain_position_window_deg'],window)
                self.assertFalse(plan['hardware_opened'])
            for value in ('4','nan','true','6.0'):
                with patch('sys.stderr',io.StringIO()),self.assertRaises(SystemExit):
                    proposed.main(['--expected-uids',str(p),'--position-window-deg',value])


if __name__=='__main__':
    unittest.main()
