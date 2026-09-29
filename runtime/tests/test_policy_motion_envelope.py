"""Pure motion-envelope properties.  No device, model, or CAN transport imports."""

from dataclasses import replace
import math
import random
import unittest

from singularitydog_hw.policy_motion_envelope import (
    AxisLimits, MotionFault, MotionSample, PolicyMotionEnvelope,
)
from singularitydog_hw import policy_motion_envelope as motion


def limits(**changes):
    return replace(AxisLimits(-1., 1., 4., .2, .4, 1.5, .5, 2., 3., 60., 3., .5), **changes)


def sample(t, q=0., v=0., torque=0., temperature=25.):
    def vector(value):
        return tuple(value) if isinstance(value, (list, tuple)) else (value,) * 12
    return MotionSample(vector(q), vector(v), vector(torque), vector(temperature), t)


def envelope(axis=None, initial=None, **changes):
    options = dict(now_s=1., startup_duration_s=.5, stop_duration_s=.4,
                   max_sample_age_s=.01, max_sample_gap_s=.03)
    options.update(changes)
    return PolicyMotionEnvelope((axis or limits(),) * 12, initial or sample(1.), **options)


class MotionEnvelopeTests(unittest.TestCase):
    def test_ramp_roundoff_cannot_make_startup_exceed_cap_or_stop_gain_negative(self):
        fractions=[0.,1.,math.nextafter(1.,0.),.9999999979]
        fractions.extend(1.-i*1e-10 for i in range(1,10000))
        for fraction in fractions:
            ramp=motion._quintic_fraction(fraction)
            self.assertGreaterEqual(ramp,0.)
            self.assertLessEqual(ramp,1.)
        controller=envelope(startup_duration_s=1.,stop_duration_s=1.,max_sample_gap_s=2.)
        controller.step((0.,)*12,sample(2.),now_s=2.)
        controller.request_stop()
        t=2.+.9999999979
        command=controller.step(None,sample(t),now_s=t)
        self.assertTrue(all(0.<=v<=4. for v in command.kp))
        self.assertTrue(all(0.<=v<=.2 for v in command.kd))

    def test_separate_damping_ramp_and_early_stop_are_continuous_and_bounded(self):
        controller=envelope(axis=limits(kp=3.,kd=.15,max_estimated_pd_torque_nm=.1),
                            startup_duration_s=.4,startup_damping_duration_s=.08)
        previous=None
        for tick in range(1,5):
            t=1.+tick*.02
            command=controller.step((0.,)*12,sample(t,v=.2),now_s=t)
            self.assertLess(command.kp[0],3.)
            self.assertLessEqual(command.kd[0],.15)
            self.assertGreater(command.kd[0]/.15,command.kp[0]/3.)
            self.assertAlmostEqual(command.estimated_pd_torque_nm[0],-.2*command.kd[0])
            if previous:self.assertGreaterEqual(command.kd[0],previous.kd[0])
            previous=command
        self.assertEqual(previous.kd[0],.15)
        controller.request_stop()
        for tick in range(5,26):
            t=1.+tick*.02
            command=controller.step(None,sample(t,v=.2),now_s=t)
            self.assertLessEqual(command.kd[0],previous.kd[0])
            self.assertLessEqual(command.kp[0],previous.kp[0])
            self.assertEqual(command.q_model_rad,(0.,)*12)
            previous=command
        self.assertEqual(command.kd[0],0.)
        self.assertEqual(command.kp[0],0.)

    def test_early_damping_preserves_torque_budget(self):
        controller=envelope(axis=limits(kd=.15,max_estimated_pd_torque_nm=.001),
                            startup_duration_s=.4,startup_damping_duration_s=.08)
        with self.assertRaisesRegex(MotionFault,'estimated PD torque budget'):
            controller.step((0.,)*12,sample(1.02,v=.2),now_s=1.02)
        with self.assertRaisesRegex(MotionFault,'Damping ramp'):
            envelope(startup_duration_s=.4,startup_damping_duration_s=.41)

    def test_float_tuple_fast_path_preserves_generic_vector_acceptance_and_failures(self):
        def original(values, name):
            if isinstance(values, (str, bytes)):
                raise MotionFault(f"{name}: twelve numeric values required")
            try:
                result = tuple(motion._number(value, name) for value in values)
            except TypeError as exc:
                raise MotionFault(f"{name}: twelve numeric values required") from exc
            if len(result) != 12:
                raise MotionFault(f"{name}: exactly twelve axes required")
            return result

        class FloatChild(float):
            pass

        finite = (0., -0., 1e-300, -1e300) + (1.25,) * 8
        cases = (
            finite,
            tuple(float(index) for index in range(12)),
            tuple(range(12)),
            (FloatChild(.25),) + (0.,) * 11,
            [float(index) for index in range(12)],
            (math.nan,) + (0.,) * 11,
            (math.inf,) + (0.,) * 11,
            (-math.inf,) + (0.,) * 11,
            (True,) + (0.,) * 11,
            (None,) + (0.,) * 11,
            (0.,) * 11,
            (0.,) * 13,
            [0.] * 11,
            "not a vector",
            b"not a vector",
            None,
        )
        for case in cases:
            with self.subTest(case=repr(case)):
                try:
                    expected = original(case, "vector")
                except MotionFault as error:
                    with self.assertRaises(MotionFault) as caught:
                        motion._vector(case, "vector")
                    self.assertEqual(str(caught.exception), str(error))
                else:
                    actual = motion._vector(case, "vector")
                    self.assertEqual(actual, expected)
                    self.assertTrue(all(type(value) is float for value in actual))
        self.assertEqual(motion._vector(iter(range(12)), "vector"),
                         original(iter(range(12)), "vector"))
        self.assertIs(motion._vector(finite, "vector"), finite)
        sample_value = MotionSample(finite, finite, finite, finite, 1.)
        self.assertEqual(sample_value.q_model_rad, finite)

    def test_dataclasses_validate_dimensions_finite_gains_and_limits(self):
        for changes in ({"kp": -1}, {"kd": math.nan}, {"lower_rad": 1.},
                        {"max_command_acceleration_rad_s2": 0},
                        {"max_temperature_c": True}, {"kp": 10 ** 1000}):
            with self.subTest(changes=changes), self.assertRaises(MotionFault):
                limits(**changes)
        for args in (([0.] * 11, [0.] * 12, [0.] * 12, [25.] * 12, 1.),
                     ([math.inf] * 12, [0.] * 12, [0.] * 12, [25.] * 12, 1.),
                     ([0.] * 12, [0.] * 12, [0.] * 12, [25.] * 12, -1.)):
            with self.assertRaises(MotionFault):
                MotionSample(*args)
        with self.assertRaises(MotionFault):
            PolicyMotionEnvelope([limits()] * 11, sample(1.), now_s=1.,
                startup_duration_s=.5, stop_duration_s=.4,
                max_sample_age_s=.01, max_sample_gap_s=.03)

    def test_startup_uses_current_position_and_quintic_gains(self):
        controller = envelope(initial=sample(1., q=.13))
        command = None
        for cycle in range(1, 26):
            t = 1. + cycle * .02
            command = controller.step((.13,) * 12, sample(t, q=.13), now_s=t)
            fraction = min(1., cycle * .02 / .5)
            expected = fraction ** 3 * (10 + fraction * (-15 + 6 * fraction))
            self.assertAlmostEqual(command.gain_scale, expected)
            self.assertEqual(command.q_model_rad, (.13,) * 12)
            self.assertEqual(command.command_velocity_rad_s, (0.,) * 12)
            self.assertEqual(command.velocity_reference_rad_s, (0.,) * 12)
            self.assertEqual(command.feedforward_torque_nm, (0.,) * 12)
        self.assertEqual(command.phase, "active")
        self.assertEqual(command.kp, (4.,) * 12)

    def test_valid_targets_rate_limited_without_modifying_desired_endpoint(self):
        controller = envelope()
        previous_q, previous_v = (0.,) * 12, (0.,) * 12
        for cycle in range(1, 151):
            t = 1. + cycle * .02
            command = controller.step((.3,) * 12, sample(t, q=previous_q), now_s=t)
            for q, v, old_q, old_v in zip(command.q_model_rad, command.command_velocity_rad_s,
                                         previous_q, previous_v):
                self.assertLessEqual(abs(q - old_q), .4 * .02 + 1e-12)
                self.assertLessEqual(abs(v - old_v), 1.5 * .02 + 1e-12)
                self.assertLessEqual(q, .3 + 1e-12)
            previous_q, previous_v = command.q_model_rad, command.command_velocity_rad_s
        self.assertEqual(command.q_model_rad, (.3,) * 12)
        self.assertEqual(command.command_velocity_rad_s, (0.,) * 12)

    def test_random_reversals_preserve_velocity_acceleration_and_range(self):
        randomizer = random.Random(7921)
        axis = limits(lower_rad=-.2, upper_rad=.2, max_displacement_from_start_rad=.2)
        controller = envelope(axis)
        previous_q, previous_v = (0.,) * 12, (0.,) * 12
        t = 1.
        target = (.2,) * 12
        for cycle in range(1800):
            dt = randomizer.uniform(.005, .025)
            t += dt
            if cycle % 7 == 0:
                target = tuple(randomizer.choice((-.2, .2, randomizer.uniform(-.2, .2)))
                               for _ in range(12))
            command = controller.step(target, sample(t, q=previous_q), now_s=t)
            for q, v, old_q, old_v in zip(command.q_model_rad, command.command_velocity_rad_s,
                                         previous_q, previous_v):
                self.assertLessEqual(abs(q), .2 + 1e-12)
                self.assertLessEqual(abs(v), .4 + 1e-12)
                self.assertLessEqual(abs(v - old_v), 1.5 * dt + 1e-12)
                self.assertLessEqual(abs(q - old_q), .4 * dt + 1e-12)
                stopping_point = q + v * abs(v) / (2 * 1.5)
                self.assertLessEqual(abs(stopping_point), .2 + 1e-12)
            previous_q, previous_v = command.q_model_rad, command.command_velocity_rad_s

    def test_target_inside_braking_distance_reverses_continuously(self):
        controller = envelope()
        q, v = (0.,) * 12, (0.,) * 12
        for cycle in range(1, 21):
            t = 1. + cycle * .02
            command = controller.step((.4,) * 12, sample(t, q=q), now_s=t)
            q, v = command.q_model_rad, command.command_velocity_rad_s
        new_target = q[0] - .01
        turning_point = q[0] + v[0] ** 2 / 3.
        maxima = q[0]
        for cycle in range(21, 121):
            t = 1. + cycle * .02
            command = controller.step((new_target,) * 12, sample(t, q=q), now_s=t)
            self.assertLessEqual(abs(command.command_velocity_rad_s[0] - v[0]), .03 + 1e-12)
            q, v = command.q_model_rad, command.command_velocity_rad_s
            maxima = max(maxima, q[0])
        self.assertLessEqual(maxima, turning_point + 1e-12)
        self.assertEqual(q, (new_target,) * 12)
        self.assertEqual(v, (0.,) * 12)

    def test_stop_brakes_before_gain_ramp_and_cannot_restart(self):
        controller = envelope()
        q, v = (0.,) * 12, (0.,) * 12
        for cycle in range(1, 31):
            t = 1. + cycle * .02
            command = controller.step((.45,) * 12, sample(t, q=q), now_s=t)
            q, v = command.q_model_rad, command.command_velocity_rad_s
        stop_start = t
        expected = tuple(pos + vel * abs(vel) / 3. for pos, vel in zip(q, v))
        controller.request_stop()
        stage_seen, last_gain = set(), command.gain_scale
        for cycle in range(1, 61):
            t = stop_start + cycle * .02
            command = controller.step(None, sample(t, q=q), now_s=t)
            stage_seen.add(command.stop_stage)
            self.assertLessEqual(command.gain_scale, last_gain + 1e-12)
            for speed, old_speed in zip(command.command_velocity_rad_s, v):
                self.assertLessEqual(abs(speed - old_speed), .03 + 1e-12)
            if command.stop_stage == "braking":
                self.assertEqual(command.gain_scale, 1.)
            q, v, last_gain = command.q_model_rad, command.command_velocity_rad_s, command.gain_scale
        self.assertEqual(stage_seen, {"braking", "gain_ramp", "complete"})
        self.assertEqual(command.phase, "stopped")
        self.assertEqual(command.gain_scale, 0.)
        self.assertEqual(q, expected)
        self.assertEqual(v, (0.,) * 12)
        t += .02
        ignored = controller.step((-.4,) * 12, sample(t, q=q), now_s=t)
        self.assertEqual(ignored.q_model_rad, expected)
        self.assertEqual(ignored.phase, "stopped")
        self.assertLess(controller.maximum_graceful_stop_s, 1.)

    def test_stop_during_startup_does_not_raise_gain(self):
        controller = envelope()
        command = controller.step((.1,) * 12, sample(1.02), now_s=1.02)
        gain = command.gain_scale
        controller.request_stop()
        for cycle in range(2, 31):
            t = 1. + cycle * .02
            command = controller.step(None, sample(t, q=command.q_model_rad), now_s=t)
            self.assertLessEqual(command.gain_scale, gain + 1e-12)
            gain = command.gain_scale
        self.assertEqual(command.phase, "stopped")

    def test_twelve_independent_limits_and_boundary_stop(self):
        rows = tuple(limits(lower_rad=-.2, upper_rad=.2,
                            max_command_velocity_rad_s=.1 + index * .01,
                            max_command_acceleration_rad_s2=.5 + index * .1,
                            max_displacement_from_start_rad=.2)
                     for index in range(12))
        controller = PolicyMotionEnvelope(rows, sample(1.), now_s=1.,
            startup_duration_s=.5, stop_duration_s=.4,
            max_sample_age_s=.01, max_sample_gap_s=.03)
        q, velocity = (0.,) * 12, (0.,) * 12
        target = tuple(.2 if index % 2 else -.2 for index in range(12))
        for cycle in range(1, 81):
            t = 1. + cycle * .02
            command = controller.step(target, sample(t, q=q), now_s=t)
            for mid, row in enumerate(rows):
                self.assertLessEqual(abs(command.command_velocity_rad_s[mid]),
                                     row.max_command_velocity_rad_s + 1e-12)
                self.assertLessEqual(abs(command.command_velocity_rad_s[mid] - velocity[mid]),
                                     row.max_command_acceleration_rad_s2 * .02 + 1e-12)
            q, velocity = command.q_model_rad, command.command_velocity_rad_s
        controller.request_stop()
        stop_start = t
        for cycle in range(1, 61):
            t = stop_start + cycle * .02
            command = controller.step(None, sample(t, q=q), now_s=t)
            q = command.q_model_rad
            self.assertTrue(all(-.2 - 1e-12 <= value <= .2 + 1e-12 for value in q))
        self.assertEqual(command.phase, "stopped")

    def test_one_invalid_axis_prevents_any_partially_advanced_command(self):
        controller = envelope()
        target = [.1] * 12
        target[10] = .6
        with self.assertRaisesRegex(MotionFault, "ID11"):
            controller.step(target, sample(1.02), now_s=1.02)
        self.assertEqual(controller._q, (0.,) * 12)
        self.assertEqual(controller._gain, 0.)

    def test_invalid_targets_are_rejected_not_clipped_and_fault_is_latched(self):
        for target in ([0.] * 11, [math.nan] * 12, [math.inf] * 12,
                       [.50001] * 12, [-.50001] * 12, [True] * 12):
            with self.subTest(target=target):
                controller = envelope()
                with self.assertRaises(MotionFault):
                    controller.step(target, sample(1.02), now_s=1.02)
                original = controller.fault_reason
                self.assertEqual(controller.phase, "faulted")
                with self.assertRaises(MotionFault) as caught:
                    controller.step([0.] * 12, sample(1.04), now_s=1.04)
                self.assertEqual(str(caught.exception), original)
                with self.assertRaises(MotionFault):
                    controller.request_stop()

    def test_sample_limits_are_hard_faults_including_during_stopping(self):
        cases = ((sample(1.02, q=1.01), "joint range"),
                 (sample(1.02, q=.50001), "displacement"),
                 (sample(1.02, v=-2.01), "velocity"),
                 (sample(1.02, torque=-3.01), "measured torque"),
                 (sample(1.02, temperature=60.01), "temperature"))
        for measured, text in cases:
            with self.subTest(text=text):
                controller = envelope()
                controller.request_stop()
                with self.assertRaisesRegex(MotionFault, text):
                    controller.step(None, measured, now_s=1.02)

    def test_tracking_and_estimated_pd_budget_are_independent_of_feedback_torque(self):
        controller = envelope(limits(max_tracking_error_rad=.001))
        with self.assertRaisesRegex(MotionFault, "tracking"):
            for cycle in range(1, 6):
                t = 1. + cycle * .02
                controller.step((.2,) * 12, sample(t), now_s=t)
        controller = envelope(limits(kp=20., max_estimated_pd_torque_nm=.01))
        with self.assertRaisesRegex(MotionFault, "estimated PD"):
            for cycle in range(1, 30):
                t = 1. + cycle * .02
                controller.step((.3,) * 12, sample(t, torque=0.), now_s=t)

    def test_pd_estimate_uses_zero_velocity_reference_and_no_feedforward(self):
        controller = envelope(limits(kp=0., kd=1.))
        command = None
        for cycle in range(1, 26):
            t = 1. + cycle * .02
            command = controller.step((0.,) * 12, sample(t, v=.2), now_s=t)
        self.assertEqual(command.estimated_pd_torque_nm, (-.2,) * 12)
        self.assertEqual(command.velocity_reference_rad_s, (0.,) * 12)

    def test_initial_stale_future_and_overlimit_samples_rejected(self):
        for initial in (sample(.9), sample(1.01), sample(1., torque=3.01)):
            with self.assertRaises(MotionFault):
                envelope(initial=initial)

    def test_causal_timing_repeat_future_stale_and_gap_are_latched(self):
        cases = ((1.02, sample(1.)), (1., sample(1.)), (1.02, sample(1.03)),
                 (1.02, sample(1.001)), (1.04, sample(1.04)), (math.nan, sample(1.02)))
        for now, measured in cases:
            with self.subTest(now=now, sample=measured.monotonic_s):
                controller = envelope()
                with self.assertRaises(MotionFault):
                    controller.step((0.,) * 12, measured, now_s=now)
                self.assertIsNotNone(controller.fault_reason)

    def test_fresh_regular_samples_do_not_hide_a_late_command_interval(self):
        controller=envelope(now_s=1.010,initial=sample(1.),
                            max_sample_age_s=.020,max_sample_gap_s=.021)
        controller.step((0.,)*12,sample(1.020),now_s=1.030)
        # A fresh input and a 20.048ms sample interval do not waive a 21.5ms
        # command interval. These reproduce the observed separate timings,
        # without claiming the missing hardware timestamp was exactly 21.5ms.
        with self.assertRaisesRegex(MotionFault,'Command gap exceeded') as caught:
            controller.step((0.,)*12,sample(1.040048),now_s=1.0515)
        self.assertIn('command_interval_ms=21.500000',str(caught.exception))
        self.assertIn('sample_interval_ms=20.048000',str(caught.exception))
        self.assertIn('limit_ms=21.000000',str(caught.exception))
        with self.assertRaises(MotionFault) as repeated:
            controller.step((0.,)*12,sample(1.06),now_s=1.061)
        self.assertEqual(str(repeated.exception),str(caught.exception))

    def test_regular_commands_do_not_hide_a_late_sample_interval(self):
        controller=envelope(now_s=1.010,initial=sample(1.),
                            max_sample_age_s=.020,max_sample_gap_s=.021)
        controller.step((0.,)*12,sample(1.019),now_s=1.030)
        with self.assertRaisesRegex(MotionFault,'Sample gap exceeded') as caught:
            controller.step((0.,)*12,sample(1.041),now_s=1.050)
        self.assertIn('command_interval_ms=20.000000',str(caught.exception))
        self.assertIn('sample_interval_ms=22.000000',str(caught.exception))
        self.assertIn('limit_ms=21.000000',str(caught.exception))
        self.assertEqual(controller.fault_reason,str(caught.exception))

    def test_explicit_emergency_is_immediate_and_cannot_be_overwritten(self):
        controller = envelope()
        with self.assertRaisesRegex(MotionFault, "external watchdog"):
            controller.emergency_fault("external watchdog")
        with self.assertRaisesRegex(MotionFault, "external watchdog"):
            controller.emergency_fault("second reason")
        with self.assertRaisesRegex(MotionFault, "external watchdog"):
            controller.step((0.,) * 12, sample(1.02), now_s=1.02)


if __name__ == "__main__":
    unittest.main()
