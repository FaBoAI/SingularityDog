"""Offline byte and fault parity for the isolated CPython encoder candidate."""

import copy
import importlib.util
import math
import os
from pathlib import Path
import random
from types import SimpleNamespace
import unittest

from singularitydog_hw.can_readonly import ATParser

from benchmark import baseline, fixture


def load_extension(path):
    spec = importlib.util.spec_from_file_location('sdbe_native', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(module, offsets, axes, initial):
    specs = tuple((offsets[mid], axes[mid]['sign'], axes[mid]['lower_rad'],
                   axes[mid]['upper_rad'], initial[mid - 1],
                   axes[mid]['max_displacement_from_start_rad'],
                   axes[mid]['max_estimated_pd_torque_nm']) for mid in range(1, 13))
    return module.make_context(specs)


def with_change(command, name, axis, value):
    row = list(getattr(command, name))
    row[axis - 1] = value
    return SimpleNamespace(**{key: tuple(row) if key == name else getattr(command, key)
                              for key in ('q_model_rad', 'kp', 'kd',
                                          'estimated_pd_torque_nm')})


class BatchEncodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = os.environ.get('SDBE_EXTENSION')
        if not path:
            raise unittest.SkipTest('Set SDBE_EXTENSION to an offline-built candidate')
        cls.module = load_extension(Path(path))

    def setUp(self):
        self.command, self.offsets, self.axes, self.initial = fixture()

    def native(self, command=None, *, axes=None, offsets=None, initial=None):
        axes = self.axes if axes is None else axes
        offsets = self.offsets if offsets is None else offsets
        initial = self.initial if initial is None else initial
        return self.module.encode(context(self.module, offsets, axes, initial),
                                  self.command if command is None else command)

    def assert_same_result(self, command, *, axes=None, offsets=None, initial=None):
        axes = self.axes if axes is None else axes
        offsets = self.offsets if offsets is None else offsets
        initial = self.initial if initial is None else initial
        try:
            expected = baseline(command, offsets, axes, initial)
        except (ValueError, RuntimeError) as error:
            with self.assertRaises(type(error)) as caught:
                self.native(command, axes=axes, offsets=offsets, initial=initial)
            self.assertEqual(str(caught.exception), str(error))
        else:
            self.assertEqual(self.native(command, axes=axes,
                                         offsets=offsets, initial=initial), expected)

    def test_random_exact_bytes_and_all_twelve_ids(self):
        rng = random.Random(92029)
        for case in range(2000):
            command = SimpleNamespace(
                q_model_rad=tuple(q + rng.uniform(-.015, .015) for q in self.initial),
                kp=tuple(rng.uniform(0., 10.) for _ in range(12)),
                kd=tuple(rng.uniform(0., .5) for _ in range(12)),
                estimated_pd_torque_nm=tuple(rng.uniform(-.01, .01)
                                             for _ in range(12)))
            with self.subTest(case=case):
                self.assert_same_result(command)
        wires = self.native()
        self.assertEqual({ATParser().feed(wire)[0].destination for bus in wires.values()
                          for wire in bus}, set(range(1, 13)))

    def test_quantization_boundaries_and_gain_endpoints(self):
        axes = {mid: dict(sign=1, lower_rad=-13., upper_rad=13.,
                          max_displacement_from_start_rad=20.,
                          max_estimated_pd_torque_nm=100.) for mid in range(1, 13)}
        offsets = {mid: 0. for mid in range(1, 13)}
        initial = (0.,) * 12
        for code in (0, 1, 2, 32766, 32767, 32768, 65533, 65534, 65535):
            raw = code * 25.14 / 65535 - 12.57
            for q in (raw, math.nextafter(raw, -math.inf),
                      math.nextafter(raw, math.inf)):
                if not -12.57 <= q <= 12.57:
                    continue
                for kp, kd in ((0., 0.), (36., 1.), (3., .15)):
                    command = SimpleNamespace(q_model_rad=(q,) + (0.,) * 11,
                                              kp=(kp,) * 12, kd=(kd,) * 12,
                                              estimated_pd_torque_nm=(0.,) * 12)
                    with self.subTest(code=code, q=q, kp=kp, kd=kd):
                        self.assert_same_result(command, axes=axes,
                                                offsets=offsets, initial=initial)

    def test_each_axis_quantized_physical_displacement_and_pd_rejections(self):
        for mid in range(1, 13):
            for kind in ('physical', 'displacement', 'pd'):
                axes = copy.deepcopy(self.axes)
                if kind == 'physical':
                    axes[mid]['lower_rad'] = self.initial[mid - 1] + .001
                elif kind == 'displacement':
                    axes[mid]['max_displacement_from_start_rad'] = .000001
                else:
                    axes[mid]['max_estimated_pd_torque_nm'] = .000001
                with self.subTest(mid=mid, kind=kind):
                    self.assert_same_result(self.command, axes=axes)

    def test_single_nonfinite_and_software_cap_rejections(self):
        cases = (
            ('q_model_rad', 1, math.nan),
            ('q_model_rad', 4, math.inf),
            ('q_model_rad', 7, 13.),
            ('kp', 2, math.nan),
            ('kp', 5, 36.0001),
            ('kd', 3, math.inf),
            ('kd', 8, 1.0001),
            ('estimated_pd_torque_nm', 9, math.nan),
        )
        for name, mid, value in cases:
            with self.subTest(name=name, mid=mid, value=value):
                self.assert_same_result(with_change(self.command, name, mid, value))

    def test_invalid_context_is_rejected_before_any_encoding(self):
        specs = [(0., 1., -1., 1., 0., .1, .1)] * 12
        for value in (0., 2., math.nan):
            changed = list(specs)
            changed[7] = (0., value, -1., 1., 0., .1, .1)
            with self.subTest(sign=value), self.assertRaises(ValueError):
                self.module.make_context(tuple(changed))


if __name__ == '__main__':
    unittest.main()
