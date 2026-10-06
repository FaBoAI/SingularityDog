"""Data-only guard tests; fixtures are not policy/performance evidence."""
import copy
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock
from . import inputs


def fixture():
    cal = {'historical_fixture': True}
    refs = {role: {'path': '/private/tmp/' + role, 'sha256': inputs.PINS[role]} for role in inputs.PINS}
    report = {'status': 'ABORTED', 'cycles_completed': 33,
        'motor_enable_sent': False, 'learned_targets_sent': False,
        'approved_for_runtime': False, 'full_controller_50Hz_verified': False,
        'input_sha256': {name: inputs.PINS[role] for name, role in
          (('calibration', 'calibration'), ('mount', 'mount'), ('gyro_bias', 'gyro_bias'),
           ('accel_input_hypothesis', 'accel_hypothesis'))}}
    rows = []
    digest = inputs.sha(inputs.canonical(cal))
    for index in range(33):
        observed = {'status': 'TICK_OBSERVED_NO_OUTPUT', 'output_allowed': False,
            'tick_index': index, 'tick_ns': 100 + index * 21,
            'inputs': {key: [0.] * width for key, width in inputs.WIDTHS.items()},
            'provenance': {'calibration_canonical_json_sha256': digest}}
        observed.update({key: [0.] * width for key, width in inputs.OUTPUTS.items()})
        rows.append({'cycle': index + 1, 'observed': observed})
    rows.append({'cycle': 34, 'output': {'failure_fixture': True}})
    return report, rows, cal, refs, digest


class InputTests(unittest.TestCase):
    def test_duplicate_nonfinite_json(self):
        for raw in (b'{"x":0,"x":1}', b'[NaN]', b'[1e999]'):
            with self.assertRaises(ValueError): inputs.strict_json(raw)

    def test_read_regular_pin_and_closed_on_failure(self):
        with tempfile.TemporaryDirectory(dir='/private/tmp') as directory:
            p = Path(directory) / 'source'; p.write_bytes(b'exact')
            with mock.patch.object(inputs.os, 'close', wraps=os.close) as closed:
                self.assertEqual(inputs.read({'path': str(p), 'sha256': inputs.sha(b'exact')}), b'exact')
                with self.assertRaisesRegex(ValueError, 'Pinned file changed'):
                    inputs.read({'path': str(p), 'sha256': '0' * 64})
                self.assertEqual(closed.call_count, 2)

    def test_fifo_directory_symlink_rejected(self):
        with tempfile.TemporaryDirectory(dir='/private/tmp') as directory:
            root = Path(directory); fifo = root / 'fifo'; os.mkfifo(fifo)
            target = root / 'target'; target.write_bytes(b'x')
            link = root / 'link'; link.symlink_to(target)
            for p in (root, fifo, link):
                with self.assertRaises(ValueError):
                    inputs.read({'path': str(p), 'sha256': inputs.sha(b'x')})

    def test_conflicting_closure_pins_rejected(self):
        c = inputs.Closure()
        with tempfile.TemporaryDirectory(dir='/private/tmp') as directory:
            p = Path(directory) / 'source'; p.write_bytes(b'x')
            c.read({'path': str(p), 'sha256': inputs.sha(b'x')})
            with self.assertRaisesRegex(ValueError, 'Conflicting'):
                c.read({'path': str(p), 'sha256': '0' * 64})
            p.write_bytes(b'y')
            with self.assertRaisesRegex(ValueError, 'Pinned file changed'): c.verify()

    def test_exact33_preserves_partial_and_tick_axis(self):
        report, rows, cal, refs, digest = fixture()
        before = copy.deepcopy(rows)
        with mock.patch.object(inputs, 'CAL_CANONICAL', digest):
            selected, partial = inputs.archived_rows(report, rows, cal, refs)
        self.assertEqual(len(selected), 33)
        self.assertIs(partial, rows[33]); self.assertEqual(rows, before)
        self.assertEqual(selected[-1]['observed']['tick_ns'], 100 + 32 * 21)

    def test_partial_order_freshness_and_cal_selection_guards(self):
        mutations = (
            lambda r, rows, c, refs: rows[33].update(observed=rows[0]['observed']),
            lambda r, rows, c, refs: rows[1].update(cycle=1),
            lambda r, rows, c, refs: rows[1]['observed'].update(tick_ns=100),
            lambda r, rows, c, refs: r['input_sha256'].update(calibration='0' * 64),
            lambda r, rows, c, refs: r.update(cycles_completed=34),
            lambda r, rows, c, refs: rows[0]['observed']['inputs']['command'].__setitem__(0, 1.),
            lambda r, rows, c, refs: rows[0]['observed']['inputs']['h_hypothesis12'].__setitem__(0, 1.),
            lambda r, rows, c, refs: rows[0]['observed'].update(output_allowed=True),
        )
        for change in mutations:
            report, rows, cal, refs, digest = fixture(); change(report, rows, cal, refs)
            with mock.patch.object(inputs, 'CAL_CANONICAL', digest):
                with self.assertRaises(ValueError): inputs.archived_rows(report, rows, cal, refs)

    def test_float32_overflow_bool_width_and_signed_zero(self):
        for values in ([1e40], [True], [], [float('inf')]):
            with self.assertRaises(ValueError): inputs.float32_bits(values, 1)
        self.assertNotEqual(inputs.float32_bits([0.], 1), inputs.float32_bits([-0.], 1))


if __name__ == '__main__': unittest.main()
