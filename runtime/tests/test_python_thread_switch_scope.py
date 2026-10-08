import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).resolve().parents[2] / 'tools' / 'python_thread_switch_scope.py'
spec = importlib.util.spec_from_file_location('python_thread_switch_scope', SOURCE)
scope = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scope)


class SwitchScopeTests(unittest.TestCase):
    def test_nested_100us_scope_restores_exact_getter_after_success_and_error(self):
        original = sys.getswitchinterval()
        try:
            sys.setswitchinterval(.0001)
            nested = sys.getswitchinterval()
            for failure in (None, KeyboardInterrupt()):
                with self.subTest(failure=type(failure).__name__):
                    events = []
                    def run(*args, **kwargs):
                        if failure is not None:
                            raise failure
                    if failure is None:
                        scope.run(scope.MODULES[0], [], interval_us=100,
                                  runner=run, emit=events.append)
                    else:
                        with self.assertRaises(KeyboardInterrupt):
                            scope.run(scope.MODULES[0], [], interval_us=100,
                                      runner=run, emit=events.append)
                    self.assertEqual(sys.getswitchinterval(), nested)
                    self.assertTrue(json.loads(events[-1])['restored'])
        finally:
            sys.setswitchinterval(math.nextafter(original, math.inf))
            self.assertEqual(sys.getswitchinterval(), original)

    def test_restore_failure_keeps_primary_error_and_original_argv(self):
        original, argv = sys.getswitchinterval(), sys.argv
        real_setter = sys.setswitchinterval
        calls = []
        primary = RuntimeError('original diagnostic failure')
        events = []
        def setter(value):
            calls.append(value)
            if len(calls) == 2:
                raise OSError('injected restoration failure')
            real_setter(value)
        def run(*args, **kwargs):
            raise primary
        try:
            with patch.object(scope.sys, 'setswitchinterval', side_effect=setter):
                with self.assertRaises(RuntimeError) as caught:
                    scope.run(scope.MODULES[0], [], interval_us=100,
                              runner=run, emit=events.append)
            self.assertIs(caught.exception, primary)
            self.assertIs(sys.argv, argv)
            self.assertFalse(json.loads(events[-1])['restored'])
            self.assertIn('restoration failed', primary.__notes__[0])
        finally:
            real_setter(math.nextafter(original, math.inf))
            self.assertEqual(sys.getswitchinterval(), original)

    def test_arguments_and_interval_restore_after_success(self):
        before, argv, events = sys.getswitchinterval(), sys.argv, []

        def run(module, **kwargs):
            self.assertEqual(module, scope.MODULES[0])
            self.assertEqual(kwargs, {'run_name': '__main__'})
            self.assertEqual(sys.argv, [module, '--cycles', '501'])
            self.assertAlmostEqual(sys.getswitchinterval(), 0.0001)
            return 'done'

        self.assertEqual(scope.run(scope.MODULES[0], ['--cycles', '501'],
                                   interval_us=100, runner=run, emit=events.append), 'done')
        self.assertIs(sys.argv, argv)
        self.assertEqual(sys.getswitchinterval(), before)
        self.assertTrue(json.loads(events[-1])['restored'])

    def test_system_exit_and_interrupt_restore_and_propagate(self):
        for exception in (SystemExit(2), KeyboardInterrupt()):
            with self.subTest(exception=type(exception).__name__):
                before, argv = sys.getswitchinterval(), sys.argv

                def run(*args, **kwargs):
                    raise exception

                with self.assertRaises(type(exception)):
                    scope.run(scope.MODULES[1], [], interval_us=100,
                              runner=run, emit=lambda _: None)
                self.assertEqual(sys.getswitchinterval(), before)
                self.assertIs(sys.argv, argv)

    def test_invalid_interval_or_module_does_not_enter_runner(self):
        for module, interval in ((scope.MODULES[0], True), (scope.MODULES[0], 0),
                                 (scope.MODULES[0], 100.0), ('os', 100)):
            with self.subTest(module=module, interval=interval):
                entered = []
                with self.assertRaises(ValueError):
                    scope.run(module, [], interval_us=interval,
                              runner=lambda *a, **k: entered.append(True))
                self.assertEqual(entered, [])


if __name__ == '__main__':
    unittest.main()
