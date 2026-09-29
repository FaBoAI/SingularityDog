import importlib.util
import json
from pathlib import Path
import sys
import unittest


SOURCE = Path(__file__).resolve().parents[2] / 'tools' / 'python_thread_switch_scope.py'
spec = importlib.util.spec_from_file_location('python_thread_switch_scope', SOURCE)
scope = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scope)


class SwitchScopeTests(unittest.TestCase):
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
