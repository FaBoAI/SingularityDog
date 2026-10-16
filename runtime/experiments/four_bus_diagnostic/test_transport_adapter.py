"""Receipt and immutable opt-in checks for separate STOP proxy deadlines."""
import ctypes as C
import json
import math
from pathlib import Path
import shutil
import time
import unittest

from singularitydog_hw.can_readonly import read_request
from . import transport_adapter as T
from . import test_stop_write_deadline as native_fixtures
pins_for = native_fixtures.pins_for


class AdapterWriteDeadlineSelectionTests(unittest.TestCase):
    # Fixture methods only; do not duplicate the native suite's test methods.
    setUpClass = classmethod(native_fixtures.StopWriteDeadlineTests.setUpClass.__func__)
    tearDownClass = classmethod(native_fixtures.StopWriteDeadlineTests.tearDownClass.__func__)
    setUp = native_fixtures.StopWriteDeadlineTests.setUp
    tearDown = native_fixtures.StopWriteDeadlineTests.tearDown
    adapter = native_fixtures.StopWriteDeadlineTests.adapter
    peer_loop = native_fixtures.StopWriteDeadlineTests.peer_loop
    silent = native_fixtures.StopWriteDeadlineTests.silent

    def copied_library(self, change):
        directory = Path(self.directory.name) / f'receipt-{time.monotonic_ns()}'
        shutil.copytree(self.library_path.parent, directory)
        path = directory / 'build-record.json'
        receipt = json.loads(path.read_bytes())
        change(receipt['four_bus_subset_stop'])
        path.write_text(json.dumps(receipt))
        return directory / self.library_path.name

    def test_new_optional_abi_functions_are_authenticated_and_sealed(self):
        self.assertEqual(self.lib.sda_subset_stop_write_deadline_abi(), 1)
        scope = json.loads((self.library_path.parent / 'build-record.json').read_bytes())['four_bus_subset_stop']
        self.assertIs(type(scope['write_deadline_abi']), int)
        self.assertEqual(scope['write_deadline_abi'], 1)
        self.assertEqual(len(T.verify_library(self.lib).write_deadline), 2)
        self.assertEqual(self.lib.sda_subset_stop_exchange_write_deadline.argtypes, list(T._WRITE_ARGS[1]))
        original = self.lib.sda_subset_stop_exchange_write_deadline
        self.lib.sda_subset_stop_exchange_write_deadline = self.lib['sda_subset_stop_exchange_write_deadline']
        try:
            with self.assertRaisesRegex(ValueError, 'binding changed'):
                T.verify_library(self.lib)
        finally:
            self.lib.sda_subset_stop_exchange_write_deadline = original

    def test_old_receipt_can_use_only_legacy_default_and_never_open_selected_session(self):
        path = self.copied_library(lambda scope: scope.pop('write_deadline_abi'))
        lib = T.load_library(path, **pins_for(path))
        self.assertEqual(T.verify_library(lib).write_deadline, ())
        with self.assertRaisesRegex(ValueError, 'receipt and ABI'):
            self.adapter(lib=lib)
        self.assertEqual(self.adapters, [])
        self.silent()
        adapter = self.adapter(lib=lib, selected=False)
        self.peer_loop(3)
        self.assertEqual(len(adapter.output_stop(deadline_ns=time.monotonic_ns()+20_000_000,
            check=lambda: None).records), 3)
        self.thread.join(timeout=1)

    def test_bool_or_unknown_receipt_capability_is_rejected(self):
        for value in (True, False, 0, 2, '1'):
            path = self.copied_library(lambda scope: scope.update(write_deadline_abi=value))
            with self.assertRaisesRegex(ValueError, 'extension pins'):
                T.load_library(path, **pins_for(path))
            self.silent()

    def test_selection_requires_an_exact_bool_before_native_session_creation(self):
        for value in (1, 0, None, 'true'):
            with self.assertRaisesRegex(ValueError, 'exact bool'):
                self.adapter(selected=value)
            self.assertEqual(self.adapters, [])
            self.silent()

    def test_selected_capability_cannot_be_changed_after_creation(self):
        adapter = self.adapter()
        adapter.separate_write_deadline = False
        now = time.monotonic_ns()
        with self.assertRaisesRegex(ValueError, 'binding changed'):
            adapter.output_stop(deadline_ns=now+33_333_334, write_deadline_ns=now+20_000_000,
                check=lambda: None)
        self.silent()

    def test_selected_and_supplied_cutoff_must_match_and_be_valid(self):
        for selected, value in ((True, None), (False, time.monotonic_ns()+20_000_000)):
            adapter = self.adapter(selected=selected)
            with self.assertRaisesRegex(ValueError, 'supplied exactly when selected'):
                adapter.output_stop(deadline_ns=time.monotonic_ns()+33_333_334,
                    write_deadline_ns=value, check=lambda: None)
            self.assertTrue(adapter._session.poisoned)
            self.silent(); adapter.close()
        for value in (True, False, math.nan, math.inf, '20ms', 0, -1):
            adapter = self.adapter()
            with self.assertRaisesRegex(ValueError, 'Separate STOP deadline'):
                adapter.output_stop(deadline_ns=time.monotonic_ns()+33_333_334,
                    write_deadline_ns=value, check=lambda: None)
            self.assertTrue(adapter._session.poisoned)
            self.assertEqual(adapter.write_deadline_journal, [])
            self.silent(); adapter.close()

    def test_other_labels_or_prepared_hook_cannot_use_new_abi(self):
        for case in ('voltage', 'hook'):
            adapter = self.adapter()
            now = time.monotonic_ns()
            wires = (read_request(1, 'voltage'),) if case == 'voltage' else adapter._stop_wires
            with self.assertRaisesRegex(ValueError, 'only for the exact output_stop triple'):
                adapter._exchange(wires, now+33_333_334, 'voltage' if case == 'voltage' else 'output_stop',
                    write_deadline_ns=now+20_000_000, before_native=(lambda: None) if case == 'hook' else None)
            self.silent(); adapter.close()

    def test_changed_published_write_proof_fails_original_owner_takeout(self):
        adapter = self.adapter()
        self.peer_loop(3)
        now = time.monotonic_ns()
        batch = adapter.output_stop(deadline_ns=now+33_333_334,
            write_deadline_ns=now+20_000_000, check=lambda: None)
        self.thread.join(timeout=1)
        object.__setattr__(batch, 'write_deadline_ns', now+30_000_000)
        with self.assertRaisesRegex(ValueError, 'Genuine current owner batch'):
            adapter.verify_batch(batch, 'output_stop')


if __name__ == '__main__':
    unittest.main()
