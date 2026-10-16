"""File-only foreground authentication of the genuine composite native DSO.

Compilation and query-only ABI inspection happen before the PLAN tests. No
Session, peer, serial FD, model execution or motor command is created here.
"""
import contextlib
import copy
import ctypes as C
import hashlib
import io
import json
from pathlib import Path
import shutil
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiments.four_bus_diagnostic import foreground as FG
from experiments.four_bus_native_group import build as group_build
from experiments.four_bus_native_group import client
from . import build, type1_foreground as F, type1_profile as P
from . import test_type1_foreground as fixtures
from . import test_type1_profile as profile_fixtures
from singularitydog_hw import policy_model_30hz as models
from singularitydog_hw.policy_clock import control_clock_payload


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class NativeGroupForegroundTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temporary.name).resolve()
        cls.group_library = group_build.build(cls.root/'group')
        cls.classic_library = build.build(cls.root/'classic')
        cls.kit = cls.root/'kit'
        cls.members = {
            'runtime/experiments/native_active_transport/transport.cpp': group_build.ORDINARY,
            'runtime/experiments/four_bus_diagnostic/subset_stop.cpp': group_build.SUBSET_STOP,
            'runtime/experiments/four_bus_type1/subset_active.cpp': group_build.SUBSET_ACTIVE,
            'runtime/experiments/four_bus_native_group/native_group.cpp': group_build.EXTENSION}
        rows = {}
        for member, source in cls.members.items():
            path = cls.kit/member; path.parent.mkdir(parents=True, exist_ok=True)
            raw = source.read_bytes(); path.write_bytes(raw); path.chmod(0o644)
            rows[member] = {'sha256': sha(path), 'bytes': len(raw), 'mode': 0o644}
        cls.manifest = {'file_count': 4, 'files': rows, 'output_allowed': False,
                        'approved_for_runtime': False}

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def args(self, library=None):
        library = self.group_library if library is None else library
        return SimpleNamespace(type1_library=str(library), type1_library_sha256=sha(library),
            type1_build_sha256=sha(library.parent/'build-record.json'),
            ordinary_source_sha256=sha(library.parent/'ordinary_transport.cpp'),
            subset_stop_source_sha256=sha(library.parent/'subset_stop.cpp'),
            type1_extension_source_sha256=sha(library.parent/'transport.cpp'),
            native_group_base_source_sha256=sha(self.kit/'runtime/experiments/four_bus_type1/subset_active.cpp'))

    def file_plan(self, args, *, native=True, manifest=None):
        with patch.object(C, 'CDLL', side_effect=AssertionError('PLAN dlopen')), \
             patch.object(threading.Thread, 'start', side_effect=AssertionError('PLAN thread')), \
             patch.object(client, 'NativeGroup', side_effect=AssertionError('PLAN native owner')):
            return F.prepare_type1_library(args, self.manifest if manifest is None else manifest,
                FG.Pins(), separate_write_deadline=True, native_group_cycle=native)

    def test_exact_actual_composite_four_source_manifest_file_plan(self):
        pins = FG.Pins(); FG.verify_manifest(self.kit, self.manifest, 4, pins)
        result = self.file_plan(self.args())
        self.assertFalse(result['loads_library_in_plan']); self.assertFalse(result['output_allowed'])
        self.assertEqual(result['library']['sha256'], sha(self.group_library))
        native = result['native_group']
        self.assertEqual(native['included_source_sha256'], {
            key: sha(source) for key, source in zip(group_build.SOURCE_KEYS,
                (group_build.ORDINARY, group_build.SUBSET_STOP, group_build.SUBSET_ACTIVE,
                 group_build.EXTENSION))})
        self.assertEqual((native['gap_ns'], native['window'], native['abi']), (900_000, 3, 1))
        self.assertEqual(native['exact_motor_ids'], list(range(1, 13)))
        self.assertFalse(native['timing_admission_eligible']); self.assertFalse(native['output_allowed'])
        self.assertEqual(result['sources']['subset_active.cpp'], sha(group_build.SUBSET_ACTIVE))

    def test_native_group_requires_complete_manifest_and_explicit_fourth_pin(self):
        missing = copy.deepcopy(self.manifest)
        del missing['files']['runtime/experiments/four_bus_native_group/native_group.cpp']
        with self.assertRaisesRegex(ValueError, 'inventory/count'):
            FG.verify_manifest(self.kit, missing, 4, FG.Pins())
        with self.assertRaisesRegex(ValueError, 'selected current source bytes'):
            self.file_plan(self.args(), manifest=missing)
        args = self.args(); args.native_group_base_source_sha256 = '0'*64
        with self.assertRaisesRegex(ValueError, 'SHA|differs|changed'):
            self.file_plan(args)

    def test_classic_library_cannot_fall_back_when_native_group_selected(self):
        with self.assertRaisesRegex(ValueError, 'selected current source bytes|Native group'):
            self.file_plan(self.args(self.classic_library), native=True)

    def test_composite_library_rejected_when_native_group_not_selected(self):
        with self.assertRaisesRegex(ValueError, 'selected current source bytes|explicitly selected'):
            self.file_plan(self.args(), native=False)
        result = self.file_plan(self.args(self.classic_library), native=False)
        self.assertNotIn('native_group', result)
        self.assertFalse(result['loads_library_in_plan'])

    def test_modified_included_subset_active_copy_detected_without_loading(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp).resolve()/'changed'; shutil.copytree(self.group_library.parent, directory)
            (directory/'subset_active.cpp').write_bytes(group_build.SUBSET_ACTIVE.read_bytes()+b'\n// changed\n')
            args = self.args(directory/group_build.LIBRARY_NAME)
            with self.assertRaisesRegex(ValueError, 'included source'):
                self.file_plan(args)

    def test_wrong_group_source_manifest_pin_cannot_use_valid_native_binary(self):
        changed = copy.deepcopy(self.manifest)
        changed['files']['runtime/experiments/four_bus_native_group/native_group.cpp']['sha256'] = 'f'*64
        with self.assertRaisesRegex(ValueError, 'selected current source bytes'):
            self.file_plan(self.args(), manifest=changed)
        args = self.args(); args.type1_extension_source_sha256 = sha(group_build.SUBSET_ACTIVE)
        with self.assertRaisesRegex(ValueError, 'SHA|differs|changed|source'):
            self.file_plan(args)

    def test_default_complete_foreground_plan_never_instantiates_native_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve(); world = fixtures.World(root)
            current = world.capture('current', 50_000_000_000)
            zero = world.admission('zero', 'zero_gain_timing', 2, current)
            output = root/'unused-run'
            argv = world.argv(*zero, current, 'zero_gain_timing', 2, output)
            with fixtures.file_only_prepare(), \
                 patch.object(C, 'CDLL', side_effect=AssertionError('PLAN dlopen')), \
                 patch.object(threading.Thread, 'start', side_effect=AssertionError('PLAN thread')), \
                 patch.object(client, 'NativeGroup', side_effect=AssertionError('PLAN native group')), \
                 patch.object(F, 'LinuxEnvironment', side_effect=AssertionError('PLAN hardware environment')), \
                 patch.object(F, 'execute', side_effect=AssertionError('PLAN execute')), \
                 contextlib.redirect_stdout(io.StringIO()) as printed:
                self.assertEqual(F.main(argv), 0)
            plan = json.loads(printed.getvalue())
            self.assertEqual(plan['status'], 'PLAN'); self.assertFalse(output.exists())
            self.assertFalse(plan['opens_devices']); self.assertFalse(plan['loads_library_model_or_torch'])
            self.assertNotIn('native_group', plan['type1_library_plan'])
            self.assertNotIn('native_group_cycle', plan['pacing'])
            for key in (*F.ATTEMPTS, 'output_approval_granted_here', 'live_type1_qualified', 'approved_for_runtime'):
                self.assertIs(plan[key], False)

    def test_native_cycle_is_explicit_30hz_only_and_preserves_all_physical_limits(self):
        with tempfile.TemporaryDirectory() as tmp:
            world = profile_fixtures.World(Path(tmp).resolve())
            arguments = (copy.deepcopy(world.plan), world.lineage_refs(),
                         copy.deepcopy(world.geometry_doc), world.geometry)
            with self.assertRaisesRegex(ValueError, 'explicit 30 Hz'):
                P.build_contract(*arguments, options={'native_group_cycle': True})
            old = P.build_contract(*arguments)
            # Contract selection validates an inline fixture only. Foreground
            # execution must separately reprove real artifact files; this test
            # neither reads a model release nor grants model/runtime approval.
            inline_model = models._plan(models.bundle_artifacts(Path(tmp).resolve()/'uninstalled-inline-fixture'),
                {'source_manifest': dict.fromkeys(models._SOURCE_NAMES),
                 'export': {'evaluation_all_intervals_completed': False}})
            selected = P.build_contract(*arguments, options={
                'native_group_cycle': True, 'decode_once': True, 'gc_freeze': True},
                control_clock=control_clock_payload(30), policy30_model=inline_model)
            P.validate_contract(selected)
            for name in ('axes', 'start_pose_bounds', 'watchdog', 'enable', 'terminal_stop',
                         'imu_limits', 'ramps', 'lineage', 'source_manifest'):
                self.assertEqual(selected[name], old[name], name)
            self.assertEqual(P.pacing_options(selected['pacing']), {
                'decode_once': True, 'gc_freeze': True, 'native_group_cycle': True})
            self.assertNotIn('native_group_cycle', old['pacing'])
            for value in (False, 1, 'true'):
                with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'exactly true'):
                    P.pacing_options({**P.PACING, 'native_group_cycle': value})


if __name__ == '__main__':
    unittest.main()
