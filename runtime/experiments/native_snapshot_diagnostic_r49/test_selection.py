"""Native copied-observer equality/restore tests; fake policy, no devices/weights."""
import contextlib
import copy
import hashlib
import importlib.util
import itertools
import io
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

from . import snapshot_generate as generate
from . import snapshot_loader as pure_loader
from . import snapshot_build as build
from singularitydog_hw import policy_observer as original
from native_policy_overnight.target_tail_fk_cache import diagnostic_generate as fk_generate

ROOT = Path(original.__file__).absolute().parents[2]
HERE = Path(__file__).absolute().parent


def bits(value):
    if type(value) is float: return ('float', struct.pack('>d', value).hex())
    if type(value) in (tuple, list): return (type(value).__name__, tuple(bits(v) for v in value))
    if type(value) is dict: return ('dict', tuple((bits(k), bits(v)) for k, v in value.items()))
    return (type(value).__name__, value)


def need(value, message):
    if not value: raise AssertionError(message)


def fixtures():
    path = ROOT / 'runtime/tests/test_policy_observer.py'
    need(hashlib.sha256(path.read_bytes()).hexdigest() ==
         'aa672fb9b0a534781c54d39c574bd2de323e952dd2e332bd1bc40347e89cbc9c', 'Fixture source changed')
    module = ModuleType('_snapshot_r49_fixtures'); module.__file__ = str(path)
    exec(compile(path.read_bytes(), str(path), 'exec'), module.__dict__)
    return module


def make_source_bundle(folder):
    r47 = folder / 'r47'
    fk_generate.generate_bundle(ROOT / 'runtime/singularitydog_hw/native_pipeline_benchmark.py', r47,
        target_bundle=Path('/home/jetson/singularitydog-logs/validation-20261006-r1/fk-stop-source-bundle-r47'),
        baseline_kit=Path('/home/jetson/singularitydog-kits/corrected-active-20261006-r37'))
    need(hashlib.sha256((r47 / 'manifest.json').read_bytes()).hexdigest() == generate.FK_MANIFEST_SHA,
         'Fixture must reproduce the actual immutable deployment manifest bytes')
    out = folder / 'bundle'
    generate.generate_bundle(r47, Path(original.__file__).absolute(), HERE, out,
                             target_bundle=out, baseline_kit=ROOT)
    return out


@contextlib.contextmanager
def bundle_modules(folder):
    names = {k: v for k, v in sys.modules.items() if k == 'native_snapshot_diagnostic_r49' or
             k.startswith('native_snapshot_diagnostic_r49.')}
    for name in names: del sys.modules[name]
    path = folder / 'native_snapshot_diagnostic_r49/__init__.py'
    spec = importlib.util.spec_from_file_location('native_snapshot_diagnostic_r49', path,
                                                 submodule_search_locations=[str(path.parent)])
    package = importlib.util.module_from_spec(spec);sys.modules[package.__name__] = package
    try:
        spec.loader.exec_module(package)
        from native_snapshot_diagnostic_r49 import snapshot_loader, snapshot_support
        yield snapshot_loader, snapshot_support
    finally:
        for name in tuple(sys.modules):
            if name == 'native_snapshot_diagnostic_r49' or name.startswith('native_snapshot_diagnostic_r49.'):
                del sys.modules[name]
        sys.modules.update(names)


def observer_parity(original, candidate, fixtures):
    def make(module, h=0, bad=None):
        policy = fixtures.Policy(bad)
        observer = module.StatefulPolicyObserver(policy, fixtures.calibration(),
            imu_mount_candidate=fixtures.mount(), h_hypothesis=h, command=[0., 0., 0.],
            max_ticks=4, max_age_ns=10_000_000, max_spread_ns=5_000_000,
            torch_module=fixtures.FakeTorch)
        observer.reset_run(1_000_000_000, warmup_completed=True)
        return observer
    pairs = []
    for h in (0, 1):
        observers = [make(module, h) for module in (original, candidate)]
        for index in range(4):
            source = fixtures.snapshot(1_000_000_000 + index * original.DT_NS)
            source['source_flags'] = {'dynamic': [index, -0.0, '方向']}
            source['imu']['gyro_rad_s'][0] += index * .001
            before = bits(source)
            records = [observer.consume(source) for observer in observers]
            need(bits(records[0]) == bits(records[1]), 'Complete observer records differ')
            need(bits(source) == before, 'Caller snapshot mutated')
            for name in ('_next_tick_ns', '_last_sources', '_last_branch_raw', 'ticks_completed', 'status'):
                need(bits(getattr(observers[0], name)) == bits(getattr(observers[1], name)), 'Observer state differs: ' + name)
            need(bits(observers[0]._policy.calls) == bits(observers[1]._policy.calls), 'Policy input bits differ')
            need(bits(observers[0]._policy.previous) == bits(observers[1]._policy.previous), 'Recurrent fake-policy state differs')
        records[1]['provenance']['snapshot_source_flags']['dynamic'].clear()
        need(bits(records[0]) != bits(records[1]), 'Returned record ownership differs')
        pairs.append(h)
    def mutate_motor(key, value):
        return lambda s: s['motors'][0].update({key: value})
    cases = [
        ('blocked', lambda s: s.update(blocked_reasons=['STOP failed']), None),
        ('allowed', lambda s: s.update(output_allowed=True), None),
        ('tick', lambda s: s.update(tick_ns=s['tick_ns'] + 1), None),
        ('max_age', lambda s: s.update(max_age_ns=True), None),
        ('missing_motor', lambda s: s['motors'].pop(), None),
        ('duplicate_motor', lambda s: s['motors'].__setitem__(1, copy.deepcopy(s['motors'][0])), None),
        ('motor_bool_id', mutate_motor('motor_id', True), None),
        ('motor_parameter', mutate_motor('parameter', 'current'), None),
        ('motor_unit', mutate_motor('unit', 'degrees'), None),
        ('motor_nan', mutate_motor('value', math.nan), None),
        ('motor_causality', mutate_motor('received_ns', 1_000_000_001), None),
        ('motor_age', mutate_motor('age_upper_bound_ns', 1), None),
        ('joint_range', mutate_motor('value', 999.), None),
        ('imu_frame', lambda s: s['imu'].update(frame='body'), None),
        ('imu_corrected', lambda s: s['imu'].update(accel_bias_subtracted=True), None),
        ('imu_nan', lambda s: s['imu']['gyro_rad_s'].__setitem__(0, math.nan), None),
        ('imu_norm', lambda s: s['imu'].update(accel_m_s2=[0., 0., 0.]), None),
        ('imu_age', lambda s: s['imu'].update(age_upper_bound_ns=1), None),
        ('summary_age', lambda s: s.update(oldest_observation_age_ns=999), None),
        ('summary_spread', lambda s: s.update(acquisition_spread_ns=999), None),
        ('source_flags', lambda s: s.update(source_flags=[]), None),
        ('not_json', lambda s: s.update(metadata=object()), None),
        ('target_shape', lambda s: None, 'target_shape'),
        ('target_range', lambda s: None, 'target_bounds'),
        ('actor_nan', lambda s: None, 'actor_nan'),
        ('observation_shape', lambda s: None, 'observation_batch'),
    ]
    for label, change, bad in cases:
        outcomes = []
        for module in (original, candidate):
            observer = make(module, bad=bad)
            source = fixtures.snapshot(); change(source)
            try:
                observer.consume(source)
            except ValueError as error:
                outcomes.append((type(error).__name__, str(error), observer.status,
                    observer.failure, observer.ticks_completed, observer._next_tick_ns,
                    bits(observer._last_sources), bits(observer._policy.calls), bits(observer._policy.previous)))
            else:
                raise ValueError('Invalid observer input accepted: ' + label)
        need(outcomes[0] == outcomes[1], 'Observer rejection differs: ' + label)
    for mode in ('held_changed', 'partly_repeated'):
        outcomes = []
        for module in (original, candidate):
            observer = make(module)
            source = fixtures.snapshot(); observer.consume(source)
            source['tick_ns'] += original.DT_NS
            source['oldest_observation_age_ns'] += original.DT_NS
            for row in source['motors']:
                row['age_upper_bound_ns'] += original.DT_NS
            source['imu']['age_upper_bound_ns'] += original.DT_NS
            if mode == 'held_changed':
                source['motors'][0]['value'] += .001
            else:
                source['motors'][0]['received_ns'] += 1
            try: observer.consume(source)
            except ValueError as error:
                outcomes.append((type(error).__name__, str(error), observer.status, observer.failure,
                                 observer.ticks_completed, bits(observer._policy.calls)))
            else: raise ValueError('Source replay accepted')
        need(outcomes[0] == outcomes[1], 'Source freshness rejection differs')
    for first_module, second_module in ((original, candidate), (candidate, original)):
        first = make(first_module)
        try:
            second_module.StatefulPolicyObserver(first._policy, fixtures.calibration(),
                imu_mount_candidate=fixtures.mount(), h_hypothesis=0, command=[0., 0., 0.],
                max_ticks=4, max_age_ns=10_000_000, max_spread_ns=5_000_000,
                torch_module=fixtures.FakeTorch)
        except ValueError as error:
            need(str(error) == 'Policy instance already belongs to an observer', 'Mixed observer ownership error differs')
        else:
            raise ValueError('Mixed observer policy sharing accepted')
    return {'successful_ticks': 8, 'h_hypotheses': pairs, 'rejection_cases': len(cases) + 2,
            'complete_records_inputs_fake_recurrent_state_and_failure_equal': True,
            'mixed_observer_ownership_rejections': 2,
            'ordinary_module_globals_unchanged': original.snapshot_event is not candidate.snapshot_event}


class SelectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.folder = Path(cls.temporary.name).resolve()
        cls.bundle = make_source_bundle(cls.folder)
        cls.bundle_manifest = cls.bundle / 'manifest.json'
        cls.bundle_sha = pure_loader.sha(cls.bundle_manifest.read_bytes())
        cls.build_output = cls.folder / 'native-build'
        build.build(cls.bundle_manifest, expected_sha256=cls.bundle_sha,
                    original_observer=Path(original.__file__).absolute(), output=cls.build_output)
        cls.manifest = cls.folder / 'artifact.json'
        data = build.artifact(cls.bundle_manifest, cls.bundle_sha, Path(original.__file__).absolute(),
                              cls.build_output / 'build-receipt.json')
        cls.manifest.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')
        cls.manifest_sha = pure_loader.sha(cls.manifest.read_bytes())
        cls.fixture = fixtures()

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def benchmark(self):
        return SimpleNamespace(__file__=str(self.bundle / 'fk_cache_diagnostic_benchmark.py'), observer=original)

    def test_default_plan_no_extension_compiler_torch_or_devices(self):
        modules = set(sys.modules)
        with mock.patch('subprocess.run', side_effect=AssertionError('Compiler started')), \
                mock.patch.object(pure_loader.importlib.util, 'spec_from_file_location',
                                  side_effect=AssertionError('Native loaded')):
            plan = pure_loader.plan(self.manifest, expected_sha256=self.manifest_sha)
        self.assertFalse(plan['native_library_loaded']); self.assertFalse(plan['observer_selected'])
        self.assertFalse(plan['hardware_opened']); self.assertNotIn('torch', set(sys.modules) - modules)
        self.assertNotIn(pure_loader.PRIVATE_NATIVE, sys.modules)

    def test_rejected_directory_fifo_and_digest_reads_close_descriptors(self):
        opened, closed = [], []
        real_open, real_close = os.open, os.close
        def record_open(*args, **kwargs):
            fd = real_open(*args, **kwargs); opened.append(fd); return fd
        def record_close(fd):
            closed.append(fd); return real_close(fd)
        pipe = self.folder / 'fifo'; os.mkfifo(pipe)
        rows = [self.folder, pipe, self.manifest]
        with mock.patch.object(pure_loader.os, 'open', side_effect=record_open), \
                mock.patch.object(pure_loader.os, 'close', side_effect=record_close):
            for path in rows:
                for _ in range(5):
                    with self.assertRaises(ValueError):
                        pure_loader.read({'path': str(path), 'sha256': '0' * 64})
        self.assertEqual(opened, closed)
        self.assertEqual(len(opened), 15)

    def test_full_observer_success_failure_state_ownership_equality(self):
        benchmark = self.benchmark(); before = dict(original.__dict__)
        with bundle_modules(self.bundle) as (loader, _):
            with loader.select(benchmark, self.manifest, expected_sha256=self.manifest_sha) as candidate:
                self.assertIs(candidate.ObserverError, original.ObserverError)
                self.assertIs(candidate._OWNERS, original._OWNERS)
                self.assertIs(benchmark.observer, candidate)
                proof = observer_parity(original, candidate, self.fixture)
                self.assertEqual(proof['successful_ticks'], 8)
                self.assertEqual(proof['rejection_cases'], 28)
                self.assertEqual(proof['mixed_observer_ownership_rejections'], 2)
            self.assertIs(benchmark.observer, original)
            self.assertIsNone(loader._ACTIVE)
            self.assertFalse(any(n in sys.modules for n in
                (loader.PRIVATE_NATIVE, loader.PRIVATE_PACKAGE, loader.PRIVATE_OBSERVER)))
        self.assertEqual(before.keys(), original.__dict__.keys())
        self.assertTrue(all(original.__dict__[k] is v for k, v in before.items()))

    def test_profile_measured_reused_buffers_records_and_failure_equal(self):
        fixture = self.fixture
        class View:
            def __init__(self, buf): self.buf = buf
            def reshape(self, rows, width):
                assert rows == 1 and width == len(self.buf); return self
            def tolist(self): return [list(self.buf)]
        class Torch(fixture.FakeTorch):
            frombuffer = staticmethod(lambda buf, dtype: View(buf))
        def make(module, bad=None):
            clock = itertools.count(1000, 10)
            run = module.StatefulPolicyObserver(fixture.Policy(bad), fixture.calibration(),
                imu_mount_candidate=fixture.mount(), h_hypothesis=0, command=[0., 0., 0.],
                max_ticks=4, max_age_ns=10_000_000, max_spread_ns=5_000_000, torch_module=Torch,
                profile_consume=True, monotonic_ns=lambda: next(clock),
                measured_diagnostic_ticks=True, reuse_input_buffers=True)
            run.reset_run(1_000_000_000, warmup_completed=True)
            return run
        with bundle_modules(self.bundle) as (loader, _), \
                loader.select(self.benchmark(), self.manifest, expected_sha256=self.manifest_sha) as candidate:
            runs = [make(m) for m in (original, candidate)]
            for tick in range(4):
                source = fixture.snapshot(1_000_000_000 + tick * original.DT_NS)
                before = bits(source)
                records = [r.consume(source) for r in runs]
                self.assertEqual(bits(records[0]), bits(records[1]))
                self.assertEqual(bits(runs[0]._last_consume_profile), bits(runs[1]._last_consume_profile))
                self.assertEqual(bits(runs[0]._policy.calls), bits(runs[1]._policy.calls))
                self.assertEqual(before, bits(source))
            for bad in ('target_shape', 'target_bounds', 'actor_nan', 'observation_batch'):
                outcomes = []
                for module in (original, candidate):
                    run = make(module, bad)
                    with self.assertRaises(original.ObserverError) as captured:
                        run.consume(fixture.snapshot())
                    outcomes.append((type(captured.exception), str(captured.exception), run.status, run.failure,
                        run.ticks_completed, bits(run._policy.calls), bits(run._policy.previous),
                        bits(run._last_consume_profile)))
                self.assertEqual(outcomes[0], outcomes[1])

    def test_selection_restored_on_body_failure_and_nested_rejection(self):
        benchmark = self.benchmark()
        with bundle_modules(self.bundle) as (loader, _):
            with self.assertRaisesRegex(RuntimeError, 'synthetic failure'):
                with loader.select(benchmark, self.manifest, expected_sha256=self.manifest_sha):
                    with self.assertRaisesRegex(ValueError, 'Nested'):
                        with loader.select(benchmark, self.manifest, expected_sha256=self.manifest_sha): pass
                    raise RuntimeError('synthetic failure')
            self.assertIs(benchmark.observer, original); self.assertIsNone(loader._ACTIVE)
            self.assertNotIn(loader.PRIVATE_NATIVE, sys.modules)

    def test_selection_restored_on_native_import_failure(self):
        benchmark = self.benchmark()
        with bundle_modules(self.bundle) as (loader, _), \
                mock.patch.object(loader.importlib.util, 'spec_from_file_location',
                                  side_effect=RuntimeError('native import failed')):
            with self.assertRaisesRegex(RuntimeError, 'native import failed'):
                with loader.select(benchmark, self.manifest, expected_sha256=self.manifest_sha): pass
            self.assertIs(benchmark.observer, original)
            self.assertNotIn(loader.PRIVATE_PACKAGE, sys.modules)

    def test_selected_binding_changes_rejected_and_alias_restored(self):
        for change in ('copier', 'benchmark', 'error', 'registry', 'native_alias', 'delete_benchmark'):
            benchmark = self.benchmark()
            with self.subTest(change=change), bundle_modules(self.bundle) as (loader, _):
                with self.assertRaisesRegex(ValueError, 'Snapshot selection cleanup'):
                    with loader.select(benchmark, self.manifest, expected_sha256=self.manifest_sha) as copied:
                        if change == 'copier': copied.snapshot_event = original.snapshot_event
                        elif change == 'benchmark': benchmark.observer = original
                        elif change == 'error': copied.ObserverError = ValueError
                        elif change == 'registry': copied._OWNERS = {}
                        elif change == 'native_alias': sys.modules[loader.PRIVATE_NATIVE] = ModuleType('foreign')
                        else: del benchmark.observer
                self.assertIs(benchmark.observer, original); self.assertIsNone(loader._ACTIVE)
                self.assertNotIn(loader.PRIVATE_NATIVE, sys.modules)

    def test_primary_body_error_preserved_when_cleanup_guard_also_fails(self):
        benchmark = self.benchmark()
        with bundle_modules(self.bundle) as (loader, _):
            with self.assertRaisesRegex(RuntimeError, 'primary body failure') as captured:
                with loader.select(benchmark, self.manifest, expected_sha256=self.manifest_sha):
                    del benchmark.observer
                    raise RuntimeError('primary body failure')
            self.assertIs(benchmark.observer, original); self.assertIsNone(loader._ACTIVE)
            self.assertNotIn(loader.PRIVATE_NATIVE, sys.modules)
            self.assertTrue(any('Snapshot selection cleanup' in note for note in captured.exception.__notes__))

    def test_live_policy_ownership_remains_shared_after_selection_restore(self):
        with bundle_modules(self.bundle) as (loader, _):
            with loader.select(self.benchmark(), self.manifest, expected_sha256=self.manifest_sha) as copied:
                policy = self.fixture.Policy()
                run = copied.StatefulPolicyObserver(policy, self.fixture.calibration(),
                    imu_mount_candidate=self.fixture.mount(), h_hypothesis=0, command=[0.,0.,0.],
                    max_ticks=4, max_age_ns=10_000_000, max_spread_ns=5_000_000,
                    torch_module=self.fixture.FakeTorch)
            self.assertIs(original._OWNERS[policy](), run)
            with self.assertRaisesRegex(original.ObserverError, 'already belongs'):
                original.StatefulPolicyObserver(policy, self.fixture.calibration(),
                    imu_mount_candidate=self.fixture.mount(), h_hypothesis=0, command=[0.,0.,0.],
                    max_ticks=4, max_age_ns=10_000_000, max_spread_ns=5_000_000,
                    torch_module=self.fixture.FakeTorch)

    def altered_manifest(self, change):
        data = json.loads(self.manifest.read_bytes()); change(data)
        path = self.folder / 'altered.json'; path.write_text(json.dumps(data))
        return path, pure_loader.sha(path.read_bytes())

    def test_unknown_original_and_native_sources_rejected_before_native_load(self):
        for key in ('original_observer', 'native_source', 'native_builder'):
            with self.subTest(key=key):
                path, pin = self.altered_manifest(lambda d: d['references'][key].update(sha256='0' * 64))
                with bundle_modules(self.bundle) as (loader, _), \
                        mock.patch.object(loader.importlib.util, 'spec_from_file_location',
                                          side_effect=AssertionError('Native load before rejection')):
                    with self.assertRaises(ValueError):
                        with loader.select(self.benchmark(), path, expected_sha256=pin): pass

    def test_private_prebinding_and_wrong_benchmark_rejected(self):
        with bundle_modules(self.bundle) as (loader, _):
            sentinel = ModuleType(loader.PRIVATE_PACKAGE)
            with mock.patch.dict(sys.modules, {loader.PRIVATE_PACKAGE: sentinel}):
                with self.assertRaisesRegex(ValueError, 'already bound'):
                    with loader.select(self.benchmark(), self.manifest, expected_sha256=self.manifest_sha): pass
                self.assertIs(sys.modules[loader.PRIVATE_PACKAGE], sentinel)
            benchmark = self.benchmark(); benchmark.__file__ = original.__file__
            with self.assertRaisesRegex(ValueError, 'benchmark selection'):
                with loader.select(benchmark, self.manifest, expected_sha256=self.manifest_sha): pass

    def test_post_selection_source_change_aborts_but_restores_alias(self):
        path = self.bundle / 'copy_observer.py'; raw = path.read_bytes(); benchmark = self.benchmark()
        try:
            with bundle_modules(self.bundle) as (loader, _):
                with self.assertRaisesRegex(ValueError, 'Pinned file changed'):
                    with loader.select(benchmark, self.manifest, expected_sha256=self.manifest_sha):
                        path.write_bytes(raw + b'\n')
                self.assertIs(benchmark.observer, original); self.assertIsNone(loader._ACTIVE)
                self.assertNotIn(loader.PRIVATE_NATIVE, sys.modules)
        finally:
            path.write_bytes(raw)

    def test_support_reports_derived_observer_identity_and_restoration(self):
        namespace = vars(self.benchmark()); report = {'status': 'COMPLETE_DIAGNOSTIC', 'errors': [],
            'source_provenance': {'baseline_dependency_source_sha256': {'policy_observer': pure_loader.OBSERVER_SHA}}}
        args = SimpleNamespace(snapshot_manifest=self.manifest, snapshot_manifest_sha256=self.manifest_sha)
        with bundle_modules(self.bundle) as (_, support):
            parser = SimpleNamespace(error=lambda message: (_ for _ in ()).throw(ValueError(message)))
            proof = support.validate_cli(args, parser, namespace['__file__'], original)
            manager = support.enter(args, namespace, proof)
            self.assertIsNot(namespace['observer'], original)
            support.restore(report, proof, manager); support.finalize_report(report, proof)
            self.assertIs(namespace['observer'], original)
            self.assertTrue(proof['selection_restored']); self.assertTrue(proof['owner_registry_shared'])
            self.assertTrue(proof['error_class_shared']); self.assertTrue(proof['sources_unchanged_after_run'])
            self.assertNotEqual(report['executing_source_sha256']['singularitydog_hw/policy_observer.py'],
                                pure_loader.OBSERVER_SHA)
            self.assertEqual(report['status'], 'COMPLETE_DIAGNOSTIC'); self.assertFalse(report['output_allowed'])

    def test_support_missing_namespace_alias_restores_and_reports_primary_plus_cleanup(self):
        namespace = vars(self.benchmark())
        report = {'status': 'ABORTED', 'errors': ['RuntimeError: primary diagnostic failure']}
        args = SimpleNamespace(snapshot_manifest=self.manifest, snapshot_manifest_sha256=self.manifest_sha)
        with bundle_modules(self.bundle) as (loader, support):
            parser = SimpleNamespace(error=lambda message: (_ for _ in ()).throw(ValueError(message)))
            proof = support.validate_cli(args, parser, namespace['__file__'], original)
            manager = support.enter(args, namespace, proof)
            del namespace['observer']
            support.restore(report, proof, manager)
            self.assertIs(namespace['observer'], original); self.assertIsNone(loader._ACTIVE)
            self.assertTrue(proof['selection_restored']); self.assertTrue(proof['ordinary_module_globals_unchanged'])
            self.assertFalse(proof['selected_bindings_unchanged'])
            self.assertEqual(report['errors'][0], 'RuntimeError: primary diagnostic failure')
            self.assertIn('KeyError', report['errors'][1])
            self.assertFalse(any(n in sys.modules for n in
                (loader.PRIVATE_NATIVE, loader.PRIVATE_PACKAGE, loader.PRIVATE_OBSERVER)))

    def test_derived_full_main_plan_uses_real_snapshot_proof_without_loading(self):
        from native_policy_overnight.target_tail_fk_cache.test_diagnostic_generation import GenerationTests
        with bundle_modules(self.bundle) as (loader, support):
            fake_fk = ModuleType('diagnostic_support')
            fake_fk.validate_cli = mock.Mock(return_value={'selection': 'separate FK test boundary'})
            module = ModuleType('singularitydog_hw._snapshot_r49_plan_test')
            module.__package__ = 'singularitydog_hw'
            module.__file__ = str(self.bundle / 'fk_cache_diagnostic_benchmark.py')
            with mock.patch.dict(sys.modules, {'diagnostic_support': fake_fk}):
                exec(compile(Path(module.__file__).read_bytes(), module.__file__, 'exec'), module.__dict__)
            argv = GenerationTests().argv() + ['--snapshot-manifest', str(self.manifest),
                                               '--snapshot-manifest-sha256', self.manifest_sha]
            output = io.StringIO(); modules = set(sys.modules)
            with mock.patch.object(module, 'collect', side_effect=AssertionError('Collector called')), \
                    mock.patch.object(module.native, 'load_library', side_effect=AssertionError('CAN loaded')), \
                    mock.patch.object(support, 'enter', side_effect=AssertionError('Snapshot loaded')), \
                    mock.patch.object(module.math_threads, 'configure_single_thread_math', return_value={}), \
                    mock.patch.object(module, '_start_source_provenance',
                                      return_value={'cadence_source_sha256': {'original': 'pin'}}), \
                    mock.patch.object(module.os, 'sched_getaffinity', return_value={0,1,2,3,4}, create=True), \
                    mock.patch.object(module.os, 'sched_setaffinity', create=True), \
                    contextlib.redirect_stdout(output):
                self.assertEqual(module.main(argv), 0)
            plan = json.loads(output.getvalue())
            self.assertEqual(plan['policy_backend_requested'], 'pinned_fk_cache_cpp')
            self.assertEqual(plan['requests_per_cycle'], 26); self.assertEqual(plan['type1_requests_per_cycle'], 0)
            proof = plan['experimental_snapshot_copy_diagnostic']
            self.assertFalse(proof['observer_selected']); self.assertFalse(proof['native_library_loaded'])
            self.assertEqual(proof['expected_executing_sources']['copied_observer']['sha256'],
                             pure_loader.sha((self.bundle / 'copy_observer.py').read_bytes()))
            self.assertFalse(plan['timing_admission_eligible']); self.assertNotIn('torch', set(sys.modules) - modules)

    def test_main_selection_required_and_abbreviation_rejected_before_support(self):
        with bundle_modules(self.bundle) as (_, support):
            fake_fk = ModuleType('diagnostic_support'); fake_fk.validate_cli = mock.Mock()
            module = ModuleType('singularitydog_hw._snapshot_r49_parse_test')
            module.__package__ = 'singularitydog_hw'; module.__file__ = str(self.bundle / 'fk_cache_diagnostic_benchmark.py')
            with mock.patch.dict(sys.modules, {'diagnostic_support': fake_fk}):
                exec(compile(Path(module.__file__).read_bytes(), module.__file__, 'exec'), module.__dict__)
            with mock.patch.object(support, 'validate_cli') as validate:
                for argv in ([], ['--snapshot-man', str(self.manifest)], ['--exec']):
                    with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                        module.main(argv)
                validate.assert_not_called(); fake_fk.validate_cli.assert_not_called()


if __name__ == '__main__':
    unittest.main()
