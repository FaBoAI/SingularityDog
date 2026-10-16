"""30Hz foreground selection and real CPU warmup; no CAN/USB/network."""
import builtins
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from singularitydog_hw import policy_model_30hz as model30
from singularitydog_hw.policy_clock import control_clock_payload
from . import foreground as fg
from .test_foreground import arguments, fake_settings
from .test_model_bridge import Fixture
from .test_pipeline_30hz import inline_model_plan

try:
    import torch
except ImportError:
    torch = None

BUNDLE = Path(__file__).resolve().parents[4] / 'RS05_30Hz_Model_20261013'


class Foreground30SelectionTests(unittest.TestCase):
    def test_30hz_plan_rejects_missing_wrong_or_boolean_native_write_deadline_receipt(self):
        for scope in ({}, {'write_deadline_abi': 0}, {'write_deadline_abi': True},
                      {'write_deadline_abi': '1'}):
            with self.subTest(scope=scope), self.assertRaisesRegex(ValueError, 'ABI1 receipt'):
                fg.require_proxy_write_deadline_receipt(scope, selected30=True)
            fg.require_proxy_write_deadline_receipt(scope, selected30=False)
        fg.require_proxy_write_deadline_receipt({'write_deadline_abi': 1}, selected30=True)

    def test_selected30_requires_verified_optional_symbols_before_devices(self):
        library = object()
        with patch.object(fg.transport_adapter, 'verify_library',
                          return_value=SimpleNamespace(write_deadline=())) as verify:
            fg.require_proxy_write_deadline_library(library, selected30=False)
            verify.assert_not_called()
            with self.assertRaisesRegex(ValueError, 'authenticated separate'):
                fg.require_proxy_write_deadline_library(library, selected30=True)
        with patch.object(fg.transport_adapter, 'verify_library',
                          return_value=SimpleNamespace(write_deadline=(object(), object()))) as verify:
            fg.require_proxy_write_deadline_library(library, selected30=True)
            verify.assert_called_once_with(library)

    def test_foreground_creation_explicitly_selects_optional_abi_only_for30hz(self):
        fixture = Fixture()
        args = fg.parser().parse_args(arguments())
        old = fixture.plan()
        legacy_config = fg.make_config(args, old, fixture.topology)
        selected = inline_model_plan()
        clock = control_clock_payload(30)
        new = fg.model_bridge.reclock_model_plan(old, clock, selected)
        selected_config = fg.make_config(args, new, fixture.topology,
                                        control_clock=clock, policy30_model=selected)
        for config, expected in ((legacy_config, False), (selected_config, True)):
            with self.subTest(selected30=expected), \
                 patch.object(fg.transport_adapter.ThreeAxisTransport, 'create', return_value='mock') as create:
                result = fg.create_selected_transport('library', 'fd', config=config,
                    group=config.groups[0], cancel_fd=1, boot_fd=2, boot_id='private-mock',
                    raw_lower_by_id={}, raw_upper_by_id={})
            self.assertEqual(result, 'mock')
            self.assertIs(create.call_args.kwargs['separate_write_deadline'], expected)
            self.assertEqual(create.call_args.kwargs['group'], config.groups[0])

    def test_legacy_selection_is_none_and_relative30_bundle_is_rejected(self):
        self.assertEqual(fg.select_policy30_bundle(None), (None, None))
        with self.assertRaisesRegex(ValueError, 'Absolute30Hz'):
            fg.select_policy30_bundle('relative/bundle')
        args = fg.parser().parse_args(arguments())
        self.assertIsNone(args.policy30_bundle)
        self.assertFalse(args.execute)

    def test_explicit_cli_selects30_candidate_only_in_plan(self):
        selected = fg.parser().parse_args(arguments()+['--policy30-bundle', '/new/candidate'])
        self.assertEqual(selected.policy30_bundle, '/new/candidate')
        self.assertFalse(selected.execute)
        planned = inline_model_plan()
        with patch.object(model30, 'plan', return_value=planned) as read, \
             patch.object(model30, 'load', side_effect=AssertionError('PLAN executed model')):
            clock, result = fg.select_policy30_bundle('/new/candidate')
        self.assertEqual(clock, control_clock_payload(30))
        self.assertIs(result, planned)
        read.assert_called_once_with(model30.bundle_artifacts('/new/candidate'))

    def test_config_binds_same_clock_and_candidate_to_pipeline(self):
        fixture = Fixture()
        old = fixture.plan()
        selected = inline_model_plan()
        clock = control_clock_payload(30)
        new = fg.model_bridge.reclock_model_plan(old, clock, selected)
        args = fg.parser().parse_args(arguments())
        config = fg.make_config(args, new, fixture.topology,
                                control_clock=clock, policy30_model=selected)
        result = fg.pipeline.plan(config)
        self.assertEqual(result['control_clock'], clock)
        self.assertEqual(result['policy30_model'], selected)
        self.assertEqual(config.profile, old['measured_input_profile'])
        self.assertEqual(config.offsets, {int(k): v for k, v in old['runtime_offsets_by_id'].items()})
        self.assertFalse(result['learned_targets_sent'])

    def test_config_rejects_30hz_model_plan_with_omitted_or_legacy_outer_clock(self):
        fixture = Fixture()
        selected = inline_model_plan()
        clock = control_clock_payload(30)
        new = fg.model_bridge.reclock_model_plan(fixture.plan(), clock, selected)
        args = fg.parser().parse_args(arguments())
        for outer_clock, outer_model in ((None, None), (None, selected),
                                         (clock, None), (control_clock_payload(50), selected)):
            with self.subTest(clock=outer_clock, model=outer_model), \
                 patch.object(model30, 'load', side_effect=AssertionError('Pure builder executed Torch/model')), \
                 patch('ctypes.CDLL', side_effect=AssertionError('Pure builder loaded native library')), \
                 self.assertRaisesRegex(ValueError, 'model-plan and outer'):
                fg.make_config(args, new, fixture.topology,
                               control_clock=outer_clock, policy30_model=outer_model)

    def test_config_rejects_legacy_model_plan_mislabeled_as_30hz(self):
        fixture = Fixture()
        old = fixture.plan()
        args = fg.parser().parse_args(arguments())
        with self.assertRaisesRegex(ValueError, 'model-plan and outer'):
            fg.make_config(args, old, fixture.topology,
                           control_clock=control_clock_payload(30), policy30_model=inline_model_plan())
        result = fg.make_config(args, old, fixture.topology)
        self.assertIsNone(result.control_clock)
        self.assertIsNone(result.policy30_model)

    def test_config_rejects_different_new_model_even_with_matching_30hz_rate(self):
        fixture = Fixture()
        selected = inline_model_plan()
        clock = control_clock_payload(30)
        new = fg.model_bridge.reclock_model_plan(fixture.plan(), clock, selected)
        other = model30._plan(model30.bundle_artifacts('/another-file-only-mock30'),
            {'source_manifest': dict.fromkeys(model30._SOURCE_NAMES),
             'export': {'evaluation_all_intervals_completed': False}})
        with self.assertRaisesRegex(ValueError, 'model-plan and outer'):
            fg.make_config(fg.parser().parse_args(arguments()), new, fixture.topology,
                           control_clock=clock, policy30_model=other)

    def test_config_revalidates_model_plan_observer_clock_without_torch_or_io(self):
        fixture = Fixture()
        selected = inline_model_plan()
        clock = control_clock_payload(30)
        new = fg.model_bridge.reclock_model_plan(fixture.plan(), clock, selected)
        changed = copy.deepcopy(new)
        changed['observer_kwargs']['control_clock'] = control_clock_payload(50)
        # Even recomputing the plain-data canonical hash cannot make an
        # internally inconsistent observer/model/cadence contract admissible.
        import hashlib
        payload = {key: value for key, value in changed.items() if key != 'plan_canonical_sha256'}
        changed['plan_canonical_sha256'] = hashlib.sha256(fg.model_bridge._canonical(payload).encode()).hexdigest()
        with self.assertRaisesRegex(ValueError, 'model/clock/geometry binding'):
            fg.make_config(fg.parser().parse_args(arguments()), changed, fixture.topology,
                           control_clock=clock, policy30_model=selected)

    @unittest.skipUnless(BUNDLE.exists(), 'Pinned distribution is unavailable')
    def test_real_bundle_selection_without_importing_torch_or_opening_devices(self):
        original_import = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name == 'torch' or name.startswith('torch.'):
                raise AssertionError('Pure30Hz selection imported Torch')
            return original_import(name, *args, **kwargs)
        with patch('builtins.__import__', side_effect=guarded), \
             patch('ctypes.CDLL', side_effect=AssertionError('PLAN loaded native library')):
            clock, selected = fg.select_policy30_bundle(str(BUNDLE))
        self.assertEqual(clock['policy_hz'], 30)
        self.assertFalse(selected['cpu_model_executed'])
        self.assertFalse(selected['output_allowed'])

    @unittest.skipUnless(BUNDLE.exists(), 'Pinned distribution is unavailable')
    def test_full_prepare30_uses_old_metadata_but_never_loads_legacy_models(self):
        fixture = Fixture()
        old_model_plan = fixture.plan()
        args = fg.parser().parse_args(arguments()+['--policy30-bundle', str(BUNDLE)])
        args.power_epoch = fixture.topology['motor_power_epoch']
        profile = copy.deepcopy(fixture.profile)
        profile['review'] = None
        manifest = {'files': {'runtime/'+name: {'sha256': '0'*64} for name in
                              fg.model_bridge.MODEL_SOURCES}}
        manifest['files'].update({
            'runtime/experiments/native_active_transport/transport.cpp': {'sha256': args.ordinary_source_sha256},
            'runtime/experiments/four_bus_diagnostic/subset_stop.cpp': {'sha256': args.subset_source_sha256}})
        historical = {'file_count': 820, 'files': {'runtime/'+name: {'sha256': 'f'*64}
                      for name in fg.model_bridge.MODEL_SOURCES}}
        for index in range(820-len(historical['files'])):
            historical['files']['history/'+str(index)] = {'sha256': '1'*64}
        source = {'four_bus_source_manifest': {'path': args.source_manifest,
            'sha256': args.source_manifest_sha256},
            'frozen_model_source_manifest': {'path': '/history/manifest', 'sha256': '9'*64},
            'frozen_model_source_sha256': {name: 'f'*64 for name in fg.model_bridge.MODEL_SOURCES}}
        build = {'abi': 1, 'binary_sha256': args.subset_library_sha256,
            'source_sha256': args.subset_source_sha256,
            'four_bus_subset_stop': {'schema': 'singularitydog.four-bus-subset-build.v1',
                'original_active_abi': 1, 'ordinary_source_sha256': args.ordinary_source_sha256,
                'extension_source_sha256': args.subset_source_sha256,
                'allowed_masks': [7, 56], 'abi': 1, 'write_deadline_abi': 1, 'output_allowed': False,
                'timing_admission_eligible': False}}
        documents = {args.source_manifest: manifest, args.source_binding: source,
            '/history/manifest': historical, args.topology: fixture.topology,
            str(Path(args.subset_library).parent/'build-record.json'): build}
        class FileMocks:
            def __init__(self):
                self.values = {}
            def read(self, path, digest):
                self.values[str(path)] = digest
                if str(path) == args.original_model_profile:
                    return json.dumps(profile).encode()
                return b'file-only fixture\n'
            def json(self, path, digest):
                self.values[str(path)] = digest
                return copy.deepcopy(documents[str(path)])
            def envelope(self, reference):
                return {'reference': copy.deepcopy(reference), 'raw_json': '{}'}
            def verify(self):
                pass
        from singularitydog_hw import policy_active_fk, policy_checked_dispatch, policy_live_profile
        with patch.object(fg, 'Pins', FileMocks), patch.object(fg, 'verify_manifest'), \
             patch.object(fg, 'verify_origins'), patch.object(fg, 'fresh_output', return_value=Path('/fresh')), \
             patch.object(fg, 'prepare_native_guard', return_value={'file_only_fixture': True}), \
             patch.object(fg.model_bridge, 'prepare_model_plan', return_value=old_model_plan), \
             patch.object(policy_active_fk, 'plan', side_effect=AssertionError('Old FK selected')), \
             patch.object(policy_checked_dispatch, 'plan', side_effect=AssertionError('Old checked selected')), \
             patch.object(policy_live_profile, 'load_profile', side_effect=AssertionError('Old model loader selected')), \
             patch.object(model30, 'load', side_effect=AssertionError('PLAN executed30Hz model')):
            result = fg.prepare(args)
        self.assertEqual(result['plan']['schema'], fg.SCHEMA_30HZ)
        self.assertIsNone(result['plan']['fk_plan'])
        self.assertIsNone(result['plan']['checked_plan'])
        self.assertFalse(result['plan']['legacy_model_kernels_loaded'])
        self.assertEqual(result['plan']['control_clock'], result['config'].control_clock)
        self.assertEqual(result['plan']['policy30_model'], result['config'].policy30_model)
        self.assertEqual(result['model_plan']['model_only_loader'], 'policy_model_30hz.load(execute_cpu=True)')
        self.assertEqual(result['plan']['input_to_stop_proxy_tx_budget_ns'], 20_000_000)
        self.assertEqual(result['plan']['acquisition_budget_ns'], 20_000_000)
        self.assertEqual(result['plan']['proxy_exchange_budget_ns'], 33_333_334)
        self.assertEqual(result['plan']['proxy_write_deadline_abi_required'], 1)
        self.assertFalse(result['plan']['diagnostic_reply_budget_unchanged'])
        self.assertFalse(result['plan']['diagnostic_type1_33ms_reply_qualification'])


@unittest.skipIf(torch is None or not BUNDLE.exists(), 'CPU Torch/pinned distribution is unavailable')
class Foreground30WarmupTests(unittest.TestCase):
    def test_actual_new_controller_two10_call_stages_then_one_reset(self):
        policy, proof = model30.load(model30.bundle_artifacts(BUNDLE), execute_cpu=True)
        inputs = tuple(torch.empty((1, size), dtype=torch.float32) for size in (3, 3, 3, 12, 12, 12))
        def reset(*, warmup_completed):
            self.assertTrue(warmup_completed)
            self.assertAlmostEqual(float(policy.controller.elapsed[0]), 20/30., places=14)
            policy.reset(torch.tensor([0], dtype=torch.long))
        observer = SimpleNamespace(_input_tensors=inputs, prepare_run=Mock(side_effect=reset))
        with fake_settings() as settings:
            settings['cpus'] = {4}
            warm = fg.selected_warmup(observer, policy, None, torch, 0., pre_calls=10, post_calls=10)
        observer.prepare_run.assert_called_once_with(warmup_completed=True)
        self.assertEqual(float(policy.controller.elapsed[0]), 0.)
        self.assertEqual(float(policy.controller.phase[0]), 0.)
        self.assertEqual(warm['pre_calls'], 10)
        self.assertEqual(warm['post_calls'], 10)
        self.assertTrue(warm['selected_model_warmup_verified'])
        self.assertFalse(proof['model_artifact_grants_output'])


if __name__ == '__main__':
    unittest.main()
