"""Explicit FK policy binding for the small, box-supported output sequence.

PLAN authenticates the existing unapproved R49 file-only artifacts without
Torch/native loading. Only a separately admitted active profile permits load;
the frozen artifact and its diagnostic provenance never grant motor output.
"""
from contextlib import contextmanager
import copy
import hashlib
import importlib
import importlib.abc
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import stat
import sys


SOURCE_PATHS = ('singularitydog_hw/policy_active_fk.py',
                'singularitydog_hw/policy_output_model.py')
LOADER_SHA256 = '34226609eb7a7b8b52a2eecf19f2156b5c611f866d5c27c3f8404f6a6b11232a'
_PACKAGE = '_sd_active_fk_deps'
_PRIVATE = _PACKAGE + '.target_tail_fk_cache._active_pinned_r49_loader'
_DEPENDENCY_PINS = {
    '__init__.py': 'e2fada62f22c3d4e436d48f59c58dbfc7dfedebdd26ef86cc5e580df9847c470',
    'contracts.py': '9b12862441bbb791419a998515e4d019a3ba4190269d175e44aa83dc10bc5e08',
    'loader.py': '1fd54274607a8f5c085bcae764cf87790d2525a0ee9e0f65c0499bb40643735a',
    'model_call_fastpath/__init__.py': '90ee669dc14636ecfff717cc95559351ba42476f178a61e5715d1da1f49166e1',
    'model_call_fastpath/saved_input_profile.py': '28a97b55284313ad61eeb9506f3cb45beae4a337d2323a005c785e7abb57badf',
    'model_call_fastpath/scalar_loader.py': '829b0109e0064c104d689052052804f59bf40470c82ea994cbe9a549ef3d912b',
    'target_tail_fk_cache/__init__.py': '4c2e011ef5f7e94f0870986717431b4ab7b8245b50d147c9c0172d2d48b43e7f',
    'target_tail_fk_cache/generator.py': '8ef5cd705382a4a4b3d8a251e797e94cdab6139454f16d09119c4576ecc26852',
    'target_tail_fusion/__init__.py': 'e1acf2a1c36b1af4bb6e54b14c8fd453beeb0480614a5aa3a72d3025a9055d9f',
    'target_tail_fusion/generator.py': 'd4d554ed0953219b45b78212838daf0967fc49819f57b61bbd6cd94f60bcca51',
    'verification.py': '533b0f860ccc2a38b5c570ead2b0011eaeb5e326f8cf61e8fe808186fbd8c8be',
    'view_cache/__init__.py': 'de8603cddbae0b9fa25ca7800907eb85f3598ced538b717a51ab400a2ffabb6e',
    'view_cache/generator.py': 'b0ae01397ba4270d45101bb5e4370111cc0e6a159c2ddd57b530757d85377534',
    'view_cache/loader.py': 'b49499a2dc5b0a5af308fdb26948925da7d24ffbd99f0677cdacee2257c035b2',
}
_FALSE_FLAGS = ('output_allowed', 'approved_for_runtime',
                'active_controller_qualification', 'timing_admission_eligible',
                'live_50hz_verified')
_MODES = {
    2: ('supported-policy-probe-v1', 'supported-policy-probe-2s-rare-jitter-v1'),
    10: ('supported-policy-probe-10s-after-2s-v1',),
    20: ('supported-policy-probe-20s-after-10s-v1',),
    30: ('supported-policy-probe-30s-after-10s-preauthorized-v1',),
    60: ('supported-policy-probe-60s-after-20s-v1',),
}


def _need(condition, message):
    if not condition:
        raise ValueError(message)


def source_paths():
    """Additional current cadence source; R49 dependencies retain their own pins."""
    return SOURCE_PATHS


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def selected(profile):
    """Pure selection/scope check; absent/false retains the existing scalar route."""
    _need(type(profile) is dict, 'FK profile must be a dictionary')
    value = profile.get('native_target_fk_cache', False)
    _need(type(value) is bool, 'native_target_fk_cache must be an explicit boolean')
    if not value:
        return False
    duration = profile.get('duration_s')
    _need(duration != 30 or profile.get('preauthorized_boxed_sequence') is True,
          'Thirty-second FK selection requires explicit preauthorized boxed sequence')
    _need(_number(duration) and duration in _MODES and
          profile.get('diagnostic_timing_acceptance') in _MODES[duration] and
          profile.get('schema') == 'singularitydog.supported-policy-profile.v3' and
          profile.get('scope') == 'supported_characterization_only' and
          profile.get('local_characterization') == 'bounded_relative_supported_v1' and
          profile.get('watchdog_review_policy') == 'command_loss_only_supported_trial' and
          profile.get('model_backend') == 'scalar_step_cpp' and
          profile.get('policy_weight') == .005 and
          profile.get('command') == [0., 0., 0.] and
          profile.get('voltage_overlap') is True and
          profile.get('voltage_pipeline') is True and
          profile.get('period_ms') == 20 and profile.get('hard_cycle_ms') == 20 and
          _number(profile.get('max_sample_age_ms')) and 0 < profile['max_sample_age_ms'] <= 20 and
          _number(profile.get('max_sample_gap_ms')) and 0 < profile['max_sample_gap_ms'] <= 21 and
          type(profile.get('max_consecutive_20ms_misses')) is int and
          profile['max_consecutive_20ms_misses'] == 0,
          'FK selection requires the bounded boxed .005 two/ten/twenty/sixty-second contract')
    axes = profile.get('axes')
    _need(type(axes) is dict and set(axes) == {str(i) for i in range(1, 13)},
          'FK selection requires all twelve reviewed axes')
    for axis in axes.values():
        _need(type(axis) is dict and
              all(_number(axis.get(key)) for key in
                  ('kp', 'kd', 'max_displacement_from_start_rad')) and
              0 < axis['kp'] <= 3 and 0 <= axis['kd'] <= .15 and
              0 < axis['max_displacement_from_start_rad'] <= math.radians(1),
              'FK selection requires Kp<=3/Kd<=.15 and maximum one-degree displacement')
    return True


def _reference(value):
    _need(type(value) is dict and set(value) == {'path', 'sha256'} and
          type(value['path']) is str and type(value['sha256']) is str and
          re.fullmatch('[0-9a-f]{64}', value['sha256']), 'Exact FK path/SHA256 reference required')
    path = Path(value['path'])
    _need(path.is_absolute() and '..' not in path.parts and
          not any(p.is_symlink() for p in (path, *path.parents)),
          'Absolute non-symlink FK reference required')
    return dict(value)


def _read(ref):
    path = Path(_reference(ref)['path'])
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    try:
        before = os.fstat(fd)
        _need(stat.S_ISREG(before.st_mode) and before.st_size <= 4 * 1024 * 1024,
              'Bounded regular FK manifest/loader required')
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            raw = stream.read(before.st_size + 1)
        after = os.fstat(fd)
    finally:
        os.close(fd)
    _need(len(raw) == before.st_size and
          (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) ==
          (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns) and
          hashlib.sha256(raw).hexdigest() == ref['sha256'],
          'FK reference changed or SHA256 differs')
    return raw


def _json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            _need(key not in result, 'Duplicate FK JSON field')
            result[key] = value
        return result
    def reject(value):
        raise ValueError('Nonfinite FK JSON value: ' + value)
    value = json.loads(raw, object_pairs_hook=pairs, parse_constant=reject)
    _need(type(value) is dict, 'FK manifest must be a dictionary')
    return value


def _bindings(profile, documents=None):
    _need(selected(profile), 'Explicit active FK selection required')
    artifacts = profile.get('artifacts')
    _need(type(artifacts) is dict, 'FK profile artifacts required')
    refs = {key: _reference(artifacts.get(name)) for key, name in
            (('target_fk_manifest', 'target_fk_manifest'),
             ('scalar_manifest', 'scalar_step_manifest'),
             ('baseline_manifest', 'model_manifest'))}
    data = {key: _json(_read(ref)) for key, ref in refs.items()}
    if documents is not None:
        _need(type(documents) is dict, 'Parsed FK documents must be a dictionary')
        for key, name in (('target_fk_manifest', 'target_fk_manifest'),
                          ('scalar_manifest', 'scalar_step_manifest'),
                          ('baseline_manifest', 'model_manifest')):
            if name in documents:
                _need(documents[name] == data[key], 'Parsed FK document differs from pinned bytes: ' + name)
    fk = data['target_fk_manifest']
    _need(fk.get('schema') == 'singularitydog.fk-cache-stop-diagnostic-artifact.v1' and
          fk.get('status') == 'PREPARED_DIAGNOSTIC_ARTIFACT_NOT_ACTIVE_APPROVAL' and
          all(fk.get(flag) is False for flag in _FALSE_FLAGS),
          'Frozen unapproved diagnostic FK artifact required')
    _need(type(fk.get('references')) is dict and type(fk.get('integration_sources')) is dict,
          'FK reference maps required')
    _need(fk['references'].get('scalar_manifest') == refs['scalar_manifest'] and
          fk['references'].get('baseline_manifest') == refs['baseline_manifest'],
          'Active FK scalar/baseline dependency references differ')
    loader = _reference(fk['integration_sources'].get('diagnostic_loader.py'))
    _need(loader['sha256'] == LOADER_SHA256, 'Frozen R49 FK loader source required')
    return refs, loader


@contextmanager
def _loader_scope(loader_ref):
    """Compile the frozen loader and its exact private dependency sources.

    Ordinary experiment modules, including cached modules and valid stale pyc,
    never supply executable code to this scope. No search path is changed.
    """
    _read(loader_ref)
    _need(not any(name == _PACKAGE or name.startswith(_PACKAGE + '.') for name in sys.modules),
          'Active FK private loader already bound')
    folder = _dependency_folder()
    refs = {}
    for relative, pin in _DEPENDENCY_PINS.items():
        parts = relative[:-3].split('/')
        package = parts[-1] == '__init__'
        name = '.'.join((_PACKAGE, *(parts[:-1] if package else parts)))
        refs[name] = ({'path': str(folder / relative), 'sha256': pin}, package)
    refs[_PRIVATE] = (loader_ref, False)
    loaded = {}
    class SealedImporter(importlib.abc.MetaPathFinder, importlib.abc.Loader):
        def find_spec(self, fullname, path=None, target=None):
            if fullname != _PACKAGE and not fullname.startswith(_PACKAGE + '.'):
                return None
            _need(fullname in refs, 'Unknown private FK dependency: ' + fullname)
            return importlib.util.spec_from_loader(fullname, self, is_package=refs[fullname][1])
        def create_module(self, spec):
            return None
        def exec_module(self, module):
            ref, package = refs[module.__name__]
            raw = _read(ref)
            module.__file__ = ref['path']
            if package:
                module.__path__ = []
            loaded[module.__name__] = module
            exec(compile(raw, module.__file__, 'exec'), module.__dict__)
    finder = SealedImporter()
    prior_meta = list(sys.meta_path)
    primary = None
    try:
        sys.meta_path.insert(0, finder)
        module = importlib.import_module(_PRIVATE)
        namespace = dict(module.__dict__)
        yield module
        _need(sys.modules.get(_PRIVATE) is module and module.__dict__.keys() == namespace.keys() and
              all(module.__dict__[key] is value for key, value in namespace.items()),
              'Active FK loader binding changed')
    except BaseException as error:
        primary = error
        raise
    finally:
        integrity = all(sys.modules.get(name) is module for name, module in loaded.items())
        for name in list(sys.modules):
            if name == _PACKAGE or name.startswith(_PACKAGE + '.'):
                del sys.modules[name]
        sys.meta_path[:] = prior_meta
        try:
            _need(integrity, 'Active FK private dependency binding changed')
            _read(loader_ref)
            for name in loaded:
                _read(refs[name][0])
        except BaseException as error:
            if primary is None:
                raise
            primary.add_note('Active FK loader cleanup verification: ' + str(error))


def _kwargs(refs):
    return dict(expected_sha256=refs['target_fk_manifest']['sha256'],
                scalar_manifest=refs['scalar_manifest']['path'], scalar_sha=refs['scalar_manifest']['sha256'],
                baseline_manifest=refs['baseline_manifest']['path'], baseline_sha=refs['baseline_manifest']['sha256'])


def _dependency_folder():
    return Path(__file__).resolve().parents[1] / 'experiments/native_policy_overnight'


def _binding(refs):
    return {'schema': 'singularitydog.active-fk-binding.v1',
            'native_target_fk_cache': True, 'model_backend': 'scalar_step_cpp',
            **copy.deepcopy(refs),
            'adapter_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'model_artifact_grants_output': False}


def plan(profile, documents=None):
    """Authenticate all file-only FK evidence; no profile approval is inferred."""
    refs, loader_ref = _bindings(profile, documents)
    with _loader_scope(loader_ref) as loader:
        proof = loader.plan(refs['target_fk_manifest']['path'], **_kwargs(refs))
    _bindings(profile, documents)
    _need(proof.get('manifest_sha256') == refs['target_fk_manifest']['sha256'] and
          proof.get('torch_or_native_loaded') is False and
          all(proof.get(flag) is False for flag in _FALSE_FLAGS), 'FK file-only PLAN provenance differs')
    return {'schema': 'singularitydog.active-fk-file-plan.v1',
            'manifest_sha256': proof['manifest_sha256'], 'model_sha256': proof['model_sha256'],
            'library_sha256': proof['library_sha256'],
            'baseline_provenance': {'manifest_sha256': refs['baseline_manifest']['sha256']},
            'active_binding': _binding(refs), 'diagnostic_plan': copy.deepcopy(proof),
            'torch_or_native_loaded': False, 'hardware_opened': False,
            **dict.fromkeys(_FALSE_FLAGS, False)}


def verify(profile, documents=None):
    """Alias for callers that need complete file-only verification."""
    return plan(profile, documents)


def load(profile):
    """Load only after immutable active-profile admission; preserve raw FK proof."""
    from .policy_live_profile import native_target_fk_cache_settings
    refs, loader_ref = _bindings(profile)
    _need(profile.get('output_allowed') is True and
          native_target_fk_cache_settings(profile) == refs['target_fk_manifest'],
          'Admitted active FK profile required')
    expected = plan(profile)
    _need(profile.get('_native_target_fk_cache_provenance') == expected,
          'Admitted active FK file/source proof changed')
    policy, provenance = _load_candidate(profile, refs, loader_ref, expected)
    _need(native_target_fk_cache_settings(profile) == refs['target_fk_manifest'],
          'Admitted active FK profile changed during model load')
    return policy, provenance


def diagnostic_load(profile):
    """Explicit model load for disabled comparison, without active admission.

    This API neither opens hardware nor grants active output. The caller must
    separately select its disabled diagnostic; default PLAN calls only plan().
    The narrow intended boxed sequence and all file/model/source guards remain.
    """
    refs, loader_ref = _bindings(profile)
    return _load_candidate(profile, refs, loader_ref, plan(profile))


def _load_candidate(profile, refs, loader_ref, expected):
    with _loader_scope(loader_ref) as loader:
        policy, provenance = loader.load_diagnostic_verified(
            refs['target_fk_manifest']['path'], **_kwargs(refs), bundle=profile['bundle_path'])
    _need(plan(profile) == expected, 'FK file/source proof changed during model load')
    _need(type(provenance) is dict and 'active_binding' not in provenance and
          provenance.get('schema') == 'singularitydog.fk-cache-stop-diagnostic-loader.v1' and
          provenance.get('manifest_sha256') == refs['target_fk_manifest']['sha256'] and
          provenance.get('model_sha256') == expected['model_sha256'] and
          provenance.get('library_sha256') == expected['library_sha256'] and
          provenance.get('baseline_provenance', {}).get('manifest_sha256') == refs['baseline_manifest']['sha256'] and
          provenance.get('original_scalar_dependency', {}).get('manifest_sha256') == refs['scalar_manifest']['sha256'] and
          provenance.get('diagnostic_only') is True and provenance.get('hardware_opened') is False and
          all(provenance.get(flag) is False for flag in _FALSE_FLAGS),
          'Loaded active FK artifact/dependency provenance differs')
    return policy, {**copy.deepcopy(provenance), 'active_binding': _binding(refs)}
