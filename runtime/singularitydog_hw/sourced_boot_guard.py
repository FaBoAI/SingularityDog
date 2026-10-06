"""Explicit, byte-pinned boot guard for disabled STOP-proxy experiments.

Planning reads metadata and bytes only. Loading returns a factory, never opens
the procfs descriptor, and does not alter the default or active-output guard.
The experiment must retain this additional provenance; the existing cadence
source graph alone is not evidence of the selected extension's execution.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import platform
import re
import sys
import sysconfig
import threading
import types

from . import sensor_pipeline_benchmark as _reference

SCHEMA = 'singularitydog.sourced-boot-guard.v1'
MODE = 'native_retained_gil_fresh_pread'
SCOPE = 'disabled_stop_proxy_diagnostic_only'
_NATIVE_NAME = '_native_boot_guard'
_FILE_KEYS = {'cpp', 'adapter', 'build_script', 'build_record', 'library', 'reference_guard'}
_ENV_KEYS = {'sys_version', 'SOABI', 'EXT_SUFFIX', 'platform'}
_APPROVED = {
    'cpp': ('native_boot_guard.cpp', '3d178288a825b34766e9123a3e01c9d425879aaae59620029c187fe47458229e'),
    'adapter': ('native_guard.py', '64e26b85546703e31d86b4cf2d0eb790e51e7121dd93298354b4f1d3c4392fa2'),
    'build_script': ('build_native_boot_guard.py', '263f5cea48d21772d2e80d4802515a1b3283d2cf69c8730052467bab262f35e1'),
    'reference_guard': ('sensor_pipeline_benchmark.py', 'e7fde259aada55a69b65ae87381785166ee5ecad71e05919af3089a117c81e00'),
}
_BUILD_KEYS = {'status', 'python', 'platform', 'command', 'compiler_version',
               'source_sha256', 'library', 'library_sha256'}
_LOAD_LOCK = threading.RLock()
_NATIVE_BINDING = None
_FACTORY_TOKEN = object()


def _need(condition, message):
    if not condition:
        raise ValueError(message)


def _environment():
    return {'sys_version': sys.version, 'SOABI': sysconfig.get_config_var('SOABI'),
            'EXT_SUFFIX': sysconfig.get_config_var('EXT_SUFFIX'), 'platform': platform.platform()}


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _json(data):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            _need(key not in result, 'Duplicate manifest/build-record key: ' + key)
            result[key] = value
        return result
    return json.loads(data, object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))


def _read_reference(reference, label):
    _need(type(reference) is dict and set(reference) == {'path', 'sha256'}, label + ' requires exact path/sha256')
    name, digest = reference['path'], reference['sha256']
    _need(type(name) is str and name and type(digest) is str and
          re.fullmatch('[0-9a-f]{64}', digest) is not None, label + ' has invalid path/digest')
    path = Path(name)
    _need(path.is_absolute() and str(path) == name and not path.is_symlink() and
          path.resolve(strict=True) == path and path.is_file(), label + ' requires a canonical regular file')
    data = path.read_bytes()
    _need(hashlib.sha256(data).hexdigest() == digest, label + ' bytes changed')
    return path, data


def _validate(reference):
    _, raw = _read_reference(reference, 'Guard manifest')
    manifest = _json(raw)
    _need(type(manifest) is dict and set(manifest) == {'schema', 'mode', 'scope', 'environment', 'files'},
          'Guard manifest schema keys differ')
    _need(manifest['schema'] == SCHEMA and manifest['mode'] == MODE and manifest['scope'] == SCOPE,
          'Guard selection is restricted to disabled STOP-proxy diagnostics')
    environment = manifest['environment']
    _need(type(environment) is dict and set(environment) == _ENV_KEYS and
          all(type(value) is str and value for value in environment.values()) and
          environment == _environment(), 'Guard target Python/ABI/platform differs')
    files = manifest['files']
    _need(type(files) is dict and set(files) == _FILE_KEYS, 'Guard requires all six exact file references')
    paths, contents = {}, {}
    for key in sorted(_FILE_KEYS):
        paths[key], contents[key] = _read_reference(files[key], 'Guard ' + key)
        if key in _APPROVED:
            filename, digest = _APPROVED[key]
            _need(paths[key].name == filename and files[key]['sha256'] == digest,
                  'Unreviewed guard source: ' + key)
    _need(len(set(paths.values())) == len(paths), 'Guard file references must be distinct')
    _need(paths['reference_guard'] == Path(_reference.__file__).resolve(), 'Guard reference module path differs')
    _need(paths['library'].name == _NATIVE_NAME + environment['EXT_SUFFIX'] and
          environment['EXT_SUFFIX'].endswith('.so'), 'Guard library ABI filename differs')
    build = _json(contents['build_record'])
    _need(type(build) is dict and set(build) == _BUILD_KEYS and build['status'] == 'BUILT',
          'Guard build record contract differs')
    _need(build['python'] == environment['sys_version'] and build['platform'] == environment['platform'] and
          build['source_sha256'] == files['cpp']['sha256'] and
          build['library'] == str(paths['library']) and build['library_sha256'] == files['library']['sha256'],
          'Guard build source/binary/environment pins differ')
    command = build['command']
    _need(type(command) is list and command and all(type(value) is str and value for value in command) and
          str(paths['cpp']) in command and str(paths['library']) in command and
          type(build['compiler_version']) is str and build['compiler_version'], 'Invalid guard build command')
    loader = Path(__file__).resolve()
    return {'schema': 'singularitydog.sourced-boot-guard-provenance.v1', 'mode': MODE, 'scope': SCOPE,
            'manifest': copy.deepcopy(reference), 'files': copy.deepcopy(files), 'environment': copy.deepcopy(environment),
            'selection_loader': {'path': str(loader), 'sha256': hashlib.sha256(loader.read_bytes()).hexdigest()},
            'native_module_name': _NATIVE_NAME, 'native_module_path': str(paths['library']),
            'native_library_loaded': False, 'byte_pins_verified': True, 'source_files_unchanged': None,
            'GIL_policy': 'retained_across_fstat_and_pread', 'fresh_pread_each_check': True,
            'pread_offset': 0, 'max_read_bytes': 80, 'EINTR_attempt_limit_per_syscall': 8,
            'inherits_reference_owned_fd_lock_and_close': True, 'existing_cadence_graph_alone_sufficient': False,
            'motor_enable_available': False, 'learned_targets_available': False,
            'output_allowed': False, 'timing_qualification': False, 'active_output_eligible': False}


def plan_sourced_boot_guard(reference):
    """Verify metadata and all bytes without importing adapter/native code."""
    return _validate(reference)


def _module_path(module, expected):
    _need(type(module) is types.ModuleType and Path(module.__file__).resolve() == expected and
          module.__spec__ is not None and Path(module.__spec__.origin).resolve() == expected,
          'Loaded guard extension origin differs')


def _load_native(path, digest):
    global _NATIVE_BINDING
    if _NATIVE_NAME in sys.modules:
        module = sys.modules[_NATIVE_NAME]
        _need(_NATIVE_BINDING is not None and _NATIVE_BINDING[:3] == (module, str(path), digest),
              'Existing native guard module is different or has no verified load provenance')
        _module_path(module, path)
        _need(module.fresh_boot_matches is _NATIVE_BINDING[3], 'Loaded guard symbol changed')
        return module
    loader = importlib.machinery.ExtensionFileLoader(_NATIVE_NAME, str(path))
    spec = importlib.util.spec_from_file_location(_NATIVE_NAME, path, loader=loader)
    _need(spec is not None and spec.loader is loader, 'Guard extension spec unavailable')
    module = importlib.util.module_from_spec(spec)
    try:
        sys.modules[_NATIVE_NAME] = module
        loader.exec_module(module)
        _module_path(module, path)
        _need(callable(getattr(module, 'fresh_boot_matches', None)), 'Guard extension symbol missing')
    except BaseException:
        if sys.modules.get(_NATIVE_NAME) is module:
            del sys.modules[_NATIVE_NAME]
        raise
    _NATIVE_BINDING = (module, str(path), digest, module.fresh_boot_matches)
    return module


@dataclass(frozen=True)
class _GuardFactory:
    _reference_json: str
    _plan_json: str
    _native: object
    _adapter: object
    _guard_type: object
    _symbol: object
    _method_bindings: tuple
    _reference_bindings: tuple
    _token: object

    def verify(self):
        """Recheck all artifact/source bytes and loaded identities at shutdown."""
        _need(self._token is _FACTORY_TOKEN, 'Unvalidated guard factory')
        current = _validate(_json(self._reference_json))
        _need(_canonical(current) == self._plan_json, 'Guard selection/source changed after load')
        _need(sys.modules.get(_NATIVE_NAME) is self._native and
              self._native.fresh_boot_matches is self._symbol, 'Native guard module/symbol changed after load')
        _module_path(self._native, Path(current['native_module_path']))
        _need(self._adapter.NativeBootIdentityGuard is self._guard_type and
              self._adapter.fresh_boot_matches is self._symbol and
              self._adapter.BootIdentityGuard is _reference.BootIdentityGuard and
              self._guard_type.__bases__ == (_reference.BootIdentityGuard,) and
              self._guard_type.close is _reference.BootIdentityGuard.close,
              'Guard adapter/reference ownership changed after load')
        _need(tuple((method, method.__code__) for method in
                    (self._guard_type.__init__, self._guard_type.check, self._guard_type.close)) ==
              self._method_bindings and
              tuple((method, method.__code__) for method in
                    (_reference.BootIdentityGuard.__init__, _reference.BootIdentityGuard.check,
                     _reference.BootIdentityGuard.close)) == self._reference_bindings,
              'Guard check/initializer/close implementation changed after load')
        result = self.provenance()
        result['source_files_unchanged'] = True
        return result

    def provenance(self):
        result = _json(self._plan_json)
        result['native_library_loaded'] = True
        return result

    def __call__(self):
        self.verify()  # Startup only; no hash/manifest work in per-cycle check.
        return self._guard_type()


def load_sourced_boot_guard_factory(reference):
    """Load the pinned extension/adapter; caller later owns guard creation/close."""
    with _LOAD_LOCK:
        plan = _validate(reference)
        _need(sys.implementation.name == 'cpython' and sys.platform == 'linux',
              'Native runtime guard requires Linux CPython')
        library = Path(plan['native_module_path'])
        native = _load_native(library, plan['files']['library']['sha256'])
        name = '_sourced_boot_guard_adapter_' + reference['sha256']
        spec = importlib.util.spec_from_file_location(name, plan['files']['adapter']['path'])
        _need(spec is not None and spec.loader is not None, 'Guard adapter spec unavailable')
        adapter = importlib.util.module_from_spec(spec)
        adapter_path, adapter_bytes = _read_reference(plan['files']['adapter'], 'Guard adapter')
        # Execute the hashed source bytes, never an unpinned timestamp-valid
        # .pyc cache that a SourceFileLoader might otherwise accept.
        exec(compile(adapter_bytes, str(adapter_path), 'exec'), adapter.__dict__)
        guard_type = getattr(adapter, 'NativeBootIdentityGuard', None)
        _need(type(guard_type) is type and guard_type.__bases__ == (_reference.BootIdentityGuard,) and
              adapter.fresh_boot_matches is native.fresh_boot_matches and
              adapter.BootIdentityGuard is _reference.BootIdentityGuard and
              guard_type.close is _reference.BootIdentityGuard.close, 'Guard adapter inheritance differs')
        _need(_validate(reference) == plan, 'Guard files changed during native/adapter load')
        factory = _GuardFactory(_canonical(reference), _canonical(plan), native, adapter,
                                guard_type, native.fresh_boot_matches,
                                tuple((method, method.__code__) for method in
                                      (guard_type.__init__, guard_type.check, guard_type.close)),
                                tuple((method, method.__code__) for method in
                                      (_reference.BootIdentityGuard.__init__, _reference.BootIdentityGuard.check,
                                       _reference.BootIdentityGuard.close)), _FACTORY_TOKEN)
        factory.verify()
        return factory
