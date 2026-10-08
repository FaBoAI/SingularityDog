"""Explicit pure six-feedback codec for the ordinary unpaired bus owners.

This is a source-bound experiment, not an output admission. It adds no reader,
Future, motor command, clock or owner. The existing parser remains authoritative
for unsupported/invalid records. The profile loader separately requires this
candidate's own source validation and current disabled whole-loop evidence.
"""
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import stat

from . import native_active_transport as active

SELECTION_SCHEMA = 'singularitydog.unpaired-native-feedback-codec-selection.v1'
PROOF_SCHEMA = 'singularitydog.unpaired-native-feedback-codec-source-binding.v1'
SOURCE_PATH = 'singularitydog_hw/unpaired_native_feedback_codec.py'
_REFERENCE_NAMES = frozenset(('library', 'build_record', 'library_source',
                              'runtime', 'binding', 'selection_source'))


def _need(condition, message):
    if not condition:
        raise ValueError(message)


def _read_reference(reference):
    _need(type(reference) is dict and set(reference) == {'path', 'sha256'},
          'Exact unpaired codec source reference required')
    path, digest = reference['path'], reference['sha256']
    _need(type(path) is str and type(digest) is str and len(digest) == 64 and
          all(c in '0123456789abcdef' for c in digest), 'Codec reference shape differs')
    path = Path(path)
    _need(path.is_absolute() and not any(p.is_symlink() for p in (path, *path.parents)),
          'Regular absolute unpaired codec source required')
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    try:
        before = os.fstat(fd)
        _need(stat.S_ISREG(before.st_mode) and 0 < before.st_size <= 64*1024*1024,
              'Bounded regular unpaired codec source required')
        with os.fdopen(fd, 'rb', closefd=False) as stream:
            raw = stream.read(before.st_size + 1)
        after = os.fstat(fd)
        key = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns)
        _need(len(raw) == before.st_size and key(before) == key(after) and
              hashlib.sha256(raw).hexdigest() == digest, 'Unpaired codec source SHA differs')
        return str(path), raw
    finally:
        os.close(fd)


def verify_source_selection(selection):
    """File-only setup verification; no native function/session/device access."""
    _need(type(selection) is dict and set(selection) == {
        'schema', 'references', 'output_allowed', 'approved_for_runtime'} and
        selection['schema'] == SELECTION_SCHEMA and selection['output_allowed'] is False and
        selection['approved_for_runtime'] is False,
        'Explicit unpaired pure codec selection grants no output')
    refs = selection['references']
    _need(type(refs) is dict and set(refs) == _REFERENCE_NAMES,
          'Complete unpaired codec source/build references required')
    pins = {name: _read_reference(ref) for name, ref in refs.items()}
    from . import policy_output_runtime as runtime
    for name, module in (('runtime', runtime), ('binding', active)):
        _need(pins[name][0] == str(Path(module.__file__).absolute()),
              'Unpaired codec actual module origin differs')
    _need(pins['selection_source'][0] == str(Path(__file__).absolute()),
          'Unpaired codec selection source origin differs')
    build = json.loads(pins['build_record'][1])
    _need(type(build) is dict and type(build.get('abi')) is int and build['abi'] == 1 and
          build.get('binary_sha256') == refs['library']['sha256'] and
          build.get('source_sha256') == refs['library_source']['sha256'],
          'Unpaired codec original ordinary ABI1 build binding differs')
    library = Path(pins['library'][0])
    _need(pins['build_record'][0] == str(library.parent/'build-record.json') and
          pins['library_source'][0] == str(library.parent/'transport.cpp'),
          'Unpaired codec source/build must be adjacent to original library')
    return {name: {'path': path, 'sha256': refs[name]['sha256']}
            for name, (path, _) in pins.items()}


def prepare_unpaired_decoders(sessions, selection):
    """Authenticate the loaded libraries and construct only pure decoder scratch.

    Both existing bus locks fence setup against a concurrent exchange or pair
    reservation. They are released before returning; no owner or I/O is added.
    Partial optional ABI and capability mutation reject explicit selection.
    """
    from . import policy_output_runtime as runtime
    pins = verify_source_selection(selection)
    _need(type(sessions) is dict and set(sessions) == set(runtime.BUSES) and
          sessions['front'] is not sessions['rear'], 'Two independent unpaired buses required')
    _need(all(type(s) is active.ActiveSession for s in sessions.values()),
          'Genuine original ActiveSession required for explicit codec')
    with ExitStack() as stack:
        for scope in runtime.BUSES:
            session = sessions[scope]
            _need(session.busy.acquire(blocking=False), 'Unpaired codec session is busy')
            stack.callback(session.busy.release)
            _need(session.first_id == runtime.BUSES[scope][0] and session._phase_pair is None and
                  session._handle and not session.poisoned,
                  'Genuine idle current unpaired ActiveSession required')
            binding = active.verified_active_source_binding(session.lib)
            expected = {'library': (binding.path, binding.binary_sha256),
                        'library_source': (binding.source_path, binding.source_sha256),
                        'build_record': (binding.build_record_path, binding.build_record_sha256)}
            _need(all(pins[name] == {'path': path, 'sha256': sha}
                      for name, (path, sha) in expected.items()),
                  'Unpaired codec selection differs from actually authenticated library')
        decoders = {scope: active.NativeFeedbackBatchDecoder(session.lib)
                    for scope, session in sessions.items()}
        _need(all(decoder.available is True for decoder in decoders.values()),
              'Explicit unpaired codec requires complete exact feedback ABI1')
    proof = {'schema': PROOF_SCHEMA, 'scope': 'ordinary_unpaired_owners.v1',
             'references': pins, 'feedback_abi': 1, 'active_abi': 1,
             'adds_owner_or_future_or_request': False,
             'timestamps_and_deadlines_unchanged': True,
             'unsupported_or_invalid_uses_legacy_codec': True,
             'hardware_timing_improvement_proven': False}
    return decoders, proof
