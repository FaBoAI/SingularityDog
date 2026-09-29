"""Verified, opt-in loader for the offline twelve-axis Type1 encoder candidate.

Loading this module does not open hardware or authorize output. A separately
reviewed caller must provide a pinned binary hash and twelve immutable axis
specs before it can obtain an encoder. The active runner does not import it.
"""

import hashlib
import importlib.util
import math
from pathlib import Path
import random
import re
from types import SimpleNamespace

from .can_readonly import ATParser
from .native_active_transport import encode_motion


PINNED_SOURCE_SHA256 = {
    'batch_encode.cpp': '73e3e28bf80800c8ea6a2b12320f783c247b706f1964b46122c3e9a5b78238ea',
    'batch_encode_py.cpp': '7210ee9cce727e5b31cac697893753d0355813fc17e5d9d34a709883c7fa3908',
}
DEFAULT_SOURCE_DIR = (Path(__file__).resolve().parents[1] / 'experiments' /
                      'native_policy_batch_encode')


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _pinned_file(path, digest, label):
    if (type(digest) is not str or re.fullmatch(r'[0-9a-f]{64}', digest) is None or
            not path.is_file() or path.is_symlink() or _sha256(path) != digest):
        raise ValueError(label + ' is absent, linked, or differs from its reviewed SHA256')


def _command(q, kp, kd, estimated):
    return SimpleNamespace(q_model_rad=tuple(q), kp=tuple(kp), kd=tuple(kd),
                           estimated_pd_torque_nm=tuple(estimated))


def _reference_wires(command, specs):
    """Mirror the current per-axis encode, parse and quantized safety checks."""
    raws = {mid: (command.q_model_rad[mid - 1] - specs[mid - 1][0]) /
            specs[mid - 1][1] for mid in range(1, 13)}
    result = {
        'front': [encode_motion(mid, raws[mid], command.kp[mid - 1], command.kd[mid - 1])
                  for mid in range(1, 7)],
        'rear': [encode_motion(mid, raws[mid], command.kp[mid - 1], command.kd[mid - 1])
                 for mid in range(7, 13)],
    }
    for wires in result.values():
        for wire in wires:
            frame = ATParser().feed(wire)[0]
            mid = frame.destination
            offset, sign, lower, upper, initial, displacement, estimated_max = specs[mid - 1]
            raw = int.from_bytes(frame.data[:2], 'big') * 25.14 / 65535 - 12.57
            q = sign * raw + offset
            if not lower <= q <= upper:
                raise RuntimeError(f'ID{mid} quantized target outside physical range')
            if not abs(q - initial) <= displacement:
                raise RuntimeError(f'ID{mid} quantized target outside trial displacement')
            estimated = (command.kp[mid - 1] * (q - command.q_model_rad[mid - 1]) +
                         command.estimated_pd_torque_nm[mid - 1])
            if not abs(estimated) <= estimated_max:
                raise RuntimeError(f'ID{mid} quantized estimated PD torque')
    return result


def _test_specs():
    rows = []
    for mid in range(1, 13):
        q = .025 * mid
        sign = -1. if mid % 3 == 0 else 1.
        rows.append((q - sign * q, sign, q - .08, q + .08, q, .04, .3))
    return tuple(rows)


def _same_rejection(module, context, command, specs):
    try:
        _reference_wires(command, specs)
    except (ValueError, RuntimeError) as expected:
        try:
            module.encode(context, command)
        except type(expected) as actual:
            if str(actual) == str(expected):
                return
    raise ValueError('Native encoder rejection parity differs')


def _check_candidate_parity(module):
    """Run fixed, hardware-free canaries before returning any encoder."""
    specs = _test_specs()
    context = module.make_context(specs)
    rng = random.Random(92029)
    initial = tuple(row[4] for row in specs)
    for _ in range(96):
        command = _command(
            (q + rng.uniform(-.015, .015) for q in initial),
            (rng.uniform(0., 10.) for _ in range(12)),
            (rng.uniform(0., .5) for _ in range(12)),
            (rng.uniform(-.01, .01) for _ in range(12)))
        if module.encode(context, command) != _reference_wires(command, specs):
            raise ValueError('Native encoder byte parity differs')
    for mid in range(1, 13):
        for field, value in (('q_model_rad', math.nan), ('kp', 37.), ('kd', 1.1)):
            rows = {name: list(getattr(_command(initial, (3.,) * 12,
                                                  (.15,) * 12, (0.,) * 12), name))
                    for name in ('q_model_rad', 'kp', 'kd', 'estimated_pd_torque_nm')}
            rows[field][mid - 1] = value
            command = _command(*(rows[name] for name in
                                 ('q_model_rad', 'kp', 'kd', 'estimated_pd_torque_nm')))
            _same_rejection(module, context, command, specs)
        for field, value in ((2, initial[mid - 1] + .001),
                             (5, .000001), (6, .000001)):
            changed = [list(row) for row in specs]
            changed[mid - 1][field] = value
            changed = tuple(tuple(row) for row in changed)
            changed_context = module.make_context(changed)
            command = _command(initial, (3.,) * 12, (.15,) * 12, (0.,) * 12)
            _same_rejection(module, changed_context, command, changed)


class VerifiedBatchEncoder:
    """Pure command-to-bytes adapter; owning runner retains all output gates."""

    def __init__(self, module, axis_specs, binary_sha256):
        axis_specs = tuple(tuple(row) for row in axis_specs)
        self._module = module
        self._context = module.make_context(axis_specs)
        self.binary_sha256 = binary_sha256
        # A real profile's ID order, offsets and physical limits get one
        # positive byte-for-byte canary before the adapter is returned.
        initial = tuple(row[4] for row in axis_specs)
        canary = _command(initial, (0.,) * 12, (0.,) * 12, (0.,) * 12)
        if module.encode(self._context, canary) != _reference_wires(canary, axis_specs):
            raise ValueError('Native encoder profile byte parity differs')

    def __call__(self, command):
        return self._module.encode(self._context, command)


class VerifiedBatchModule:
    """Verified extension loaded before enable; bind fresh axis specs later."""

    def __init__(self, module, binary_sha256):
        self._module = module
        self.binary_sha256 = binary_sha256

    def bind(self, axis_specs):
        return VerifiedBatchEncoder(self._module, axis_specs, self.binary_sha256)


def load_verified_module(binary_path, *, expected_binary_sha256,
                         source_dir=DEFAULT_SOURCE_DIR):
    """Verify reviewed sources and binary, then load only the file-only ABI.

    The binary hash must come from a separately reviewed output profile; this
    function never infers or approves it. Call ``bind`` only after the fresh
    initial sample has fixed each axis's starting displacement reference.
    """
    source_dir = Path(source_dir)
    for name, digest in PINNED_SOURCE_SHA256.items():
        _pinned_file(source_dir / name, digest, name)
    binary_path = Path(binary_path)
    _pinned_file(binary_path, expected_binary_sha256, 'Native encoder binary')
    spec = importlib.util.spec_from_file_location('sdbe_native', binary_path)
    if spec is None or spec.loader is None:
        raise ValueError('Native encoder binary is not a loadable Python extension')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _pinned_file(binary_path, expected_binary_sha256, 'Native encoder binary after load')
    if not callable(getattr(module, 'make_context', None)) or not callable(getattr(module, 'encode', None)):
        raise ValueError('Native encoder ABI differs')
    _check_candidate_parity(module)
    return VerifiedBatchModule(module, expected_binary_sha256)


def load_verified_encoder(binary_path, *, expected_binary_sha256, axis_specs,
                          source_dir=DEFAULT_SOURCE_DIR):
    """Convenience for offline use when the final axis specs are already known.

    ``axis_specs`` has twelve rows in motor-ID order. Each row is
    (offset, sign, physical lower/upper, initial q, displacement cap,
    estimated PD cap).
    """
    return load_verified_module(binary_path,
        expected_binary_sha256=expected_binary_sha256,
        source_dir=source_dir).bind(axis_specs)
