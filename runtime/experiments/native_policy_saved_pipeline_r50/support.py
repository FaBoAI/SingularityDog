"""Pure comparison primitives for an incomplete, unexecuted saved replay candidate."""
import math
import struct
from .inputs import WIDTHS, OUTPUTS, float32_bits, require

IMPLEMENTATION_STATUS = 'INCOMPLETE_ACTUAL_OBSERVER_MODEL_EXEC_NOT_IMPLEMENTED'
# These are comparison metadata only, never input/output/source/STOP guards.
IGNORED_OBSERVER_METADATA = frozenset(('run_number', 'consume_profile', 'timing_scope'))


def tree_bits(value):
    """Exact scalar type/binary64 bits, ignoring only dict insertion order."""
    kind = type(value)
    if kind is dict:
        require(all(type(key) is str for key in value), 'String keys required')
        return ('dict', tuple((key, tree_bits(value[key])) for key in sorted(value)))
    if kind in (list, tuple):
        return (kind.__name__, tuple(tree_bits(item) for item in value))
    if kind is float:
        require(math.isfinite(value), 'Finite comparison value required')
        return ('float', struct.pack('=d', value))
    require(kind in (int, bool, str, type(None)), 'Unsupported comparison value')
    return (kind.__name__, value)


def comparable_record(value):
    require(type(value) is dict, 'Observer record required')
    return {key: item for key, item in value.items() if key not in IGNORED_OBSERVER_METADATA}


def compare_observed(actual, expected):
    """Proposed per-tick assertion; no model, Torch, state access or timing."""
    require(type(actual) is dict and type(expected) is dict, 'Observer records required')
    require(set(actual['inputs']) == set(expected['inputs']) == set(WIDTHS), 'Six inputs required')
    for key, width in WIDTHS.items():
        require(float32_bits(actual['inputs'][key], width) == float32_bits(expected['inputs'][key], width),
                'Input float32 bits differ: ' + key)
    for key, width in OUTPUTS.items():
        require(float32_bits(actual[key], width) == float32_bits(expected[key], width),
                'Output float32 bits differ: ' + key)
    require(tree_bits(comparable_record(actual)) == tree_bits(comparable_record(expected)),
            'Observer result/provenance bits differ')


def abba_schedule(rounds=3):
    """Bounded planned labels only; these are not recorded measurements."""
    require(type(rounds) is int and 1 <= rounds <= 3, 'One to three bounded ABBA rounds required')
    return tuple(label for _ in range(rounds) for label in ('python_copy', 'c_copy', 'c_copy', 'python_copy'))


def reject_incomplete_exec():
    raise NotImplementedError(IMPLEMENTATION_STATUS + ': EXEC is disabled before model/native load')
