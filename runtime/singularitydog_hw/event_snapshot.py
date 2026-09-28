"""Owned, bounded snapshots of the built-in JSON event schema.

No serialization, arbitrary-object copy hooks or shared mutable descendants.
Repeated containers are copied independently; cycles are rejected. Tuples keep
their type (and serialize like lists), while immutable scalar values are shared.
The byte budget is a conservative bound on compact ensure_ascii JSON, including
escaped strings and punctuation, rather than the actual encoded length.
"""
import math

MAX_DEPTH = 24
MAX_NODES = 20_000
MAX_BYTES = 1_048_576


def snapshot_event(value, *, max_depth=MAX_DEPTH, max_nodes=MAX_NODES, max_bytes=MAX_BYTES):
    """Copy only finite built-in JSON values, failing before a caller publishes.

    Depth zero permits a root scalar or empty container; each contained value
    adds one level. Nodes include dictionary keys and repeated references. Byte
    accounting deliberately overestimates escaping and integer text without
    constructing encoded strings. Unknown/subclass types never invoke hooks.
    """
    if (type(max_depth) is not int or not 0 <= max_depth <= 64
            or type(max_nodes) is not int or max_nodes < 1
            or type(max_bytes) is not int or max_bytes < 1):
        raise ValueError("Invalid event snapshot bounds")
    nodes_left, bytes_left = max_nodes - 1, max_bytes
    active = set()
    isfinite = math.isfinite

    def visit(recurse, item, depth):
        nonlocal nodes_left, bytes_left
        if depth > max_depth:
            raise ValueError("Event snapshot depth bound exceeded")
        kind = type(item)
        is_dict = kind is dict
        if not is_dict and kind is not list and kind is not tuple:
            raise TypeError("Expected built-in event container")
        size = len(item)
        if size and depth >= max_depth:
            raise ValueError("Event snapshot depth bound exceeded")
        nodes_left -= size * (2 if is_dict else 1)
        if nodes_left < 0:
            raise ValueError("Event snapshot node bound exceeded")
        local_bytes = 2 + size * (4 if is_dict else 2)
        identity = id(item)
        if identity in active:
            raise ValueError("Cyclic event container")
        active.add(identity)
        try:
            result = {} if is_dict else []
            entries = item.items() if is_dict else enumerate(item)
            for key, child in entries:
                if is_dict:
                    if type(key) is not str:
                        raise TypeError("Event dictionary keys must be built-in strings")
                    local_bytes += len(key) * 12 + 2
                child_kind = type(child)
                # Most audit values are immutable scalars: validate/share them
                # inline, without deepcopy dispatch, memo or recursive calls.
                if child_kind is str:
                    cost = len(child) * 12 + 2
                elif child_kind is int:
                    cost = child.bit_length() + 2
                elif child_kind is float:
                    if not isfinite(child):
                        raise ValueError("Nonfinite event number")
                    cost = 32
                elif child is None:
                    cost = 4
                elif child_kind is bool:
                    cost = 5
                elif child_kind is dict or child_kind is list or child_kind is tuple:
                    child = recurse(recurse, child, depth + 1)
                    cost = 0
                else:
                    raise TypeError("Unsupported event value type")
                local_bytes += cost
                if is_dict:
                    result[key] = child
                else:
                    result.append(child)
            bytes_left -= local_bytes
            if bytes_left < 0:
                raise ValueError("Event snapshot byte bound exceeded")
            return tuple(result) if kind is tuple else result
        finally:
            active.remove(identity)

    kind = type(value)
    if kind is dict or kind is list or kind is tuple:
        return visit(visit, value, 0)
    # Scalar roots are uncommon for events, but keep the same strict contract.
    if kind is str:
        cost = len(value) * 12 + 2
    elif kind is int:
        cost = value.bit_length() + 2
    elif kind is float:
        if not isfinite(value):
            raise ValueError("Nonfinite event number")
        cost = 32
    elif value is None:
        cost = 4
    elif kind is bool:
        cost = 5
    else:
        raise TypeError("Unsupported event value type")
    if cost > max_bytes:
        raise ValueError("Event snapshot byte bound exceeded")
    return value
