"""Native owned-event snapshot candidate; explicit import, no silent fallback.

Build the package-root extension using build_native_event_copy.py on the executing interpreter.
This module is an optional drop-in adapter for singularitydog_hw.event_snapshot.
"""
from _event_snapshot_native import snapshot_event

MAX_DEPTH = 24
MAX_NODES = 20_000
MAX_BYTES = 1_048_576
