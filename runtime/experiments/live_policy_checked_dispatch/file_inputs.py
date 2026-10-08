"""File-only pin checks shared by the explicit component entry points."""
import hashlib
import json
import math
from pathlib import Path
import re
import statistics


def need(condition, message):
    if not condition:
        raise ValueError(message)


def pinned(path, digest):
    path = Path(path)
    need(path.is_absolute() and path.is_file() and
         not any(p.is_symlink() for p in (path, *path.parents)),
         'Absolute regular nonsymlink input required: ' + str(path))
    need(type(digest) is str and re.fullmatch(r'[0-9a-f]{64}', digest),
         'Explicit SHA256 required: ' + str(path))
    raw = path.read_bytes()
    need(hashlib.sha256(raw).hexdigest() == digest, 'Input SHA differs: ' + str(path))
    return raw


def reference(ref, pins):
    need(type(ref) is dict and set(ref) == {'path', 'sha256'}, 'Exact path/SHA reference required')
    raw = pinned(ref['path'], ref['sha256'])
    need(ref['path'] not in pins or pins[ref['path']] == ref['sha256'], 'Conflicting input pin')
    pins[ref['path']] = ref['sha256']
    return raw


def distribution(values):
    rows = sorted(values)
    return {'count': len(rows), 'median_us': statistics.median(rows) / 1000,
            'p99_us': rows[math.ceil(.99 * len(rows)) - 1] / 1000,
            'max_us': max(rows) / 1000}


def source_inventory(runtime, manifest_ref, pins):
    """An explicit immutable inventory is provenance, never a runtime grant."""
    raw = reference(manifest_ref, pins)
    manifest = json.loads(raw)
    rows = manifest['files']
    need(type(rows) is dict and type(manifest['file_count']) is int and
         manifest['file_count'] == len(rows) > 0 and
         manifest['output_allowed'] is manifest['approved_for_runtime'] is False,
         'Exact source-only inventory count and false approval flags required')
    runtime = Path(runtime)
    need(runtime.is_absolute() and runtime.is_dir() and
         not any(p.is_symlink() for p in (runtime, *runtime.parents)), 'Absolute runtime root required')
    for name, row in rows.items():
        relative = Path(name)
        need(not relative.is_absolute() and '..' not in relative.parts,
             'Safe inventory relative path required')
        path = runtime.parent / relative
        data = reference({'path': str(path), 'sha256': row['sha256']}, pins)
        need(len(data) == row['bytes'] and path.stat().st_mode & 0o777 == row['mode'],
             'Inventory size/mode differs: ' + name)
    return manifest


def verify_pins(pins):
    for path, digest in pins.items():
        pinned(path, digest)


def fresh_output(path):
    path = Path(path).absolute()
    need(not path.exists() and path.parent.is_dir() and
         not any(p.is_symlink() for p in (path, *path.parents)), 'Fresh output directory required')
    return path
