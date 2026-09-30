#!/usr/bin/env python3
"""Create/verify an explicit local artifact manifest using only the standard library."""
import argparse
import hashlib
import json
from pathlib import Path


def _canonical_name(name):
    # Validate the spelling before Path normalizes repeated separators or '.'.
    # One member must have one portable POSIX spelling in the manifest.
    if (type(name) is not str or not name or not name.isprintable() or '\\' in name
            or any(part in ('', '.', '..') for part in name.split('/'))):
        raise ValueError('Expected a canonical safe relative POSIX file name')
    return name


def _names(names):
    if type(names) is not list or not names:
        raise ValueError('A nonempty unique list of relative names is required')
    for name in names:
        _canonical_name(name)
    if len(set(names)) != len(names):
        raise ValueError('A nonempty unique list of relative names is required')
    return names


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate manifest JSON key: ' + key)
        result[key] = value
    return result


def read_json(path):
    return json.loads(Path(path).read_text(), object_pairs_hook=_unique_object)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def member(root, name):
    _canonical_name(name)
    root = Path(root).resolve()
    path = Path(name)
    if path.is_absolute():
        raise ValueError('Expected a safe relative file name')
    if '.git' in path.parts or any((root / Path(*path.parts[:i])).is_symlink() for i in range(1, len(path.parts) + 1)):
        raise ValueError('Git metadata and symbolic links are excluded')
    result = root / path
    if not result.resolve().is_relative_to(root):
        raise ValueError('File escapes root')
    return result


def load_names(path):
    return _names(read_json(path))


def create(root, names):
    _names(names)
    files = {}
    for name in names:
        p = member(root, name)
        if not p.is_file():
            raise ValueError('Missing regular file: ' + name)
        before = p.stat()
        value = digest(p)
        after = p.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError('File changed during hashing: ' + name)
        files[name] = {'sha256': value, 'bytes': after.st_size}
    return {'schema': 'singularitydog.manifest.v1', 'files': files}


def verify(root, manifest):
    if (type(manifest) is not dict or set(manifest) != {'schema', 'files'}
            or manifest.get('schema') != 'singularitydog.manifest.v1'
            or type(manifest.get('files')) is not dict or not manifest['files']):
        raise ValueError('Unsupported or empty manifest')
    _names(list(manifest['files']))
    for name, row in manifest['files'].items():
        if (type(row) is not dict or set(row) != {'sha256', 'bytes'}
                or type(row['sha256']) is not str or len(row['sha256']) != 64
                or any(c not in '0123456789abcdef' for c in row['sha256'])
                or type(row['bytes']) is not int or row['bytes'] < 0):
            raise ValueError('Invalid manifest file entry: ' + name)
    actual = create(root, list(manifest['files']))
    if actual != manifest:
        raise ValueError('Manifest mismatch')
    return len(actual['files'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['create', 'verify'])
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--files', type=Path)
    args = parser.parse_args(argv)
    if args.mode == 'create':
        if args.files is None:
            parser.error('--files is required for create')
        result = create(args.root, load_names(args.files))
        if args.manifest.resolve() in {member(args.root, n).resolve() for n in result['files']}:
            raise ValueError('Manifest cannot contain itself')
        with args.manifest.open('x') as stream:
            json.dump(result, stream, indent=2, ensure_ascii=False)
            stream.write('\n')
    else:
        count = verify(args.root, read_json(args.manifest))
        print(json.dumps({'status': 'VERIFIED', 'files': count}))


if __name__ == '__main__':
    main()
