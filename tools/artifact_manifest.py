#!/usr/bin/env python3
"""Create/verify an explicit local artifact manifest using only the standard library."""
import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def member(root, name):
    root = Path(root).resolve()
    path = Path(name)
    if not isinstance(name, str) or not name or path.is_absolute() or '..' in path.parts:
        raise ValueError('Expected a safe relative file name')
    if '.git' in path.parts or any((root / Path(*path.parts[:i])).is_symlink() for i in range(1, len(path.parts) + 1)):
        raise ValueError('Git metadata and symbolic links are excluded')
    result = root / path
    if not result.resolve().is_relative_to(root):
        raise ValueError('File escapes root')
    return result


def load_names(path):
    names = json.loads(Path(path).read_text())
    if not isinstance(names, list) or not names or not all(isinstance(x, str) for x in names) or len(set(names)) != len(names):
        raise ValueError('A nonempty unique list of relative names is required')
    return names


def create(root, names):
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
    if manifest.get('schema') != 'singularitydog.manifest.v1' or not manifest.get('files'):
        raise ValueError('Unsupported or empty manifest')
    actual = create(root, list(manifest['files']))
    if actual != manifest:
        raise ValueError('Manifest mismatch')
    return len(actual['files'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['create', 'verify'])
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--files', type=Path)
    args = parser.parse_args()
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
        count = verify(args.root, json.loads(args.manifest.read_text()))
        print(json.dumps({'status': 'VERIFIED', 'files': count}))


if __name__ == '__main__':
    main()
