#!/usr/bin/env python3
"""Check an explicit publication list for binary/vendor assets and local connection data."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import zipfile
from artifact_manifest import load_names, member, digest

TEXT_SUFFIXES = {'.md', '.py', '.json', '.yaml', '.yml', '.toml', '.txt', '.html', '.urdf', '.csv'}
BLOCKED_PARTS = {'.git', 'vendor', 'meshes', 'weights', 'checkpoints', 'raw_logs'}
RECORDING_AUTHORIZATION = 'user_selected_six_history_assets_and_l05_l06_l07_l08_l09_l10_l11_l12_results_2026-09-10'
PRINT_AUTHORIZATION = 'user_selected_next_ten_print_parts_2026-09-12'
PRINT_DIRECTORIES = (Path('docs/printing/next-10-parts'), Path('docs/printing/yellow-dense'))
PATTERNS = {
    'private-network-address': re.compile(r'(?<![\d.])(?:10\.\d{1,3}\.\d{1,3}\.\d{1,3}|192\.168\.\d{1,3}\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3})(?![\d.])'),
    'local-home-path': re.compile(r'/(?:Users|home)/[A-Za-z0-9_.-]+/'),
    'private-key': re.compile(r'-----BEGIN (?:OPENSSH |RSA |EC )?PRIVATE KEY-----'),
    'credential-assignment': re.compile(r'(?i)[\"\']?(?:password|access_token|api_key|secret_key)[\"\']?\s*[:=]\s*[\"\'][^\"\'\n]{3,}[\"\']'),
    'host-login': re.compile(r'\b[A-Za-z0-9_.-]+@(?:\d{1,3}\.){3}\d{1,3}\b'),
}


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate publication registry key: ' + key)
        result[key] = value
    return result


def _media_payload_matches(path, entry):
    payload = path.read_bytes()
    valid_type = ((path.suffix == '.gif' and payload[:6] in (b'GIF87a', b'GIF89a'))
                  or (path.suffix == '.png' and payload[:8] == b'\x89PNG\r\n\x1a\n')
                  or (path.suffix == '.mp4' and payload[4:8] == b'ftyp'))
    return valid_type and digest(path) == entry['sha256'] and len(payload) == entry['bytes']


def approved_media(root):
    path = Path(root) / 'evidence/media-publication.json'
    if not path.exists():
        return {}
    data = json.loads(path.read_text())
    if data.get('schema') != 'singularitydog.reviewed-media.v1' or data.get('vendor_geometry_used') is not False:
        raise ValueError('Wrong media review scope')
    for key in ('measured_data', 'generator'):
        item = data[key]
        if digest(member(root, item['path'])) != item['sha256']:
            raise ValueError('Media source or generator changed after review')
    for name, entry in data['files'].items():
        if Path(name).parent != Path('docs/media') or entry.get('reviewed_as_own_measured_visualization') is not True:
            raise ValueError('Only explicitly reviewed measured visualizations may be published')
    return data['files']


def approved_recordings(root):
    path = member(root, 'evidence/recording-publication.json')
    if not path.exists():
        return {}
    data = json.loads(path.read_text(), object_pairs_hook=_unique_object)
    if (data.get('schema') != 'singularitydog.user-selected-recordings.v1'
            or data.get('authorization') != RECORDING_AUTHORIZATION):
        raise ValueError('Wrong user-selected recording authorization')
    files = data.get('files')
    if not isinstance(files, dict) or not files:
        raise ValueError('A nonempty explicit recording registry is required')
    for name, entry in files.items():
        local = member(root, name)
        if (Path(name).parent != Path('docs/media') or name != Path(name).as_posix()
                or '\\' in name or local.suffix not in {'.gif', '.mp4', '.png'}):
            raise ValueError('Selected recordings must be directly inside docs/media')
        if (not isinstance(entry, dict) or entry.get('reviewed_for_publication') is not True
                or entry.get('kind') not in {'user_selected_original', 'temporal_preview'}):
            raise ValueError('Recording lacks individual publication approval')
        if (not isinstance(entry.get('sha256'), str)
                or re.fullmatch(r'[0-9a-f]{64}', entry['sha256']) is None
                or type(entry.get('bytes')) is not int or entry['bytes'] <= 0):
            raise ValueError('Recording requires an exact SHA and positive byte count')
        if entry['kind'] == 'user_selected_original' and entry.get('source_sha256') != entry['sha256']:
            raise ValueError('Selected original source SHA mismatch')
    for entry in files.values():
        if entry['kind'] != 'temporal_preview':
            continue
        source_name = entry.get('source_path')
        if not isinstance(source_name, str) or source_name not in files:
            raise ValueError('Preview source must be in the same recording registry')
        source = files[source_name]
        if (source['kind'] != 'user_selected_original' or Path(source_name).suffix != '.mp4'
                or entry.get('source_sha256') != source['sha256']):
            raise ValueError('Preview source must match a selected original MP4')
        if not _media_payload_matches(member(root, source_name), source):
            raise ValueError('Preview original MP4 changed after approval')
    return files


def approved_print_files(root):
    """Validate the individually selected print files and every archive member."""
    registry = member(root, 'evidence/print-publication.json')
    if not registry.exists():
        return {}
    data = json.loads(registry.read_text(), object_pairs_hook=_unique_object)
    if (data.get('schema') != 'singularitydog.user-selected-print-files.v1'
            or data.get('authorization') != PRINT_AUTHORIZATION):
        raise ValueError('Wrong print publication authorization')
    files = data.get('files')
    if not isinstance(files, dict) or not files:
        raise ValueError('Explicit print file registry required')
    for name, entry in files.items():
        path = member(root, name)
        if (name != Path(name).as_posix() or '\\' in name
                or not any(Path(name).is_relative_to(directory) for directory in PRINT_DIRECTORIES)
                or path.suffix not in {'.stl', '.png', '.csv', '.md', '.json', '.zip'}):
            raise ValueError('Print file outside selected package scope')
        if (not isinstance(entry, dict) or entry.get('reviewed_for_publication') is not True
                or re.fullmatch(r'[0-9a-f]{64}', str(entry.get('sha256'))) is None
                or type(entry.get('bytes')) is not int or entry['bytes'] <= 0):
            raise ValueError('Print file requires individual approval and hash')
        payload = path.read_bytes()
        if len(payload) != entry['bytes'] or hashlib.sha256(payload).hexdigest() != entry['sha256']:
            raise ValueError('Print file changed after review: ' + name)
        # Archive-only publication must not bypass the text checks on its members.
        if path.suffix in TEXT_SUFFIXES:
            text = payload.decode('utf-8')
            if '\x00' in text or any(pattern.search(text) for pattern in PATTERNS.values()):
                raise ValueError('Private or binary content in print document: ' + name)
        if path.suffix == '.png' and not payload.startswith(b'\x89PNG\r\n\x1a\n'):
            raise ValueError('Print preview is not PNG')
        if path.suffix != '.stl':
            continue
        if (entry.get('units') != 'mm' or entry.get('strict_binary_stl') is not True
                or entry.get('source_sha256') != entry['sha256'] or len(payload) < 84):
            raise ValueError('Print STL requires unchanged source and mm review')
        count = struct.unpack_from('<I', payload, 80)[0]
        if count <= 0 or count != entry.get('triangles') or len(payload) != 84 + 50 * count:
            raise ValueError('Invalid binary STL triangle length')
        lo, hi = [math.inf] * 3, [-math.inf] * 3
        for face in struct.iter_unpack('<12fH', payload[84:]):
            if not all(math.isfinite(v) for v in face[:12]):
                raise ValueError('Nonfinite STL geometry')
            for offset in (3, 6, 9):
                for axis in range(3):
                    v = face[offset + axis]
                    lo[axis], hi[axis] = min(lo[axis], v), max(hi[axis], v)
        for field, actual in [('bbox_min_mm', lo), ('bbox_max_mm', hi),
                              ('dimensions_mm', [b-a for a,b in zip(lo,hi)])]:
            expected = entry.get(field)
            if (not isinstance(expected, list) or len(expected) != 3
                    or any(not isinstance(x, (int, float)) or not math.isfinite(x)
                           or abs(x-y) > 0.0001 for x,y in zip(expected, actual))):
                raise ValueError('STL dimension mismatch: ' + name)
    for name, entry in files.items():
        if Path(name).suffix != '.zip':
            continue
        members = entry.get('members')
        if not isinstance(members, dict) or not members:
            raise ValueError('Print archive requires exact members')
        with zipfile.ZipFile(member(root, name)) as archive:
            names = archive.namelist()
            if len(set(names)) != len(names) or set(names) != set(members):
                raise ValueError('Unexpected or duplicate print archive member')
            for item in archive.infolist():
                relative = Path(item.filename)
                if (relative.is_absolute() or '..' in relative.parts or '\\' in item.filename
                        or relative.as_posix() != item.filename or item.is_dir()
                        or (item.external_attr >> 16) & 0o170000 == 0o120000):
                    raise ValueError('Unsafe print archive member')
                expected = members[item.filename]
                public = expected.get('public_path')
                if public not in files or Path(public).suffix == '.zip':
                    raise ValueError('Archive member must be a selected public file')
                original = files[public]
                if (item.file_size != original['bytes']
                        or expected.get('bytes') != original['bytes']
                        or expected.get('sha256') != original['sha256']):
                    raise ValueError('Archive member receipt mismatch')
                payload = archive.read(item)
                if hashlib.sha256(payload).hexdigest() != original['sha256']:
                    raise ValueError('Archive member changed after review')
    return files


def check(root, names):
    failures = []
    media = approved_media(root)
    recordings = approved_recordings(root)
    print_files = approved_print_files(root)
    if set(media) & set(recordings):
        raise ValueError('Duplicate media approval across registries')
    media = {**media, **recordings}
    if set(media) & set(print_files):
        raise ValueError('Duplicate print and media approval')
    if len(set(names)) != len(names):
        raise ValueError('Duplicate publication file name')
    for name in names:
        path = member(root, name)
        if name in media:
            if not _media_payload_matches(path, media[name]):
                failures.append({'file': name, 'reason': 'reviewed media type/bytes/hash mismatch'})
            continue
        if name in print_files and path.suffix in {'.stl', '.png', '.zip'}:
            continue
        if (path.suffix not in TEXT_SUFFIXES and path.name != '.gitignore') or set(Path(name).parts) & BLOCKED_PARTS:
            failures.append({'file': name, 'reason': 'excluded asset or file type'})
            continue
        try:
            data = path.read_text(encoding='utf-8')
        except (UnicodeError, OSError):
            failures.append({'file': name, 'reason': 'not readable UTF-8'})
            continue
        if '\x00' in data:
            failures.append({'file': name, 'reason': 'binary content'})
        for label, pattern in PATTERNS.items():
            if pattern.search(data):
                failures.append({'file': name, 'reason': label})
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--files', type=Path, required=True)
    args = parser.parse_args()
    failures = check(args.root, load_names(args.files))
    print(json.dumps({'status': 'REVIEW_REQUIRED' if failures else 'PUBLICATION_SCOPE_CHECK_PASS', 'findings': failures,
                      'scope': 'Text patterns plus exact reviewed-media hashes. Original authorship requires content review.'}, indent=2))
    raise SystemExit(1 if failures else 0)


if __name__ == '__main__':
    main()
