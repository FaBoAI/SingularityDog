#!/usr/bin/env python3
"""Check an explicit publication list for binary/vendor assets and local connection data."""
import argparse
import json
from pathlib import Path
import re
from artifact_manifest import load_names, member, digest

TEXT_SUFFIXES = {'.md', '.py', '.json', '.yaml', '.yml', '.toml', '.txt', '.html', '.urdf'}
BLOCKED_PARTS = {'.git', 'vendor', 'meshes', 'weights', 'checkpoints', 'raw_logs'}
RECORDING_AUTHORIZATION = 'user_selected_six_history_assets_and_l05_l06_l07_l08_l09_l10_l11_results_2026-09-10'
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


def check(root, names):
    failures = []
    media = approved_media(root)
    recordings = approved_recordings(root)
    if set(media) & set(recordings):
        raise ValueError('Duplicate media approval across registries')
    media = {**media, **recordings}
    if len(set(names)) != len(names):
        raise ValueError('Duplicate publication file name')
    for name in names:
        path = member(root, name)
        if name in media:
            if not _media_payload_matches(path, media[name]):
                failures.append({'file': name, 'reason': 'reviewed media type/bytes/hash mismatch'})
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
