#!/usr/bin/env python3
"""Check an explicit publication list for binary/vendor assets and local connection data."""
import argparse
import json
from pathlib import Path
import re
from artifact_manifest import load_names, member

TEXT_SUFFIXES = {'.md', '.py', '.json', '.yaml', '.yml', '.toml', '.txt', '.html', '.urdf'}
BLOCKED_PARTS = {'.git', 'vendor', 'meshes', 'weights', 'checkpoints', 'raw_logs'}
PATTERNS = {
    'private-network-address': re.compile(r'(?<![\d.])(?:10\.\d{1,3}\.\d{1,3}\.\d{1,3}|192\.168\.\d{1,3}\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3})(?![\d.])'),
    'local-home-path': re.compile(r'/(?:Users|home)/[A-Za-z0-9_.-]+/'),
    'private-key': re.compile(r'-----BEGIN (?:OPENSSH |RSA |EC )?PRIVATE KEY-----'),
    'credential-assignment': re.compile(r'(?i)[\"\']?(?:password|access_token|api_key|secret_key)[\"\']?\s*[:=]\s*[\"\'][^\"\'\n]{3,}[\"\']'),
    'host-login': re.compile(r'\b[A-Za-z0-9_.-]+@(?:\d{1,3}\.){3}\d{1,3}\b'),
}


def check(root, names):
    failures = []
    for name in names:
        path = member(root, name)
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
    print(json.dumps({'status': 'REVIEW_REQUIRED' if failures else 'TEXT_SCOPE_CHECK_PASS', 'findings': failures,
                      'scope': 'Pattern check only. Original authorship and publication scope require content review.'}, indent=2))
    raise SystemExit(1 if failures else 0)


if __name__ == '__main__':
    main()
