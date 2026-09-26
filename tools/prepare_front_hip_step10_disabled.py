"""Build a front-hip 10-degree candidate and disabled package from local files.

This command never opens a device or creates an active package. A same-boot
hold summary and frozen disabled source are required by the existing builders.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re

import build_role_group_step as builder


def prepare(source: Path, hold: Path, output_root: Path, tag: str) -> dict:
    if re.fullmatch(r'[a-z0-9][a-z0-9-]{0,47}', tag) is None:
        raise ValueError('Tag must be 1-48 lowercase letters, digits or hyphens')
    if output_root.is_symlink():
        raise ValueError('Use a fresh output root without a symlink')
    source = source.resolve(strict=True)
    hold = hold.resolve(strict=True)
    output_root = output_root.resolve()
    if not source.is_dir() or not hold.is_file():
        raise ValueError('Frozen source directory and hold summary file are required')
    if output_root.exists() or output_root.is_symlink():
        raise ValueError('Use a fresh output root')
    if output_root.is_relative_to(source):
        raise ValueError('Output root cannot be inside the frozen source')
    prepared = output_root / f'{tag}-prepared'
    disabled = output_root / f'{tag}-disabled'
    candidate = builder.prepare(source, hold, 'front-hip', prepared,
                                'front-hip-mirrored', 10.)
    package = builder.disabled(source, hold, prepared, disabled)
    return {'prepared': str(prepared), 'disabled_package': str(disabled),
            'boot_id': candidate['boot_id'],
            'candidate_sha256': candidate['candidate_sha256'],
            'manifest_sha256': package['manifest_sha256'],
            'disabled_only': True}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-disabled', type=Path, required=True)
    parser.add_argument('--current-hold-summary', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--tag', required=True)
    args = parser.parse_args(argv)
    result = prepare(args.source_disabled, args.current_hold_summary,
                     args.output_root, args.tag)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
