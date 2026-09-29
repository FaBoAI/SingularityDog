#!/usr/bin/env python3
"""Read-only verification of a historical Jetson source manifest; never imports it."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath


SCHEMA = "singularitydog.tested-jetson-source-snapshot.v1"
DEFAULT_MANIFEST = "runtime/tested_sources/20260929-r64/manifest.json"


def _relative(value: object) -> PurePosixPath:
    if not isinstance(value, str) or not value or "\\" in value:
        raise ValueError("Path must be a nonempty relative POSIX path")
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != value:
        raise ValueError(f"Noncanonical or escaping path: {value!r}")
    return path


def _file(root: Path, relative: object) -> Path:
    path = root
    for part in _relative(relative).parts:
        path = path / part
        if path.is_symlink():
            raise ValueError(f"Symlink is not allowed: {relative}")
    if not path.is_file():
        raise ValueError(f"Missing source file: {relative}")
    return path


def verify(root: Path, manifest_path: str = DEFAULT_MANIFEST) -> dict:
    root = root.resolve(strict=True)
    manifest = json.loads(_file(root, manifest_path).read_text(encoding="utf-8"))
    if (manifest.get("schema") != SCHEMA
            or manifest.get("historical_source_only") is not True
            or manifest.get("live_output_authorized") is not False):
        raise ValueError("Not an explicitly historical, non-authorizing source manifest")
    snapshot = _relative(manifest.get("snapshot"))
    if len(snapshot.parts) != 1:
        raise ValueError("Snapshot name must be one path component")
    entries = manifest.get("sources")
    if not isinstance(entries, list) or not entries:
        raise ValueError("Source list is empty or invalid")
    seen = set()
    archived = 0
    for entry in entries:
        source = _relative(entry["source_path"])
        if source.suffix not in (".py", ".cpp") or str(source) in seen:
            raise ValueError("Unexpected source type or duplicate source path")
        seen.add(str(source))
        storage = str(_relative(entry["storage_path"]))
        current = f"runtime/{source}"
        historical = f"runtime/tested_sources/{snapshot}/{source}"
        if storage not in (current, historical):
            raise ValueError("Source storage does not match its declared source path")
        archived += storage == historical
        data = _file(root, storage).read_bytes()
        if len(data) != entry.get("size_bytes"):
            raise ValueError(f"Size mismatch: {storage}")
        if hashlib.sha256(data).hexdigest() != entry.get("sha256"):
            raise ValueError(f"SHA-256 mismatch: {storage}")
    if len(entries) != manifest.get("source_count") or archived != manifest.get("archived_source_count"):
        raise ValueError("Manifest source counts do not match")
    return {"status": "VERIFIED_HISTORICAL_SOURCE", "sources": len(entries),
            "archived_sources": archived, "live_output_authorized": False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--manifest", default=DEFAULT_MANIFEST)
    args = parser.parse_args()
    try:
        result = verify(args.repo_root, args.manifest)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(json.dumps({"status": "INVALID_SOURCE_SNAPSHOT", "error": str(exc),
                          "live_output_authorized": False}))
        return 1
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
