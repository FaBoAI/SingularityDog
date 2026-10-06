#!/usr/bin/env python3
"""Compact pinned saved 501-cycle diagnostic JSON, without new measurement.

Only the two explicitly selected JSON files are read. Referenced CAN devices,
models, reviews and commands are never opened or executed. Copies are distinct
artifacts; original SHA pins and every decoded value remain auditable.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import struct
import sys

CAP = 16*1024*1024
ORIGINAL_CAP = 64*1024*1024
BINDING_KEYS = ('v3_voltage_pipeline', 'v3_voltage_fast_pipeline')
PROVENANCE_KEY = 'lossless_evidence_serialization'


def _need(condition, message):
    if not condition:
        raise ValueError(message)


def _hash(raw):
    return hashlib.sha256(raw).hexdigest()


def _digest(value):
    _need(type(value) is str and len(value) == 64 and
          all(c in '0123456789abcdef' for c in value), 'Explicit SHA256 pin required')
    return value


def _absolute_plain(path, *, must_exist=True):
    path = Path(path)
    _need(path.is_absolute() and '..' not in path.parts, 'Absolute path without parent traversal required')
    for part in reversed((path, *path.parents)):
        _need(not part.is_symlink(), 'Symlink path components prohibited')
    _need(path.exists() if must_exist else not path.exists(),
          'Existing source required' if must_exist else 'Fresh output directory required')
    return path


def _read(path, digest):
    digest = _digest(digest)
    path = _absolute_plain(path)
    info = path.stat()
    _need(stat.S_ISREG(info.st_mode), 'Regular saved JSON file required')
    _need(info.st_size <= ORIGINAL_CAP, 'Original JSON exceeds 64MiB cap')
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    with os.fdopen(os.open(path, flags), 'rb') as handle:
        actual = os.fstat(handle.fileno())
        _need(stat.S_ISREG(actual.st_mode) and (actual.st_dev, actual.st_ino) ==
              (info.st_dev, info.st_ino), 'Source identity changed before reading')
        _need(actual.st_size <= ORIGINAL_CAP, 'Original JSON exceeds 64MiB cap')
        raw = handle.read(ORIGINAL_CAP+1)
        _need(len(raw) <= ORIGINAL_CAP, 'Original JSON grew beyond 64MiB cap')
    _need(_hash(raw) == digest, 'Source SHA256 differs')
    return raw


def _pairs(items):
    result = {}
    for key, value in items:
        _need(key not in result, 'Duplicate JSON key prohibited')
        result[key] = value
    return result


def _float(token):
    value = float(token)
    _need(math.isfinite(value), 'Nonfinite JSON number prohibited')
    return value


def parse(raw):
    return json.loads(raw, object_pairs_hook=_pairs, parse_float=_float,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON number prohibited')))


def same_values(before, after):
    """Check types, insertion/list order and IEEE-754 bits including -0.0."""
    counts = dict(nodes=0, float64_values=0, dictionaries=0, lists=0)

    def visit(a, b):
        counts['nodes'] += 1
        _need(type(a) is type(b), 'Decoded JSON type changed')
        if type(a) is dict:
            counts['dictionaries'] += 1
            _need(list(a) == list(b), 'Dictionary key order changed')
            for key in a:
                visit(a[key], b[key])
        elif type(a) is list:
            counts['lists'] += 1
            _need(len(a) == len(b), 'Array length changed')
            for x, y in zip(a, b):
                visit(x, y)
        elif type(a) is float:
            counts['float64_values'] += 1
            _need(math.isfinite(a) and math.isfinite(b) and
                  struct.pack('>d', a) == struct.pack('>d', b), 'Float64 bits changed')
        else:
            _need(type(a) in (str, int, bool, type(None)) and a == b, 'Decoded JSON value changed')
    visit(before, after)
    return counts


def _encode(value):
    raw = (json.dumps(value, separators=(',', ':'), ensure_ascii=False, allow_nan=False)+'\n').encode()
    _need(len(raw) <= CAP, 'Compact JSON exceeds unchanged 16MiB cap')
    return raw


def _bindings(report, records_digest):
    keys = []
    for key in BINDING_KEYS:
        value = report.get(key)
        if type(value) is dict and value.get('enabled') is True:
            _need(_digest(value.get('records_sha256')) == records_digest,
                  'Report records SHA256 binding differs')
            keys.append(key)
        elif type(value) is dict and 'records_sha256' in value:
            raise ValueError('Records binding requires an enabled diagnostic pipeline')
    _need(bool(keys), 'Enabled diagnostic records SHA256 binding required')
    for key, value in report.items():
        if type(value) is dict and 'records_sha256' in value:
            _need(key in keys, 'Unsupported records SHA256 binding')
    return keys


def _prepare(report, records, report_path, records_path, report_digest, records_digest):
    _need(type(report) is dict and report.get('status') == 'COMPLETE_DIAGNOSTIC' and
          report.get('errors') == [] and
          type(report.get('cycles_completed')) is int and report['cycles_completed'] == 501 and
          type(report.get('cycles_requested')) is int and report['cycles_requested'] == 501 and
          type(report.get('measurements')) is list and len(report['measurements']) == 501 and
          all(type(row) is dict for row in report['measurements']),
          'Complete 501-cycle diagnostic report required')
    _need(type(records) is list and len(records) == 501 and
          all(type(row) is dict and type(row.get('cycle')) is int and row['cycle'] == index
              for index, row in enumerate(records, 1)),
          'Exactly 501 original records in cycle 1..501 order required')
    _need(PROVENANCE_KEY not in report, 'Original report already has serialization provenance')
    bindings = _bindings(report, records_digest)
    records_raw = _encode(records)
    record_counts = same_values(records, parse(records_raw))
    new_report = copy.deepcopy(report)
    for key in bindings:
        new_report[key]['records_sha256'] = _hash(records_raw)
    new_report[PROVENANCE_KEY] = dict(schema='singularitydog.lossless-diagnostic-serialization.v1',
        original_report=dict(path=str(report_path), sha256=report_digest),
        original_records=dict(path=str(records_path), sha256=records_digest),
        record_objects_unchanged=True, all_wire_bytes_and_timestamps_preserved=True,
        measurements_and_status_unchanged=True, only_whitespace_and_unicode_json_representation_changed=True,
        measurement_rerun=False, motor_output_allowed=False)
    report_raw = _encode(new_report)
    parsed_report = parse(report_raw)
    provenance = parsed_report.pop(PROVENANCE_KEY)
    same_values(new_report[PROVENANCE_KEY], provenance)
    for key in bindings:
        _need(parsed_report[key]['records_sha256'] == _hash(records_raw), 'Derived records binding differs')
        parsed_report[key]['records_sha256'] = records_digest
    report_counts = same_values(report, parsed_report)
    return report_raw, records_raw, bindings, record_counts, report_counts


def compact(report_path, report_sha256, records_path, records_sha256, output):
    report_path = _absolute_plain(report_path)
    records_path = _absolute_plain(records_path)
    output = _absolute_plain(output, must_exist=False)
    _need(report_path != records_path, 'Separate report and records sources required')
    _need(output.parent.is_dir(), 'Existing private output parent directory required')
    for parent in (output.parent, *output.parent.parents):
        marker = parent/'.git'
        _need(not marker.exists() and not marker.is_symlink(), 'Output must be outside a Git checkout')
    report_sha256, records_sha256 = _digest(report_sha256), _digest(records_sha256)
    report_raw, records_raw = _read(report_path, report_sha256), _read(records_path, records_sha256)
    source_raw = Path(__file__).read_bytes()
    tool_digest = _hash(source_raw)
    new_report, new_records, bindings, record_counts, report_counts = _prepare(
        parse(report_raw), parse(records_raw), report_path, records_path, report_sha256, records_sha256)
    _read(report_path, report_sha256); _read(records_path, records_sha256)
    _need(_hash(Path(__file__).read_bytes()) == tool_digest, 'Tool source changed during analysis')
    output.mkdir(mode=0o700)
    directory_fd = os.open(output, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0) | getattr(os, 'O_NOFOLLOW', 0))
    identity = os.fstat(directory_fd)
    created, completed = [], False

    def write(name, raw):
        flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, 'O_NOFOLLOW', 0)
        fd = os.open(name, flags, 0o600, dir_fd=directory_fd)
        created.append(name)
        with os.fdopen(fd, 'wb') as handle:
            handle.write(raw)
        with os.fdopen(os.open(name, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0), dir_fd=directory_fd), 'rb') as handle:
            _need(handle.read() == raw, 'Derived bytes differ after saving')

    try:
        write('records.json', new_records)
        write('report.json', new_report)
        _read(report_path, report_sha256); _read(records_path, records_sha256)
        _need(_hash(Path(__file__).read_bytes()) == tool_digest, 'Tool source changed before publication')
        receipt = dict(schema='singularitydog.compact-diagnostic-evidence-receipt.v1',
            status='LOSSLESS_SERIALIZATION_COMPLETE_NOT_NEW_MEASUREMENT', tool_source_sha256=tool_digest,
            originals={'report.json':dict(path=str(report_path), sha256=report_sha256, bytes=len(report_raw)),
                       'records.json':dict(path=str(records_path), sha256=records_sha256, bytes=len(records_raw))},
            derived={'report.json':dict(sha256=_hash(new_report), bytes=len(new_report)),
                     'records.json':dict(sha256=_hash(new_records), bytes=len(new_records))},
            record_count=501, unchanged_json_bytes_cap=CAP, original_json_bytes_cap=ORIGINAL_CAP,
            report_only_changes=[key+'.records_sha256' for key in bindings]+[PROVENANCE_KEY],
            records_equivalence=record_counts, report_equivalence=report_counts,
            float64_bits_types_and_orders_preserved=True, signed_zero_preserved=True,
            no_trace_wire_timestamp_removed=True, original_files_unchanged=True,
            measurement_rerun=False, hardware_opened=False, output_allowed=False,
            approval_granted=False, original_reference_files_opened=False)
        write('receipt.json', (json.dumps(receipt, indent=2, allow_nan=False)+'\n').encode())
        completed = True
        return receipt
    finally:
        if not completed:
            # Remove only files this invocation created on its owned directory.
            for name in reversed(created):
                try:
                    os.unlink(name, dir_fd=directory_fd)
                except OSError:
                    pass
            try:
                current = output.lstat()
                if (current.st_dev, current.st_ino) == (identity.st_dev, identity.st_ino):
                    output.rmdir()
            except OSError:
                pass
        os.close(directory_fd)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--report-sha256', required=True)
    parser.add_argument('--records', type=Path, required=True)
    parser.add_argument('--records-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        receipt = compact(args.report, args.report_sha256, args.records, args.records_sha256, args.output)
        print(json.dumps(dict(status=receipt['status'], derived=receipt['derived'], output_allowed=False)))
        return 0
    except (ValueError, OSError, TypeError, RecursionError) as error:
        print('File-only compaction failed: '+str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
