"""Compare the pure C++ feedback codec against saved, original wire records.

This tool only opens files and a verified shared library. It creates no active
session, sends no command, and does not qualify a live controller.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import struct

from singularitydog_hw import native_active_transport as native
from singularitydog_hw import policy_output_runtime as runtime

FIELDS = ('start_ns', 'finish_ns', 'read_start_ns', 'received_ns', 'deadline_ns',
          'written', 'received')


def owned_records(rows):
    if type(rows) is not list or not 1 <= len(rows) <= 12:
        raise ValueError('A bounded nonempty saved record batch is required')
    result = (native.Record * len(rows))()
    for record, row in zip(result, rows):
        if type(row) is not dict:
            raise ValueError('A saved record object is required')
        for name in FIELDS:
            value = row.get(name)
            maximum = 2**32 if name in ('written', 'received') else 2**64
            if type(value) is not int or not 0 <= value < maximum:
                raise ValueError(f'Unsigned original record field required: {name}')
            setattr(record, name, value)
        for field in ('tx', 'rx'):
            text = row.get(field + '_hex')
            if (type(text) is not str or len(text) != 34 or
                    any(c not in '0123456789abcdef' for c in text)):
                raise ValueError('An exact original 17-byte hex frame is required')
            getattr(record, field)[:] = bytes.fromhex(text)
    return result


def verify(saved, decoder):
    if type(saved) is not list or not 1 <= len(saved) <= 10000:
        raise ValueError('A bounded saved cycle list is required')
    counts = {'cycles': len(saved), 'native_batches': 0, 'native_feedback_rows': 0,
              'legacy_batches': 0, 'original_records': 0}
    original_digest = hashlib.sha256()
    for cycle in saved:
        if type(cycle) is not dict:
            raise ValueError('A saved cycle object is required')
        for phase in ('acquired', 'voltage', 'output'):
            buses = cycle.get(phase)
            if type(buses) is not dict or set(buses) != {'front', 'rear'}:
                raise ValueError('Original two-bus acquired/voltage/output records required')
            for bus, first in (('front', 1), ('rear', 7)):
                batch = buses[bus]
                if type(batch) is not dict:
                    raise ValueError('An original bus record object is required')
                records = owned_records(batch.get('records'))
                raw = bytes(records)
                original_digest.update(raw)
                expected = runtime.decode_records((records, None))
                actual = decoder.decode(records, first)
                counts['original_records'] += len(records)
                if actual is None:
                    counts['legacy_batches'] += 1
                    # Existing parameter/version/identity decoding remains
                    # authoritative. A valid full feedback batch must use C++.
                    if len(records) == 6 and all(key[1] == 'feedback' for key in expected):
                        raise AssertionError('Eligible saved feedback batch was not decoded natively')
                else:
                    if actual != expected or list(actual) != list(expected):
                        raise AssertionError('Saved feedback objects, timestamps or order differ')
                    for key in expected:
                        for field in ('protocol_position_rad', 'velocity_rad_s', 'torque_nm', 'temperature_c'):
                            if struct.pack('>d', getattr(actual[key][0], field)) != struct.pack('>d', getattr(expected[key][0], field)):
                                raise AssertionError('Saved feedback floating point bits differ')
                    counts['native_batches'] += 1
                    counts['native_feedback_rows'] += len(actual)
                if bytes(records) != raw:
                    raise AssertionError('A saved record was modified by the codec')
    return {'schema': 'singularitydog.saved-feedback-decode-parity.v1', 'status': 'PASS',
            **counts, 'original_record_buffers_sha256': original_digest.hexdigest(),
            'object_order_timestamp_and_double_bits_equal': True,
            'original_record_buffers_unchanged': True,
            'hardware_opened': False, 'robot_commands_sent': False,
            'runtime_qualification': False, 'timing_measured': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records', type=Path, required=True)
    parser.add_argument('--records-sha256', required=True)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('A fresh output file is required')
    if not 0 < args.records.stat().st_size <= 64 * 1024 * 1024:
        parser.error('Saved records must fit the 64 MiB file limit')
    raw = args.records.read_bytes()
    if hashlib.sha256(raw).hexdigest() != args.records_sha256:
        parser.error('Saved records differ from their SHA256 pin')
    library = native.load_library(args.library)
    decoder = native.NativeFeedbackBatchDecoder(library)
    if not decoder.available:
        parser.error('The selected library does not have the feedback batch ABI')
    result = verify(json.loads(raw), decoder)
    if hashlib.sha256(args.records.read_bytes()).hexdigest() != args.records_sha256:
        raise ValueError('Saved records changed during verification')
    result['records_sha256'] = args.records_sha256
    result['library_sha256'] = hashlib.sha256(args.library.read_bytes()).hexdigest()
    result['source_sha256'] = {name: hashlib.sha256(Path(path).read_bytes()).hexdigest()
                             for name, path in (('transport.cpp', args.library.parent / 'transport.cpp'),
                                                ('native_active_transport.py', native.__file__),
                                                ('policy_output_runtime.py', runtime.__file__),
                                                ('verify_native_feedback_decode.py', __file__))}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
