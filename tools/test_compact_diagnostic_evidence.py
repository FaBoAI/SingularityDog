"""Saved-data integrity and publication boundaries for the compact copier."""
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import stat
import tempfile
import unittest
from unittest import mock

import compact_diagnostic_evidence as tool


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


class CompactEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name).resolve()
        self.report_path = self.base/'original-report.json'
        self.records_path = self.base/'original-records.json'
        self.output = self.base/'fresh-copy'
        # Generic fixtures contain no real host, private artifact path or UID.
        self.records = [dict(cycle=i, original_time=i*1_000_000,
            saved_wire='aabb00', values=[-0.0, 0.0, 1, True, None,
                math.nextafter(1.0, math.inf), '静止'],
            nested=dict(second=2, first=1)) for i in range(1, 502)]
        self.report = dict(status='COMPLETE_DIAGNOSTIC', errors=[],
            cycles_requested=501, cycles_completed=501,
            measurements=[dict(cadence_slot=i, elapsed_ms=19.5) for i in range(501)],
            v3_voltage_fast_pipeline=dict(enabled=True, records_sha256=None),
            never_open_reference={'path':'missing-reference.json', 'sha256':'e'*64})
        self.save()

    def save(self):
        self.records_raw = (json.dumps(self.records, indent=2, ensure_ascii=True)+'\n').encode()
        self.records_pin = digest(self.records_raw)
        self.report['v3_voltage_fast_pipeline']['records_sha256'] = self.records_pin
        self.report_raw = (json.dumps(self.report, indent=2, ensure_ascii=True)+'\n').encode()
        self.report_pin = digest(self.report_raw)
        self.records_path.write_bytes(self.records_raw)
        self.report_path.write_bytes(self.report_raw)

    def run_compact(self, **changes):
        arguments = dict(report_path=self.report_path, report_sha256=self.report_pin,
            records_path=self.records_path, records_sha256=self.records_pin, output=self.output)
        arguments.update(changes)
        return tool.compact(**arguments)

    def assert_rejected(self, **changes):
        with self.assertRaises((ValueError, OSError, TypeError)):
            self.run_compact(**changes)
        self.assertFalse(self.output.exists())

    def test_lossless_copy_preserves_values_and_records_binding(self):
        receipt = self.run_compact()
        records_raw = (self.output/'records.json').read_bytes()
        report_raw = (self.output/'report.json').read_bytes()
        records, report = json.loads(records_raw), json.loads(report_raw)
        # Independent checks of the important representation distinctions.
        self.assertEqual(list(records[0]['nested']), ['second', 'first'])
        self.assertEqual(records[0]['values'][0].hex(), '-0x0.0p+0')
        self.assertEqual(records[0]['values'][1].hex(), '0x0.0p+0')
        self.assertIs(type(records[0]['values'][2]), int)
        self.assertIs(type(records[0]['values'][3]), bool)
        self.assertEqual(records[0]['values'][5].hex(), self.records[0]['values'][5].hex())
        self.assertEqual([r['cycle'] for r in records], list(range(1, 502)))
        self.assertEqual(records, self.records)
        self.assertLess(len(records_raw), len(self.records_raw))
        self.assertLessEqual(len(records_raw), 16*1024*1024)
        self.assertEqual(report['v3_voltage_fast_pipeline']['records_sha256'], digest(records_raw))
        provenance = report.pop(tool.PROVENANCE_KEY)
        self.assertEqual(provenance['original_report'], {'path':str(self.report_path), 'sha256':self.report_pin})
        self.assertFalse(provenance['motor_output_allowed'])
        report['v3_voltage_fast_pipeline']['records_sha256'] = self.records_pin
        self.assertEqual(list(report), list(self.report))
        self.assertEqual(report, self.report)
        self.assertEqual(receipt['record_count'], 501)
        self.assertEqual(receipt['tool_source_sha256'], digest(Path(tool.__file__).read_bytes()))
        self.assertEqual(receipt['derived']['records.json']['sha256'], digest(records_raw))
        self.assertEqual(receipt['derived']['report.json']['sha256'], digest(report_raw))
        for field in ('measurement_rerun', 'hardware_opened', 'output_allowed',
                      'approval_granted', 'original_reference_files_opened'):
            self.assertIs(receipt[field], False)
        self.assertEqual(self.records_path.read_bytes(), self.records_raw)
        self.assertEqual(self.report_path.read_bytes(), self.report_raw)
        self.assertEqual(stat.S_IMODE(self.output.stat().st_mode), 0o700)
        for name in ('records.json', 'report.json', 'receipt.json'):
            self.assertEqual(stat.S_IMODE((self.output/name).stat().st_mode), 0o600)
        self.assertEqual(json.loads((self.output/'receipt.json').read_bytes()), receipt)

    def test_both_enabled_pipeline_bindings_are_rebound(self):
        self.report['v3_voltage_pipeline'] = {'enabled':True, 'records_sha256':self.records_pin}
        self.save()
        self.run_compact()
        report = json.loads((self.output/'report.json').read_bytes())
        expected = digest((self.output/'records.json').read_bytes())
        for key in tool.BINDING_KEYS:
            self.assertEqual(report[key]['records_sha256'], expected)

    def test_source_sha_is_mandatory_and_exact(self):
        for change in ({'report_sha256':'0'*64}, {'records_sha256':'0'*64},
                       {'report_sha256':True}, {'report_sha256':'A'*64},
                       {'records_sha256':'abc'}):
            with self.subTest(change=change):
                self.assert_rejected(**change)

    def test_bad_pin_rejected_before_source_open(self):
        with mock.patch.object(tool.os, 'open') as opened:
            with self.assertRaises(ValueError):
                tool._read(self.records_path, 'not-a-sha')
        opened.assert_not_called()

    def test_oversize_sparse_original_rejected_before_read(self):
        self.assertEqual(tool.ORIGINAL_CAP, 64*1024*1024)
        sparse = self.base/'oversize-original.json'
        with sparse.open('wb') as handle:
            handle.truncate(tool.ORIGINAL_CAP+1)
        with mock.patch.object(tool.os, 'open') as opened:
            with self.assertRaisesRegex(ValueError, '64MiB cap'):
                tool._read(sparse, '0'*64)
        opened.assert_not_called()

    def test_exact_original_cap_boundary_is_accepted(self):
        with mock.patch.object(tool, 'ORIGINAL_CAP', len(self.records_raw)):
            self.assertEqual(tool._read(self.records_path, self.records_pin), self.records_raw)

    def test_original_growth_after_fstat_is_bounded_and_rejected(self):
        small = self.base/'growing-original.json'
        small.write_bytes(b'{}')
        original_fdopen = tool.os.fdopen
        original_cap = 8
        read_lengths = []

        class GrowingReader:
            def __init__(self, handle):
                self.handle = handle

            def __enter__(self):
                return self

            def __exit__(self, *args):
                self.handle.close()

            def fileno(self):
                return self.handle.fileno()

            def read(self, length):
                read_lengths.append(length)
                # Growth after both size checks but before the actual read.
                with small.open('ab') as handle:
                    handle.write(b' '*original_cap)
                return self.handle.read(length)

        with mock.patch.object(tool, 'ORIGINAL_CAP', original_cap), \
             mock.patch.object(tool.os, 'fdopen', side_effect=lambda fd, mode:GrowingReader(original_fdopen(fd, mode))):
            with self.assertRaisesRegex(ValueError, 'grew beyond'):
                tool._read(small, digest(b'{}'))
        self.assertEqual(read_lengths, [original_cap+1])

    def test_missing_incomplete_or_mistyped_501_report_rejected(self):
        original = copy.deepcopy(self.report)
        changes = [('status', 'ABORTED'), ('errors', ['failure']),
            ('cycles_completed', 500), ('cycles_completed', True),
            ('cycles_completed', 501.0), ('cycles_requested', 502),
            ('measurements', self.report['measurements'][:-1]),
            ('measurements', [None]*501)]
        for key, value in changes:
            with self.subTest(field=key, value=str(value)[:40]):
                self.report = copy.deepcopy(original)
                self.report[key] = value
                self.save()
                self.assert_rejected()
        self.report = copy.deepcopy(original)
        del self.report['cycles_requested']
        self.save()
        self.assert_rejected()

    def test_record_count_cycle_type_and_original_order_rejected(self):
        original = copy.deepcopy(self.records)
        for kind in ('missing', 'extra', 'reordered', 'bool', 'float', 'missing-cycle'):
            with self.subTest(kind=kind):
                self.records = copy.deepcopy(original)
                if kind == 'missing':
                    self.records.pop()
                elif kind == 'extra':
                    self.records.append({'cycle':502})
                elif kind == 'reordered':
                    self.records[0], self.records[1] = self.records[1], self.records[0]
                elif kind == 'bool':
                    self.records[0]['cycle'] = True
                elif kind == 'float':
                    self.records[0]['cycle'] = 1.0
                else:
                    del self.records[0]['cycle']
                self.save()
                self.assert_rejected()

    def test_binding_missing_disabled_mismatched_and_unknown_rejected(self):
        original = copy.deepcopy(self.report)
        for kind in ('missing', 'disabled', 'truthy', 'mismatch', 'unknown'):
            with self.subTest(kind=kind):
                self.report = copy.deepcopy(original)
                self.save()
                if kind == 'missing':
                    del self.report['v3_voltage_fast_pipeline']
                elif kind in ('disabled', 'truthy'):
                    self.report['v3_voltage_fast_pipeline']['enabled'] = False if kind == 'disabled' else 1
                elif kind == 'mismatch':
                    self.report['v3_voltage_fast_pipeline']['records_sha256'] = '0'*64
                else:
                    self.report['other_binding'] = {'records_sha256':self.records_pin}
                raw = json.dumps(self.report).encode()
                self.report_path.write_bytes(raw)
                self.report_pin = digest(raw)
                self.assert_rejected()

    def test_duplicate_keys_nonfinite_and_overflow_json_rejected(self):
        for raw in (b'{"x":1,"x":2}', b'{"x":NaN}', b'{"x":Infinity}',
                    b'{"x":-Infinity}', b'{"x":1e400}', b'{"x":-1e400}'):
            with self.subTest(raw=raw):
                self.report_path.write_bytes(raw)
                self.assert_rejected(report_sha256=digest(raw))
        # Duplicate keys deeply inside otherwise valid records must also fail.
        self.report_path.write_bytes(self.report_raw)
        raw = self.records_raw.replace(b'"cycle": 1,', b'"cycle": 1, "cycle": 1,', 1)
        self.records_path.write_bytes(raw)
        self.assert_rejected(records_sha256=digest(raw))
        # The same strict finite rule is enforced inside records, not just report.
        raw = self.records_raw.replace(b'-0.0', b'1e400', 1)
        self.records_path.write_bytes(raw)
        self.assert_rejected(records_sha256=digest(raw))

    def test_already_serialized_report_cannot_be_relabelled_original(self):
        self.report[tool.PROVENANCE_KEY] = {'measurement_rerun':False}
        self.save()
        self.assert_rejected()

    def test_exact_comparison_detects_signedzero_types_and_order(self):
        for before, after in ((-0.0, 0.0), (1, 1.0), (True, 1),
                ({'a':1, 'b':2}, {'b':2, 'a':1}), ([1, 2], [2, 1]),
                (float('nan'), float('nan'))):
            with self.subTest(before=before, after=after):
                with self.assertRaises(ValueError):
                    tool.same_values(before, after)

    def test_cap_boundary_and_overflow_are_exact_and_not_truncated(self):
        self.assertEqual(tool.CAP, 16*1024*1024)
        raw = tool._encode({'x':'bounded'})
        with mock.patch.object(tool, 'CAP', len(raw)):
            self.assertEqual(tool._encode({'x':'bounded'}), raw)
        with mock.patch.object(tool, 'CAP', len(raw)-1):
            with self.assertRaisesRegex(ValueError, 'cap'):
                tool._encode({'x':'bounded'})
        with mock.patch.object(tool, 'CAP', 1000):
            self.assert_rejected()
        self.assertEqual(self.records_path.read_bytes(), self.records_raw)

    def test_existing_output_cannot_be_overwritten(self):
        self.output.mkdir()
        marker = self.output/'untouched'
        marker.write_bytes(b'original')
        with self.assertRaises(ValueError):
            self.run_compact()
        self.assertEqual(marker.read_bytes(), b'original')
        self.assertEqual(list(self.output.iterdir()), [marker])

    def test_symlink_source_output_and_ancestor_prohibited(self):
        source_link = self.base/'source-link'
        source_link.symlink_to(self.records_path)
        self.assert_rejected(records_path=source_link)
        output_link = self.base/'output-link'
        output_link.symlink_to(self.base/'missing-target')
        self.assert_rejected(output=output_link)
        ancestor = self.base/'ancestor-link'
        ancestor.symlink_to(self.base, target_is_directory=True)
        self.assert_rejected(output=ancestor/'fresh-child')

    def test_output_must_be_absolute_outside_git_and_existing_parent(self):
        self.assert_rejected(output=Path('relative-output'))
        self.assert_rejected(report_path=Path('relative-report'))
        self.assert_rejected(output=self.base/'missing-parent'/'fresh-copy')
        for marker_kind in ('directory', 'file'):
            checkout = self.base/('checkout-'+marker_kind)
            checkout.mkdir()
            marker = checkout/'.git'
            marker.mkdir() if marker_kind == 'directory' else marker.write_text('gitdir: saved')
            self.assert_rejected(output=checkout/'fresh-copy')

    def test_nonregular_sources_and_same_file_rejected(self):
        self.assert_rejected(records_path=self.base)
        self.assert_rejected(records_path=self.report_path, records_sha256=self.report_pin)

    def test_source_mutation_before_publication_leaves_no_success_copy(self):
        original_read = tool._read
        calls = 0

        def change_on_last_reaudit(path, pin):
            nonlocal calls
            calls += 1
            if calls == 5:
                self.report_path.write_bytes(self.report_raw+b' ')
            return original_read(path, pin)

        with mock.patch.object(tool, '_read', side_effect=change_on_last_reaudit):
            self.assert_rejected()
        self.assertEqual(calls, 5)
        self.assertEqual(self.records_path.read_bytes(), self.records_raw)

    def test_tool_source_mutation_before_receipt_fails_closed(self):
        original_read_bytes = Path.read_bytes
        source_reads = 0
        source_path = Path(tool.__file__)

        def changed_source(path):
            nonlocal source_reads
            raw = original_read_bytes(path)
            if path == source_path:
                source_reads += 1
                if source_reads == 3:
                    return raw+b'changed'
            return raw

        with mock.patch.object(Path, 'read_bytes', changed_source):
            self.assert_rejected()
        self.assertEqual(source_reads, 3)
        self.assertEqual(self.report_path.read_bytes(), self.report_raw)

    def test_report_cap_failure_does_not_publish_records_alone(self):
        # Records fit; only the report exceeds the same strict cap.
        self.report['saved_text'] = 'x'*200_000
        self.save()
        records_size = len(tool._encode(self.records))
        with mock.patch.object(tool, 'CAP', records_size+1):
            self.assert_rejected()

    def test_save_failure_cleans_only_owned_files(self):
        original_open = tool.os.open

        def fail_receipt(path, *args, **kwargs):
            if path == 'receipt.json':
                raise OSError('injected storage failure')
            return original_open(path, *args, **kwargs)

        with mock.patch.object(tool.os, 'open', side_effect=fail_receipt):
            self.assert_rejected()
        self.assertEqual(self.report_path.read_bytes(), self.report_raw)
        self.assertEqual(self.records_path.read_bytes(), self.records_raw)

    def test_cli_failure_is_explicit_and_success_does_not_grant_output(self):
        arguments = ['--report', str(self.report_path), '--report-sha256', self.report_pin,
            '--records', str(self.records_path), '--records-sha256', self.records_pin,
            '--output', str(self.output)]
        with mock.patch.object(tool.sys, 'stdout', new_callable=io.StringIO) as stdout:
            self.assertEqual(tool.main(arguments), 0)
        self.assertIs(json.loads(stdout.getvalue())['output_allowed'], False)
        failed_args = arguments[:-1]+[str(self.base/'never-created')]
        failed_args[3] = '0'*64
        with mock.patch.object(tool.sys, 'stderr', new_callable=io.StringIO) as stderr:
            self.assertEqual(tool.main(failed_args), 1)
        self.assertIn('File-only compaction failed', stderr.getvalue())
        self.assertFalse((self.base/'never-created').exists())

    def test_cli_abbreviations_are_not_accepted(self):
        arguments = ['--report', str(self.report_path), '--report-sha256', self.report_pin,
            '--records', str(self.records_path), '--records-sha256', self.records_pin,
            '--out', str(self.output)]
        with mock.patch.object(tool.sys, 'stderr', new_callable=io.StringIO):
            with self.assertRaises(SystemExit) as error:
                tool.main(arguments)
        self.assertEqual(error.exception.code, 2)
        self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()
