"""Synthetic predecessor metadata gates; no model or physical qualification."""
import copy
from pathlib import Path
import tempfile
import unittest

from singularitydog_hw import policy_model_30hz as models
from singularitydog_hw.policy_clock import control_clock_payload
from . import type1_profile as P, test_type1_profile as fixtures


class NativePredecessorTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name).resolve()
        world = fixtures.World(root)
        model = models._plan(models.bundle_artifacts(root/'uninstalled-inline-fixture'),
            {'source_manifest': dict.fromkeys(models._SOURCE_NAMES),
             'export': {'evaluation_all_intervals_completed': False}})
        self.contract = P.build_contract(copy.deepcopy(world.plan), world.lineage_refs(),
            copy.deepcopy(world.geometry_doc), world.geometry, options={'native_group_cycle': True},
            control_clock=control_clock_payload(30), policy30_model=model)
        self.report = fixtures.type1_report(self.contract, 'zero_gain_timing', 2,
                                           1_000_000_000, 3_000_000_000)
        self.report.update(schema=P.REPORT_SCHEMA_30, completed_cycles=55,
            control_clock=self.contract['control_clock'], policy30_model=self.contract['policy30_model'],
            inference_tx_budget_ns=20_000_000, cycle_budget_ns=33_333_334)

    def validate(self, report):
        return P.validate_type1_report(report, contract=self.contract,
            contract_digest=P.contract_sha256(self.contract), mode='zero_gain_timing')

    def selected_report(self):
        result = copy.deepcopy(self.report)
        result.update(backend_kind='genuine_native_group_cpp', measurement={
            'backend_usage': {'kind': 'genuine_native_group_cpp'},
            'native_group_setup': {'settings': [dict(native_tid=i+100, cpu_mask=1 << i,
                configured=1, status=0, timer_slack_ns=1000) for i in range(4)]},
            'native_group_close': {'closed': True, 'restored': True,
                                   'snapshot_complete': True, 'snapshot_error': None}})
        return result

    def test_old_or_mock_backend_cannot_qualify_selected_native_predecessor(self):
        with self.assertRaisesRegex(ValueError, 'actual selected native'):
            self.validate(self.report)
        for field in ('backend_kind', 'measurement'):
            row = self.selected_report()
            if field == 'measurement':
                row[field]['backend_usage']['kind'] = 'injected_file_only_mock'
            else:
                row[field] = 'genuine_subset_active_cpp'
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'actual selected native'):
                self.validate(row)

    def test_unrestored_or_incomplete_native_receipt_is_rejected(self):
        for field, value in (('closed', False), ('restored', False), ('snapshot_complete', False),
                             ('snapshot_error', 'raw conflict')):
            row = self.selected_report()
            row['measurement']['native_group_close'][field] = value
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'restore four actual'):
                self.validate(row)

    def test_metadata_gate_requires_four_distinct_placed_owners(self):
        row = self.selected_report()
        self.assertEqual(self.validate(row)['mode'], 'zero_gain_timing')
        # Acceptance here is only this file-only metadata gate. The caller
        # must separately authenticate actual source/artifacts/raw run data.
        for field, value in (('native_tid', 100), ('cpu_mask', 1), ('configured', 0),
                             ('status', -1), ('timer_slack_ns', 50000)):
            changed = copy.deepcopy(row)
            changed['measurement']['native_group_setup']['settings'][1][field] = value
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'restore four actual'):
                self.validate(changed)


if __name__ == '__main__':
    unittest.main()
