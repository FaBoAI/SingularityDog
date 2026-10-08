"""Offline comparison contracts; no real devices or network."""
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from contextlib import redirect_stdout, redirect_stderr
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('offline_native_pair_comparison',HERE/'offline_native_pair_comparison.py')
tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)


class OfflinePairComparisonTests(unittest.TestCase):
    def test_default_plan_never_constructs_a_socket(self):
        stream=io.StringIO()
        with patch.object(tool.socket,'socketpair',side_effect=AssertionError('Socket opened in PLAN')):
            with redirect_stdout(stream):self.assertEqual(tool.main(['--cycles','5']),0)
        report=json.loads(stream.getvalue())
        self.assertEqual(report['status'],'PLAN_ONLY');self.assertEqual(report['requests_per_cycle'],26)
        self.assertFalse(report['hardware_opened']);self.assertFalse(report['hardware_timing_qualified'])
        self.assertFalse(report['prepared_feedback_voltage_overlap_emulated'])

    def test_cycles_are_bounded_and_bool_not_numeric_count(self):
        for count in (0,501,-1,True,1.5):
            with self.assertRaises(ValueError):tool.plan(count)

    def test_fake_peer_rejects_enable_type1_and_noncanonical_requests(self):
        for wire in (tool.frame((3<<24)|(0xfd<<8)|1,bytes(8)),
                     tool.native.encode_motion(1,0,3,.15),b'X'*17):
            with self.assertRaises(ValueError):tool.fake_reply(wire)
        self.assertEqual(len(tool.fake_reply(tool.stop_wire(12))),17)
        self.assertEqual(len(tool.fake_reply(tool.voltage_wire(7))),17)

    def test_existing_private_output_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'retained';path.mkdir();(path/'original').write_text('retain')
            with patch.object(tool.socket,'socketpair',side_effect=AssertionError('Socket opened')):
                with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                    tool.main(['--execute-offline','--output',str(path)])
            self.assertEqual((path/'original').read_text(),'retain')

    def test_all_summary_attempts_and_failures_are_retained(self):
        rows=[{'status':'COMPLETE_FAKE_CYCLE','elapsed_ns':20_100_000,'phases':[]},
              {'status':'INCOMPLETE_FAKE_CYCLE','elapsed_ns':25_000_000,'phases':[]}]
        report=tool.summary(rows,20)
        self.assertEqual(report['requested_cycles'],20);self.assertEqual(report['attempted_cycles'],2)
        self.assertEqual(report['complete_cycles'],1);self.assertEqual(report['incomplete_cycles'],1)
        self.assertEqual(report['complete_host_cycles_over_20ms'],1)


if __name__=='__main__':unittest.main()
