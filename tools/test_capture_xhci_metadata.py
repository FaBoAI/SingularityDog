from pathlib import Path
import tempfile
import unittest
from tools.capture_xhci_metadata import device_pipes, trace_stats_lossless


class PipeTests(unittest.TestCase):
    def make_usb(self, root):
        p = root/'1-2.2'; p.mkdir()
        for name, value in [('idVendor', '1a86'), ('idProduct', '7523'), ('devnum', '4'), ('busnum', '1')]:
            (p/name).write_text(value)
        for address, attr in [('02', '02'), ('81', '03'), ('82', '02')]:
            ep = root/'1-2.2:1.0'/('ep_' + address); ep.mkdir(parents=True)
            (ep/'bEndpointAddress').write_text(address)
            (ep/'bmAttributes').write_text(attr)

    def test_pipe_filter_is_adapter_bulk_only(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); self.make_usb(root)
            r = device_pipes(['1-2.2'], root)
            self.assertEqual(r['1-2.2']['pipes'], [0xc0010400, 0xc0410480])

    def test_wrong_device_duplicate_or_path_traversal_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); self.make_usb(root)
            for names in (['../1-2.2'], ['1-2.2', '1-2.2']):
                with self.assertRaises(ValueError): device_pipes(names, root)
            (root/'1-2.2/idVendor').write_text('1234')
            with self.assertRaises(ValueError): device_pipes(['1-2.2'], root)

    def test_overrun_makes_capture_incomplete(self):
        clean = 'entries: 500\noverrun: 0\ncommit overrun: 0\ndropped events: 0\n'
        lost = clean.replace('overrun: 0\n', 'overrun: 1\n', 1)
        self.assertTrue(trace_stats_lossless({'cpu0': clean, 'cpu1': clean}))
        self.assertFalse(trace_stats_lossless({'cpu0': lost, 'cpu1': clean}))
        self.assertFalse(trace_stats_lossless({}))


if __name__ == '__main__': unittest.main()
