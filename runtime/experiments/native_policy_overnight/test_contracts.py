"""CPU contract tests; no model weights/private fixtures required."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from native_policy_overnight import contracts as c
from native_policy_overnight import loader
from native_policy_overnight import verification as v


class FileContractTests(unittest.TestCase):
    def test_reused_sources_unchanged(self):
        for name, digest in c.REUSED_PINS.items():
            c.pinned(c.HERE / name, digest)

    def test_duplicate_json_rejected(self):
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            c.strict_json('{"same": 1, "same": 2}')

    def test_nonfinite_json_rejected(self):
        for value in ("NaN", "Infinity", "-Infinity"):
            with self.assertRaisesRegex(ValueError, "Nonfinite"):
                c.strict_json('{"value": '+value+'}')

    def test_artifact_traversal_rejected(self):
        for name in ("../file", "a/b", "/tmp/file", ".", "..", "", None):
            with self.assertRaises(ValueError):
                c.member(Path("/tmp/example"), name)

    def test_hash_and_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plain"
            path.write_bytes(b"source")
            self.assertEqual(c.pinned(path, c.sha(b"source")), b"source")
            with self.assertRaisesRegex(ValueError, "SHA"):
                c.pinned(path, "0"*64)
            link = Path(directory) / "link"
            link.symlink_to(path)
            with self.assertRaisesRegex(ValueError, "nonsymlink"):
                c.pinned(link, c.sha(b"source"))
            with self.assertRaisesRegex(ValueError, "nonsymlink"):
                loader.load_library(link, c.sha(b"source"))

    def test_loader_rejects_incomplete_before_library_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "manifest.json"
            raw = b'{"schema":"native-policy-overnight-v1","status":"INCOMPLETE"}'
            path.write_bytes(raw)
            with mock.patch.object(loader, "load_library") as register:
                with self.assertRaisesRegex(ValueError, "Unvalidated"):
                    loader.load_verified(path, expected_manifest_sha256=c.sha(raw), bundle=directory)
                register.assert_not_called()

    def test_loader_rejects_source_drift_before_bundle_or_library(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "manifest.json"
            raw = json.dumps(dict(schema="native-policy-overnight-v1", status="VALIDATED_FILE_ONLY",
                                  source_hashes={})).encode()
            path.write_bytes(raw)
            with mock.patch.object(loader, "load_library") as register:
                with self.assertRaisesRegex(ValueError, "Experiment source"):
                    loader.load_verified(path, expected_manifest_sha256=c.sha(raw), bundle=directory)
                register.assert_not_called()

    def test_loaded_library_cannot_silently_change(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "candidate.so"
            path.write_bytes(b"not executable")
            with mock.patch.object(loader, "_LOADED_LIBRARY", ("different", "0"*64)):
                with self.assertRaisesRegex(ValueError, "Different native library"):
                    loader.load_library(path, c.sha(path.read_bytes()))


class TensorContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        cls.torch = torch

    def raw_input(self):
        return {key: [0.]*width for key, width in zip(c.INPUT_KEYS, (3, 3, 3, 12, 12, 12))}

    def test_saved_shapes_and_boolean_rejected(self):
        for key in c.INPUT_KEYS:
            data = self.raw_input()
            data[key].pop()
            with self.assertRaisesRegex(ValueError, "Invalid saved"):
                v.saved_inputs(self.torch, json.dumps(data).encode())
            data = self.raw_input()
            data[key][0] = True
            with self.assertRaisesRegex(ValueError, "Invalid saved"):
                v.saved_inputs(self.torch, json.dumps(data).encode())

    def test_saved_capture_formats_match_without_mutating_source(self):
        data = self.raw_input()
        encodings = [data, {"inputs": data}, {"inference": {"inputs": data}},
                     {"observation": {"observer_tick": {"inputs": data}}}]
        outputs = [v.saved_inputs(self.torch, json.dumps(item).encode())[0] for item in encodings]
        for candidate in outputs[1:]:
            self.assertTrue(all(self.torch.equal(a, b) for a, b in zip(outputs[0], candidate)))
        self.assertEqual([tuple(t.shape) for t in outputs[0]], [(1, 3)]*3+[(1, 12)]*3)

    def test_26_distinct_invalid_cases(self):
        base, _ = v.saved_inputs(self.torch, json.dumps(self.raw_input()).encode())
        cases = v.rejections(self.torch, base)
        self.assertEqual(len(cases), 26)
        self.assertTrue(all(self.torch.isfinite(x).all() for x in base))
        self.assertTrue(all(x is not y for row in cases.values() for x, y in zip(row, base)))

    def test_parity_rejects_changed_state_even_if_target_matches(self):
        a = self.torch.nn.Module()
        a.register_buffer("state", self.torch.tensor([0.], dtype=self.torch.float64))
        b = copy.deepcopy(a)
        b.state.add_(.01)
        with self.assertRaises(AssertionError):
            v.compare_state(self.torch, a, b, {})

    def test_parameters_must_match_exactly(self):
        a = self.torch.nn.Linear(1, 1)
        b = copy.deepcopy(a)
        with self.torch.no_grad():
            b.weight.copy_(self.torch.nextafter(a.weight, self.torch.full_like(a.weight, float("inf"))))
        with self.torch.inference_mode(), self.assertRaisesRegex(ValueError, "Exact tensor"):
            v.compare_state(self.torch, a, b, {})

    def test_nonfinite_output_rejected(self):
        with self.assertRaisesRegex(ValueError, "Nonfinite"):
            v.compare_tensor(self.torch, self.torch.tensor([float("nan")]), self.torch.tensor([0.]), "output", {})

    def test_dtype_change_rejected(self):
        with self.assertRaisesRegex(ValueError, "metadata"):
            v.compare_tensor(self.torch, self.torch.tensor([0.]), self.torch.tensor([0.], dtype=self.torch.float64), "output", {})


if __name__ == "__main__":
    unittest.main()
