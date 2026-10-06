"""Real ScriptMethod collisions, without candidate artifacts or device access."""
import copy
import unittest
from unittest import mock

from . import diagnostic_loader as loader
from .test_diagnostic_loader import fake_model


class RegisteredMethodTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import torch
        except ImportError:
            raise unittest.SkipTest("Torch unavailable for real ScriptMethod regression")
        cls.torch = torch

    def policies(self, changed=False):
        torch = self.torch
        scalar, candidate = fake_model(), fake_model()
        seed = torch.nn.Sequential(torch.nn.Linear(2, 2), torch.nn.ELU())
        scalar.actor = torch.jit.script(copy.deepcopy(seed))
        candidate.actor = torch.jit.script(
            torch.nn.Sequential(copy.deepcopy(seed[0])) if changed else copy.deepcopy(seed))
        return scalar, candidate

    def validate(self, scalar, candidate):
        with mock.patch.object(loader, "verify_aliases") as aliases, mock.patch.object(
                loader, "_state_bits") as state:
            loader._validate_models(self.torch, scalar, candidate)
            return aliases, state

    def test_sequential_len_python_attribute_collision_uses_registered_code(self):
        scalar, candidate = self.policies()
        self.assertIn("__len__", scalar.actor._c._method_names())
        self.assertFalse(hasattr(scalar.actor.__len__, "code"))
        self.assertIsInstance(scalar.actor._c._get_method("__len__").code, str)
        aliases, state = self.validate(scalar, candidate)
        self.assertEqual(aliases.call_count, 4)
        state.assert_called_once()

    def test_real_different_registered_actor_code_still_rejects(self):
        scalar, candidate = self.policies(changed=True)
        self.assertNotEqual(scalar.actor._c._get_method("__len__").code,
                            candidate.actor._c._get_method("__len__").code)
        with self.assertRaisesRegex(ValueError, "actor executable method differs"):
            self.validate(scalar, candidate)


if __name__ == "__main__":
    unittest.main()
