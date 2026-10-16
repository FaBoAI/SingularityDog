"""Pure Kp6 proposal checks: no native library, open descriptor or permission."""
import json
from pathlib import Path
import types
import unittest

from experiments.four_bus_type1 import type1_transport as boxed
from experiments.four_bus_diagnostic.transport_adapter import Group
from ._proposal_fixture import reconstruct_proposal

HERE=Path(__file__).resolve().parent


def proposal():
    file=HERE/'type1_transport.py'
    frozen=reconstruct_proposal(HERE/'transport-proposal-manifest.json',
        HERE/'transport_hooks.proposed.patch',file,'proposed_transport_sha256')
    module=types.ModuleType('experiments.four_bus_human_hold._adapter_fixture')
    module.__package__='experiments.four_bus_human_hold';module.__file__=str(file)
    exec(compile(frozen.proposed,str(file),'exec'),module.__dict__)
    return module


class IndependentTransportProposalTests(unittest.TestCase):
    def setUp(self):
        self.transport=proposal();self.group=Group('port3',(1,2,3))
        self.bounds={mid:(-.03,.03) for mid in self.group.ids}
        self.kd={mid:.15 for mid in self.group.ids}

    def limits(self,kp):
        return self.transport.session_limits(self.group,self.bounds,
                         {mid:kp for mid in self.group.ids},self.kd)

    def test_distinct_proposal_accepts6_and_original_boxed_stays3(self):
        lower,upper,kp,kd=self.limits(6.)
        self.assertEqual([kp[mid] for mid in (1,2,3)],[6.,6.,6.])
        self.assertEqual(self.transport.KP_CAP,6.)
        self.assertEqual(boxed.KP_CAP,3.)
        with self.assertRaises(ValueError):
            boxed.session_limits(self.group,self.bounds,{mid:6. for mid in self.group.ids},self.kd)
        doc=json.loads((HERE/'transport-proposal-manifest.json').read_text())
        self.assertFalse(doc['compiled_or_installed'])
        self.assertFalse(doc['output_allowed'])

    def test_excess_gain_nan_and_bad_keys_rejected(self):
        for kp in (6.001,float('nan'),float('inf'),-1.):
            with self.subTest(kp=kp),self.assertRaises(ValueError): self.limits(kp)
        with self.assertRaises(ValueError):
            self.transport.session_limits(self.group,self.bounds,{1:6.,2:6.,3:6.,4:0.},self.kd)

    def test_nonmembers_remain_zero_gain_and_unreachable(self):
        lower,upper,kp,kd=self.limits(6.)
        for mid in (4,5,6):
            self.assertEqual(kp[mid],0.)
            self.assertEqual(kd[mid],0.)
            self.assertEqual((lower[mid],upper[mid]),boxed.unreachable_window())
        self.assertEqual(self.transport.GAP_NS,900_000)
        self.assertEqual(self.transport.WINDOW,3)
        self.assertEqual(self.transport.KD_CAP,.15)


if __name__=='__main__':unittest.main()
