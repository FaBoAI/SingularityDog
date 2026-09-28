"""Offline smoke harness contracts and real native failure paths, no private inputs."""
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest

import offline_policy_output_smoke as smoke


class PlantContractTests(unittest.TestCase):
    def setUp(self):
        self.plant=smoke.SocketPlant(None,tuple(range(1,7)))

    def parsed(self,wire):return smoke.codec.ATParser().feed(wire)[0]

    def test_synthetic_identity_and_watchdog_readback_not_hardware_evidence(self):
        f=self.parsed(smoke.codec.read_request(1))
        self.assertEqual(self.parsed(self.plant.respond(f)).data,bytes([1])*8)
        request=smoke.runtime.protocol.watchdog_setup_request(
            phase=smoke.runtime.protocol.TrialPhase.WATCHDOG_SETUP,motor_id=1)
        ack=self.parsed(self.plant.respond(self.parsed(request)))
        self.assertEqual(ack.kind,2)
        self.assertEqual(ack.can_id,(2<<24)|(1<<8)|0xfd)
        self.assertEqual(ack.data,struct.pack('>4H',self.plant.positions[1],32767,32767,250))
        self.assertNotIn(1,self.plant.enabled)
        response=self.parsed(self.plant.respond(self.parsed(smoke.codec.read_request(1,'can_timeout'))))
        self.assertEqual(response.kind,17)
        self.assertEqual(response.data[:4],bytes.fromhex('28700000'))
        self.assertEqual(struct.unpack('<I',response.data[4:])[0],4000)

    def test_motion_requires_enable_and_stop_never_clears_faults(self):
        motion=self.parsed(smoke.native.encode_motion(1,-.8,4,.2))
        with self.assertRaisesRegex(ValueError,'before explicit enable'):self.plant.respond(motion)
        enable=smoke.runtime.protocol.enable_request(phase=smoke.runtime.protocol.TrialPhase.ENABLE,motor_id=1)
        self.plant.respond(self.parsed(enable))
        self.assertEqual((self.parsed(self.plant.respond(motion)).can_id>>22)&3,2)
        bad_stop=smoke.wire((4<<24)|(0xfd<<8)|1,b'\x01'+bytes(7))
        with self.assertRaisesRegex(ValueError,'must not clear faults'):self.plant.respond(self.parsed(bad_stop))
        self.assertIn(1,self.plant.enabled)
        stop=smoke.runtime.protocol.stop_request(phase=smoke.runtime.protocol.TrialPhase.STOP,motor_id=1)
        self.assertEqual((self.parsed(self.plant.respond(self.parsed(stop))).can_id>>22)&3,0)

    def test_unsupported_cross_bus_and_out_of_range_are_rejected(self):
        for request in (smoke.codec.read_request(7),smoke.wire((6<<24)|(0xfd<<8)|1,bytes(8))):
            with self.assertRaises(ValueError):self.plant.respond(self.parsed(request))
        for value in (12.58,float('nan')):
            with self.assertRaises(ValueError):smoke.quantize(value,12.57)

    def test_profile_is_explicit_in_memory_simulation_without_review(self):
        p=smoke.synthetic_profile()
        self.assertTrue(p['simulation_only'])
        self.assertFalse(p['physical_review_generated'])
        self.assertNotIn('review',p)
        self.assertNotIn('approved_for_supported_policy_output',p)
        self.assertTrue(all(a['sign']==1 for a in p['axes'].values()))


class NativeFailureIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=smoke.REPO/'runtime/experiments/native_active_transport/build.py'
        spec=importlib.util.spec_from_file_location('build_smoke_active_transport',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        cls.lib=smoke.native.load_library(module.build())

    def test_dropped_feedback_poison_attempts_both_buses_and_keeps_ambiguity(self):
        with tempfile.TemporaryDirectory() as folder:
            report=smoke.run_case('drop-feedback',self.lib,Path(folder)/'case')
            self.assertEqual(report['status'],'PASS_OFFLINE_SIMULATION',report['errors'])
            self.assertEqual(report['runtime']['status'],'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
            self.assertIn(1,report['runtime']['stop_reports']['front']['ambiguous_ids'])
            self.assertEqual(report['runtime']['stop_reports']['front']['attempted_ids'],list(range(1,7)))
            self.assertEqual(report['runtime']['stop_reports']['rear']['attempted_ids'],list(range(7,13)))
            self.assertFalse(report['runtime']['normal_ramp_completed'])
            self.assertTrue(report['all_local_fds_closed'])
            saved=json.loads((Path(folder)/'case/report.json').read_text())
            self.assertFalse(saved['approved_for_runtime'])
            self.assertFalse(saved['hardware_opened'])
            self.assertTrue(all(p['thread_joined'] for p in saved['peers'].values()))

    def test_disconnect_cannot_turn_stop_attempt_into_confirmation(self):
        with tempfile.TemporaryDirectory() as folder:
            report=smoke.run_case('usb-disconnect',self.lib,Path(folder)/'case')
            self.assertEqual(report['status'],'PASS_OFFLINE_SIMULATION',report['errors'])
            self.assertEqual(report['runtime']['status'],'STOP_UNCONFIRMED_POWER_OFF_REQUIRED')
            front=report['runtime']['stop_reports']['front']
            self.assertEqual(front['confirmed_ids'],[])
            self.assertEqual(front['unconfirmed_ids'],list(range(1,7)))
            self.assertEqual(front['attempted_ids'],list(range(1,7)))
            self.assertFalse(report['physical_motor_enable_sent'])
            self.assertFalse(report['physical_learned_targets_sent'])
            self.assertFalse(report['physical_review_generated'])
            self.assertTrue(report['all_local_fds_closed'])
            for peer in report['peers'].values():
                self.assertEqual(peer['errors'],[])
                self.assertEqual(peer['residual_bytes'],0)


if __name__=='__main__':unittest.main()
