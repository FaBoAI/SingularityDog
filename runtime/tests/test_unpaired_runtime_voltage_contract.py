"""Synthetic file-only admission boundaries; not hardware/profile evidence."""
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

from singularitydog_hw import policy_live_profile as live
from singularitydog_hw import can_readonly as codec
import test_policy_live_profile as legacy_fixture



class ContractTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        fixture = legacy_fixture.EvidenceJSONBoundTests(); fixture.base = self.base
        _, self.report, self.data, self.rows, self.records, self.proof = fixture.trace_fixture(fast=True)
        self.original_report = copy.deepcopy(self.report)
        self.original_data = copy.deepcopy(self.data)
        self.original_records = copy.deepcopy(self.records)
        self.data.update(schema=live.SCHEMA_V3, native_feedback_batch_decode=True,
            native_phase_pair=False, scope='supported_characterization_only',
            model_backend=live.SCALAR_BACKEND, local_characterization=live.LOCAL_RELATIVE_SUPPORTED,
            watchdog_review_policy=live.COMMAND_LOSS_ONLY_SUPPORTED,
            voltage_overlap=True, voltage_pipeline=True, request_gap_us=900, request_window=3,
            hard_cycle_ms=20, max_sample_age_ms=20, max_consecutive_20ms_misses=0,
            policy_weight=.005, diagnostic_timing_acceptance=live.SUPPORTED_POLICY_PROBE_2S_RARE_JITTER,
            duration_s=2., axes={str(i):dict(kp=3.,kd=.15,max_displacement_from_start_rad=math.radians(1),
                max_estimated_pd_torque_nm=.1) for i in range(1,13)},
            cadence_source_sha256={'synthetic_only': '0'*64})
        self.proof.update(voltage_dispatch_schedule=live._UNPAIRED_CODEC_VOLTAGE_SCHEDULE,
                          feedback_publication='after_voltage_native_preparation')
        self.report.update(cadence_source_sha256=self.data['cadence_source_sha256'],
            source_provenance=dict(source_files_unchanged=True,cadence_source_sha256=self.data['cadence_source_sha256']),
            native_phase_pair=dict(enabled=False),
            private_seven_request_experiment=dict(branch='baseline6plus1',genuine_unpaired_sessions=True,
                active_phase_pair_used=False,original_collector_worker_count=3,
                extra_native_owner_worker_count=0,all_phase_owners_closed=True,fd_release_safe=True),
            diagnostic_runtime_output=dict(schema='singularitydog.diagnostic-runtime-output-owner.v1',
                scope='disabled_stop_proxy_only',actual_busworkers_submit_decoded=True,
                actual_busworkers_collect_output=True,genuine_original_output_futures=True,
                native_feedback_batch_decode_selected=True,original_absolute_deadlines_unchanged=True,
                output_future_notifications_selected=False,collector_worker_count=3,
                extra_worker_or_reader_count=0,borrowed_executor_owner='original_collector',
                owns_or_closes_borrowed_executor=False,type1_sent=False,
                active_controller_qualification=False,output_allowed=False,approved_for_runtime=False))
        record = self.records[0]; trace = record['voltage_fast_pipeline']; release = self.rows[0]['release_ns']
        for bus, ids in (('front',range(1,7)),('rear',range(7,13))):
            for phase, count, sent, received in (('acquired',6,release+100,release+8_000_000),
                    ('voltage',1,release+8_200_000,release+11_000_000),
                    ('output',6,release+13_000_000,release+16_000_000)):
                part = record[phase][bus]
                part.update(stats=dict(begin_ns=sent,end_ns=received+1,writes=count,bytes=count*17),rejected_hex='')
                for raw in part['records']:
                    raw.update(start_ns=sent,finish_ns=sent+10,read_start_ns=received-1,
                        received_ns=received,deadline_ns=release+20_000_000,written=17,received=17)
            for mid, raw in zip(ids, record['acquired'][bus]['records']):
                raw['rx_hex']=(b'AT'+((((2<<24)|(mid<<8)|0xfd)<<3)|4).to_bytes(4,'big')+
                    b'\x08'+bytes(8)+b'\r\n').hex()
            record['voltage'][bus]['records'][0]['rx_hex']=(b'AT'+((((17<<24)|(ids[0]<<8)|0xfd)<<3)|4).to_bytes(4,'big')+
                b'\x08'+bytes(8)+b'\r\n').hex()
        self.freeze()

    def freeze(self):
        raw=(json.dumps(self.records,sort_keys=True,allow_nan=False)+'\n').encode()
        (self.base/'records.json').write_bytes(raw)
        self.proof['records_sha256']=hashlib.sha256(raw).hexdigest()
        self.report['unpaired_codec_voltage_dispatch_contract']=live._make_unpaired_codec_voltage_contract(self.report)

    def run_reader(self):
        return live._voltage_fast_pipeline_trace(self.report,self.data,self.rows)

    def test_new_source_specific_contract_accepts_complete_synthetic_raw_without_relabel(self):
        before=copy.deepcopy((self.report,self.data,self.rows,self.records))
        self.run_reader()
        self.assertEqual((self.report,self.data,self.rows,self.records),before)
        self.assertEqual(self.proof['voltage_dispatch_schedule'],live._UNPAIRED_CODEC_VOLTAGE_SCHEDULE)

    def test_legacy_trace_fixture_still_accepted_identically(self):
        raw=json.dumps(self.original_records).encode();(self.base/'records.json').write_bytes(raw)
        self.original_report['v3_voltage_fast_pipeline']['records_sha256']=hashlib.sha256(raw).hexdigest()
        live._voltage_fast_pipeline_trace(self.original_report,self.original_data,self.rows)

    def test_new_label_without_fresh_contract_marker_rejected(self):
        self.report.pop('unpaired_codec_voltage_dispatch_contract')
        with self.assertRaisesRegex(live.ProfileError,'source/records-bound'):self.run_reader()

    def test_same_label_cannot_qualify_unselected_legacy_profile(self):
        self.data['native_feedback_batch_decode']=False
        with self.assertRaisesRegex(live.ProfileError,'explicit unpaired runtime selection'):self.run_reader()

    def test_selected_codec_cannot_launder_old_generic_schedule_label(self):
        self.proof['voltage_dispatch_schedule']='after_each_bus_feedback'
        self.report.pop('unpaired_codec_voltage_dispatch_contract')
        with self.assertRaisesRegex(live.ProfileError,'own truthful'):self.run_reader()

    def test_combined_notification_proof_cannot_qualify_codec_poll_path(self):
        self.report['diagnostic_runtime_output']['output_future_notifications_selected']=True
        self.freeze()
        with self.assertRaises(live.ProfileError):self.run_reader()

    def notification_case(self,codec_selected=False):
        self.data.update(native_feedback_batch_decode=codec_selected,
            unpaired_output_future_notifications=True,period_ms=20,max_sample_gap_ms=21,
            voltage_min_v=35.,voltage_max_v=42.,command=[0.,0.,0.])
        self.report['diagnostic_runtime_output'].update(native_feedback_batch_decode_selected=codec_selected,
            output_future_notifications_selected=True)
        self.freeze()

    def test_independently_selected_notification_trace_has_both_settings_sealed(self):
        self.notification_case();self.run_reader()
        contract=self.report['unpaired_codec_voltage_dispatch_contract']
        self.assertIs(contract['native_feedback_batch_decode_selected'],False)
        self.assertIs(contract['unpaired_output_future_notifications_selected'],True)

    def test_combined_selection_requires_current_true_true_marker(self):
        self.notification_case(True);self.run_reader()
        self.data['unpaired_output_future_notifications']=False
        with self.assertRaises(live.ProfileError):self.run_reader()

    def test_matching_trace_is_not_missing_notification_loader_evidence(self):
        self.notification_case();self.run_reader()
        with self.assertRaisesRegex(live.ProfileError,'Own-source'):
            live._unpaired_output_future_notifications_evidence({'pipeline_diagnostic':self.report},self.data)

    def test_new_contract_never_accepts_changed_boolean_or_old_schema(self):
        self.notification_case();before=copy.deepcopy(self.report['unpaired_codec_voltage_dispatch_contract'])
        for key,value in (('schema','singularitydog.unpaired-codec-voltage-dispatch.v1'),
                           ('unpaired_output_future_notifications_selected',False)):
            self.report['unpaired_codec_voltage_dispatch_contract']=copy.deepcopy(before)
            self.report['unpaired_codec_voltage_dispatch_contract'][key]=value
            with self.assertRaises(live.ProfileError):self.run_reader()
    def test_split7_label_and_phase_pair_forbidden(self):
        for part in ('label','branch','pair'):
            with self.subTest(part=part):
                r=copy.deepcopy(self.report)
                if part=='label':self.proof['voltage_dispatch_schedule']='combined_native7; seventh_write_can_precede_all_six_feedback_replies'
                if part=='branch':self.report['private_seven_request_experiment']['branch']='split7'
                if part=='pair':self.report['native_phase_pair']['enabled']=True
                self.freeze()
                with self.assertRaises(live.ProfileError):self.run_reader()
                self.report=r;self.proof=r['v3_voltage_fast_pipeline']

    def test_changed_source_or_source_seal_refused(self):
        for part in ('source','seal'):
            r=copy.deepcopy(self.report)
            if part=='source':self.report['cadence_source_sha256']={'synthetic_only':'1'*64}
            else:self.report['source_provenance']['source_files_unchanged']=False
            self.freeze()
            with self.assertRaisesRegex(live.ProfileError,'source'):self.run_reader()
            self.report=r;self.proof=r['v3_voltage_fast_pipeline']

    def test_original_actual_busworker_owner_metadata_required(self):
        for key,bad in (('actual_busworkers_submit_decoded',False),('actual_busworkers_collect_output',False),
                ('genuine_original_output_futures',False),('collector_worker_count',5),
                ('extra_worker_or_reader_count',1),('borrowed_executor_owner','new_executor'),
                ('original_absolute_deadlines_unchanged',False),('type1_sent',True)):
            with self.subTest(key=key):
                r=copy.deepcopy(self.report);self.report['diagnostic_runtime_output'][key]=bad
                self.freeze()
                with self.assertRaises(live.ProfileError):self.run_reader()
                self.report=r;self.proof=r['v3_voltage_fast_pipeline']

    def test_true_caps_not_widened_by_new_schedule(self):
        for key,bad in (('request_gap_us',890),('request_window',2),('hard_cycle_ms',21),
                ('max_sample_age_ms',21),('policy_weight',.006),('duration_s',20)):
            with self.subTest(key=key):
                d=copy.deepcopy(self.data);self.data[key]=bad
                with self.assertRaises(live.ProfileError):self.run_reader()
                self.data=d

    def test_partial_native6_raw_never_accepted_by_complete_timeline(self):
        for bus in ('front','rear'):
            with self.subTest(bus=bus):
                records=copy.deepcopy(self.records);self.records[0]['acquired'][bus]['records'].pop()
                self.freeze()
                with self.assertRaises(live.ProfileError):self.run_reader()
                self.records=records;self.freeze()

    def test_raw_sixth_reply_after_voltage_request_rejected_despite_original_markers(self):
        trace=self.records[0]['voltage_fast_pipeline'];part=self.records[0]['acquired']['front']
        part['records'][-1]['received_ns']=self.records[0]['voltage']['front']['records'][0]['start_ns']+1
        part['stats']['end_ns']=part['records'][-1]['received_ns']+1
        self.freeze()
        with self.assertRaises(live.ProfileError):self.run_reader()

    def test_native6_end_after_native1_begin_rejected(self):
        self.records[0]['acquired']['rear']['stats']['end_ns']=self.records[0]['voltage']['rear']['stats']['begin_ns']+1
        self.freeze()
        with self.assertRaises(live.ProfileError):self.run_reader()

    def test_native_phase_stats_end_cannot_follow_actual_join_or_verification(self):
        for phase, key in (('voltage','voltage_join_ns'),('output','stop_reply_verified_ns')):
            with self.subTest(phase=phase):
                records=copy.deepcopy(self.records)
                self.records[0][phase]['rear']['stats']['end_ns']=self.records[0]['voltage_fast_pipeline'][key]+1
                self.freeze()
                with self.assertRaisesRegex(live.ProfileError,'Stats'):self.run_reader()
                self.records=records;self.freeze()

    def test_raw_feedback_timestamp_cannot_be_hidden_behind_metadata(self):
        self.records[0]['voltage_fast_pipeline']['feedback_reply_end_ns_by_bus']['front']-=1
        self.freeze()
        with self.assertRaisesRegex(live.ProfileError,'before all six'):self.run_reader()

    def test_native_raw_rejected_noise_and_incomplete_bytes_fail(self):
        for part in ('noise','bytes'):
            r=copy.deepcopy(self.records)
            if part=='noise':self.records[0]['voltage']['front']['rejected_hex']='00'
            else:self.records[0]['acquired']['front']['records'][5]['received']=11
            self.freeze()
            with self.assertRaises(live.ProfileError):self.run_reader()
            self.records=r;self.freeze()

    def test_acquired_mode_or_fault_ids_cannot_be_relabelled_healthy(self):
        raw=self.records[0]['acquired']['front']['records'][0]
        raw['rx_hex']=(b'AT'+((((2<<24)|(2<<22)|(1<<8)|0xfd)<<3)|4).to_bytes(4,'big')+
            b'\x08'+bytes(8)+b'\r\n').hex()
        self.freeze()
        with self.assertRaisesRegex(live.ProfileError,'healthy disabled'):self.run_reader()

    def test_original_finalstop_frame_and_digest_checks_still_run(self):
        raw=self.records[0]['output']['rear']['records'][5];raw['tx_hex']='00'*17
        self.freeze()
        with self.assertRaisesRegex(live.ProfileError,'exact feedback, voltage and STOP'):self.run_reader()
        (self.base/'records.json').write_text('[]\n')
        with self.assertRaisesRegex(live.ProfileError,'SHA256 mismatch'):self.run_reader()


if __name__ == '__main__':
    unittest.main()
