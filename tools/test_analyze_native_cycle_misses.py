"""Saved-host-timing classification tests; no device access."""
import unittest

from analyze_native_cycle_misses import analyze


def trial(inference, dispatch, whole):
    base = 1_000_000_000
    inferred = base + int(inference*1e6)
    begin = inferred + int(dispatch*1e6)
    end = base + int(whole*1e6)
    measurement = {'release_ns': base, 'infer_end_ns': inferred,
                   'cycle_end_ns': end, 'inference_ms': inference,
                   'whole_iteration_ms': whole}
    record = {'output': {bus: {'stats': {'begin_ns': begin},
                               'records': [{'start_ns': begin+100,
                                            'written': 17, 'received': 17} for _ in range(6)]}
                         for bus in ('front', 'rear')}}
    return measurement, record


class CycleMissTests(unittest.TestCase):
    def test_reports_model_call_budget_from_saved_profile(self):
        measurement, record = trial(5.5, .2, 20.6)
        record['observed'] = {'consume_profile': {'durations_ns': {
            'model_call': 5_200_000}}}
        report = {'status':'COMPLETE_DIAGNOSTIC','mode':'stop-proxy',
                  'motor_enable_sent':False,'learned_targets_sent':False,
                  'cycles_completed':1,'measurements':[measurement]}
        result = analyze(report, [{**record, 'cycle':1}])
        budget = result['model_call_budget']
        self.assertAlmostEqual(budget['model_call_ms']['median'], 5.2)
        self.assertAlmostEqual(budget['other_cycle_work_ms']['median'], 15.4)
        self.assertAlmostEqual(budget['miss_rows'][0]['available_model_budget_ms'], 4.6)
        self.assertAlmostEqual(budget['miss_rows'][0]['minimum_saving_needed_ms'], .6)

    def test_distinguishes_model_and_dispatch_late_cycles(self):
        cases = [trial(5.5, .2, 20.6), trial(3.8, 3.1, 21.4),
                 trial(4.8, .2, 20.1), trial(3.9, .2, 19.5)]
        report = {'status':'COMPLETE_DIAGNOSTIC','mode':'stop-proxy',
                  'motor_enable_sent':False,'learned_targets_sent':False,
                  'cycles_completed':len(cases),
                  'measurements':[m for m,_ in cases]}
        records = [{**r, 'cycle':i} for i,(_,r) in enumerate(cases,1)]
        result = analyze(report, records)
        self.assertEqual(result['whole_iteration_over_20ms'], 3)
        self.assertEqual(result['inference_slow_miss_cycles'], [1])
        self.assertEqual(result['pre_native_output_slow_miss_cycles'], [2])
        self.assertEqual(result['other_miss_cycles'], [3])

    def test_rejects_missing_output_and_wrong_cycle_order(self):
        measurement, record = trial(3.8, .2, 19.5)
        report = {'status':'COMPLETE_DIAGNOSTIC','mode':'stop-proxy',
                  'motor_enable_sent':False,'learned_targets_sent':False,
                  'cycles_completed':1,'measurements':[measurement]}
        with self.assertRaisesRegex(ValueError, 'Record cycle'):
            analyze(report, [{**record, 'cycle':2}])
        with self.assertRaisesRegex(ValueError, 'Missing complete output'):
            analyze(report, [{'cycle':1, 'output':{}}])

    def test_opt_in_trace_locates_main_check_and_gc_without_claiming_cause(self):
        measurement, record = trial(3.8, 3.1, 21.4)
        base = measurement['infer_end_ns']
        begin = record['output']['front']['stats']['begin_ns']
        fields = ['infer_end_ns','main_check_start_ns','main_check_end_ns',
                  'front_submit_end_ns','rear_submit_end_ns',
                  'front_worker_enter_ns','front_worker_check_end_ns',
                  'front_native_begin_ns','front_first_write_ns',
                  'rear_worker_enter_ns','rear_worker_check_end_ns',
                  'rear_native_begin_ns','rear_first_write_ns',
                  'main_infer_thread_cpu_ns','main_submits_end_thread_cpu_ns']
        values = [base,base+100,base+2_000_000,base+2_200_000,base+2_300_000,
                  base+2_500_000,base+2_600_000,begin,begin+100,
                  base+2_700_000,base+2_800_000,begin,begin+100,100_000,2_500_000]
        report = {'status':'COMPLETE_DIAGNOSTIC','mode':'stop-proxy',
                  'motor_enable_sent':False,'learned_targets_sent':False,
                  'cycles_completed':1,'measurements':[measurement],
                  'output_dispatch_trace':{'schema':'native-output-dispatch-v1',
                    'fields':fields,'rows':[values],
                    'gc_events':[{'monotonic_ns':base+1_000_000,'native_tid':123,
                                  'cycle':1,'generation':0,'phase':'start'},
                                 {'monotonic_ns':base+2_500_000,'native_tid':123,
                                  'cycle':1,'generation':0,'phase':'stop'}],
                    'gc_overflow':0,'gc_probe_errors':0}}
        result = analyze(report,[{**record,'cycle':1}])['output_dispatch_detail']
        self.assertEqual(result['slow_dispatch_rows'][0]['main_check_ms'],1.9999)
        self.assertEqual(len(result['slow_dispatch_rows'][0]['gc_events_in_interval']),2)
        self.assertEqual(result['whole_iteration_miss_rows'][0]['cycle'],1)
        self.assertEqual(result['whole_iteration_miss_rows'][0]['front_submit_to_native_ms'],.9)
        self.assertEqual(result['whole_iteration_miss_rows'][0]
                         ['gc_intervals_overlapping_cycle'][0]['duration_ms'],1.5)
        self.assertEqual(result['gc_overlap_miss_cycles'],[1])
        self.assertIn('correlation',result['interpretation'])

    def test_trace_reports_delayed_rear_bus_even_when_front_is_on_time(self):
        measurement, record = trial(3.8, .2, 21.4)
        base = measurement['infer_end_ns']
        front_begin = record['output']['front']['stats']['begin_ns']
        rear_begin = base + 3_300_000
        record['output']['rear']['stats']['begin_ns'] = rear_begin
        for frame in record['output']['rear']['records']:
            frame['start_ns'] = rear_begin+100
        fields = ['infer_end_ns','main_check_start_ns','main_check_end_ns',
                  'front_submit_end_ns','rear_submit_end_ns',
                  'front_worker_enter_ns','front_worker_check_end_ns',
                  'front_native_begin_ns','front_first_write_ns',
                  'rear_worker_enter_ns','rear_worker_check_end_ns',
                  'rear_native_begin_ns','rear_first_write_ns',
                  'main_infer_thread_cpu_ns','main_submits_end_thread_cpu_ns']
        values = [base,base+100,base+200,base+300,base+400,
                  base+500,base+600,front_begin,front_begin+100,
                  base+700,base+800,rear_begin,rear_begin+100,100_000,100_100]
        report = {'status':'COMPLETE_DIAGNOSTIC','mode':'stop-proxy',
                  'motor_enable_sent':False,'learned_targets_sent':False,
                  'cycles_completed':1,'measurements':[measurement],
                  'output_dispatch_trace':{'schema':'native-output-dispatch-v1',
                    'fields':fields,'rows':[values],
                    'gc_events':[{'monotonic_ns':base+900,'native_tid':124,
                                  'cycle':1,'generation':1,'phase':'start'},
                                 {'monotonic_ns':rear_begin-100,'native_tid':124,
                                  'cycle':1,'generation':1,'phase':'stop'}],
                    'gc_overflow':0,'gc_probe_errors':0}}
        result = analyze(report,[{**record,'cycle':1}])
        self.assertEqual(result['pre_native_output_slow_miss_cycles'],[])
        row = result['output_dispatch_detail']['whole_iteration_miss_rows'][0]
        self.assertAlmostEqual(row['rear_submit_to_native_ms'],3.2996)
        self.assertAlmostEqual(row['front_submit_to_native_ms'],.1997)
        self.assertEqual(row['gc_intervals_overlapping_cycle'][0]['generation'],1)
        self.assertEqual(result['output_dispatch_detail']
                         ['any_bus_submit_to_native_slow_miss_cycles'],[1])


if __name__ == '__main__':
    unittest.main()
