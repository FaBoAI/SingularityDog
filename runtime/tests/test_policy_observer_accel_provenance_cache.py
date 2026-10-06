"""Owned static provenance parity; fake policy/tensors, no sensors or weights."""
import json
import struct
import sys
import unittest
from unittest.mock import patch

from singularitydog_hw import imu_accel_input_hypothesis as hypotheses
from singularitydog_hw import policy_observer as observer
import test_policy_observer as fixtures


REFERENCE = {'path': '/synthetic/not-opened/hypothesis.json', 'sha256': 'a'*64}
START = 1_000_000_000


def frozen_hypothesis(**changes):
    # This consumer fixture does not authenticate captures or grant output.
    provenance = {'schema': hypotheses.SCHEMA, 'formal_calibration_approved': False,
        'grants_motor_output': False, 'absolute_orientation_error_bound_rad': None,
        'bias_sensor_m_s2': [.03, -.02, -.1], 'scale_sensor': [1., 1., 1.],
        'nested': {'values': [1, 1.0, -0.0, True, None, '方向'],
                   'source_sha256': {'source.py': 'b'*64}}}
    values = dict(bias_m_s2=(.03, -.02, -.1), scale=(1., 1., 1.),
        raw_norm_min_m_s2=9., raw_norm_max_m_s2=10.8,
        corrected_norm_min_m_s2=9., corrected_norm_max_m_s2=10.2,
        reference_sha256='a'*64, candidate_sha256='c'*64, manifest_sha256='d'*64,
        _provenance_json=json.dumps(provenance, ensure_ascii=False, allow_nan=False),
        _proof=hypotheses._VALIDATED)
    values.update(changes)
    return hypotheses.AccelInputHypothesis(**values)


class UncachedCorrection:
    """The previous per-tick provenance path, with identical correct() guards."""
    def __init__(self, frozen):
        self.frozen, self.provenance_calls = frozen, 0
    def correct(self, raw):
        return self.frozen.correct(raw)
    def provenance(self):
        self.provenance_calls += 1
        return self.frozen.provenance()


def make(correction, **options):
    with patch.object(hypotheses, 'load_accel_input_hypothesis', return_value=correction):
        return fixtures.make(accel_input_hypothesis=REFERENCE, max_ticks=4, **options)


def arm(run):
    run.reset_run(START, warmup_completed=True)
    return run


def tree_bits(value):
    """Include types, dict order and IEEE bits: equality alone misses -0.0."""
    kind = type(value)
    if kind is dict:
        return kind.__name__, tuple((key, tree_bits(item)) for key, item in value.items())
    if kind in (list, tuple):
        return kind.__name__, tuple(tree_bits(item) for item in value)
    if kind is float:
        return 'float', struct.pack('>d', value)
    return kind.__name__, value


class AccelProvenanceCacheTests(unittest.TestCase):
    def test_exact_original_class_parses_once_at_setup_and_not_in_tick_loop(self):
        calls = []
        code = hypotheses.strict_json.__code__
        previous = sys.getprofile()
        def count(frame, event, arg):
            if event == 'call' and frame.f_code is code:
                calls.append(frame.f_code)
        try:
            sys.setprofile(count)
            run = make(frozen_hypothesis())
            self.assertEqual(len(calls), 1)
            arm(run)  # The two explicitly requested summaries keep their old path.
            before_ticks = len(calls)
            for index in range(3):
                run.consume(fixtures.snapshot(START+index*observer.DT_NS))
            self.assertEqual(len(calls), before_ticks)
        finally:
            sys.setprofile(previous)
        self.assertIsNotNone(run._accel_provenance_cache)

    def test_complete_records_float_bits_inputs_and_recurrent_state_match_uncached(self):
        for h in (0, 1):
            with self.subTest(h=h):
                candidate = frozen_hypothesis()
                cached = arm(make(candidate, h_hypothesis=h))
                old = arm(make(UncachedCorrection(candidate), h_hypothesis=h))
                for index in range(4):
                    source = fixtures.snapshot(START+index*observer.DT_NS)
                    source['imu']['accel_m_s2'][0] = index*.001
                    source['source_flags'] = {'nested': ['dynamic', index, -0.0]}
                    self.assertEqual(tree_bits(cached.consume(source)), tree_bits(old.consume(source)))
                    self.assertEqual(tree_bits(cached._policy.calls), tree_bits(old._policy.calls))
                    self.assertEqual(tree_bits(cached._policy.previous), tree_bits(old._policy.previous))
                    for name in ('_next_tick_ns', '_last_sources', 'ticks_completed', 'status'):
                        self.assertEqual(getattr(cached, name), getattr(old, name))
                self.assertEqual(tree_bits(cached.finish()), tree_bits(old.finish()))

    def test_every_returned_tree_is_independent_of_other_ticks_and_cache(self):
        candidate = frozen_hypothesis()
        expected = tree_bits(candidate.provenance())
        run = arm(make(candidate))
        first = run.consume(fixtures.snapshot())
        second = run.consume(fixtures.snapshot(START+observer.DT_NS))
        first_tree = first['provenance']['accel_input_hypothesis']
        second_tree = second['provenance']['accel_input_hypothesis']
        self.assertIsNot(first_tree['nested']['values'], second_tree['nested']['values'])
        first_tree['nested']['values'].clear()
        first_tree['nested']['source_sha256']['source.py'] = 'changed'
        first_tree['bias_sensor_m_s2'][0] = 999
        third = run.consume(fixtures.snapshot(START+2*observer.DT_NS))
        self.assertEqual(tree_bits(second_tree), expected)
        self.assertEqual(tree_bits(third['provenance']['accel_input_hypothesis']), expected)
        self.assertEqual(tree_bits(candidate.provenance()), expected)

    def test_correct_is_called_on_every_tick_and_changed_proof_is_rejected(self):
        candidate = frozen_hypothesis()
        run = arm(make(candidate))
        original, calls = hypotheses.AccelInputHypothesis.correct, []
        def correct(instance, raw):
            calls.append((instance, tuple(raw)))
            return original(instance, raw)
        with patch.object(hypotheses.AccelInputHypothesis, 'correct', correct):
            run.consume(fixtures.snapshot())
            object.__setattr__(candidate, '_proof', None)
            with self.assertRaisesRegex(ValueError, 'loader proof'):
                run.consume(fixtures.snapshot(START+observer.DT_NS))
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(instance is candidate for instance, _ in calls))
        self.assertEqual((run.ticks_completed, len(run._policy.calls), run.status), (1, 1, 'INCOMPLETE'))

    def test_raw_corrected_norm_source_joint_and_model_failures_match_old_path(self):
        cases = [
            ('raw_norm', lambda source: source['imu'].update(accel_m_s2=[0., 0., -8.])),
            ('corrected_norm', lambda source: source['imu'].update(accel_m_s2=[0., 0., -10.7])),
            ('nonfinite', lambda source: source['imu'].update(accel_m_s2=[float('nan'), 0., -10.])),
            ('frame', lambda source: source['imu'].update(frame='body')),
            ('correction', lambda source: source['imu'].update(accel_bias_subtracted=True)),
            ('age', lambda source: source.update(oldest_observation_age_ns=999)),
            ('joint', lambda source: source['motors'][0].update(value=999)),
            ('output', lambda source: None)]
        for label, mutate in cases:
            with self.subTest(label=label):
                outcomes = []
                for correction in (frozen_hypothesis(), UncachedCorrection(frozen_hypothesis())):
                    policy = fixtures.Policy('target_bounds' if label == 'output' else None)
                    run = arm(make(correction, policy=policy))
                    source = fixtures.snapshot(); mutate(source)
                    with self.assertRaises(ValueError) as error:
                        run.consume(source)
                    outcomes.append((type(error.exception).__name__, str(error.exception), run.status,
                        run.ticks_completed, tree_bits(policy.calls), tree_bits(policy.previous),
                        run._next_tick_ns, run.failure))
                self.assertEqual(*outcomes)

    def test_custom_and_subclass_provenance_remain_dynamic(self):
        class Dynamic(UncachedCorrection):
            def provenance(self):
                result = super().provenance()
                result['dynamic_call'] = self.provenance_calls
                return result
        class Subclass(hypotheses.AccelInputHypothesis):
            def provenance(self):
                return {**super().provenance(), 'subclass': True}
        for candidate in (Dynamic(frozen_hypothesis()), Subclass(**vars(frozen_hypothesis()))):
            with self.subTest(kind=type(candidate).__name__):
                run = make(candidate)
                self.assertIsNone(run._accel_provenance_cache)
                if isinstance(candidate, Dynamic): self.assertEqual(candidate.provenance_calls, 0)
                arm(run)
                for index in range(2):
                    result = run.consume(fixtures.snapshot(START+index*observer.DT_NS))
                    if isinstance(candidate, Dynamic):
                        self.assertEqual(result['provenance']['accel_input_hypothesis']['dynamic_call'],
                                         candidate.provenance_calls)
                    else: self.assertTrue(result['provenance']['accel_input_hypothesis']['subclass'])

    def test_replaced_exact_or_custom_object_never_receives_previous_cached_tree(self):
        for custom in (False, True):
            with self.subTest(custom=custom):
                run = arm(make(frozen_hypothesis()))
                data = frozen_hypothesis().provenance(); data['replacement'] = True
                replacement = frozen_hypothesis(_provenance_json=json.dumps(data))
                if custom: replacement = UncachedCorrection(replacement)
                run._accel_calibration = replacement
                result = run.consume(fixtures.snapshot())
                self.assertTrue(result['provenance']['accel_input_hypothesis']['replacement'])
                if custom: self.assertEqual(replacement.provenance_calls, 1)

    def test_provenance_classmethod_change_before_or_after_setup_keeps_old_behavior(self):
        original = hypotheses.AccelInputHypothesis.provenance
        calls = []
        def changed(candidate):
            calls.append(candidate)
            return {**original(candidate), 'changed_method': len(calls)}
        for before in (False, True):
            with self.subTest(before=before):
                candidate = frozen_hypothesis()
                if before:
                    with patch.object(hypotheses.AccelInputHypothesis, 'provenance', changed):
                        run = arm(make(candidate))
                        self.assertIsNone(run._accel_provenance_cache)
                        result = run.consume(fixtures.snapshot())
                else:
                    run = arm(make(candidate))
                    with patch.object(hypotheses.AccelInputHypothesis, 'provenance', changed):
                        result = run.consume(fixtures.snapshot())
                self.assertEqual(result['provenance']['accel_input_hypothesis']['changed_method'], len(calls))

    def test_instance_method_override_falls_back(self):
        candidate = frozen_hypothesis(); run = arm(make(candidate))
        object.__setattr__(candidate, 'provenance', lambda: {'instance_override': True})
        result = run.consume(fixtures.snapshot())
        self.assertEqual(result['provenance']['accel_input_hypothesis'], {'instance_override': True})

    def test_json_replacement_and_parse_errors_do_not_use_old_cache(self):
        for raw in ('{"changed_json":true,"negative_zero":-0.0}',
                    '{"duplicate":1,"duplicate":2}', '{"bad":NaN}', '{"bad":1e999}', '{'):
            with self.subTest(raw=raw):
                candidate = frozen_hypothesis(); run = arm(make(candidate))
                object.__setattr__(candidate, '_provenance_json', raw)
                if 'changed_json' in raw:
                    result = run.consume(fixtures.snapshot())
                    self.assertEqual(tree_bits(result['provenance']['accel_input_hypothesis']),
                                     tree_bits(hypotheses.strict_json(raw)))
                else:
                    with self.assertRaises(ValueError): run.consume(fixtures.snapshot())
                    self.assertEqual((run.ticks_completed, len(run._policy.calls), run.status), (0, 0, 'INCOMPLETE'))

    def test_replaced_parser_keeps_dynamic_errors_before_or_after_setup(self):
        candidate = frozen_hypothesis()
        with patch.object(hypotheses, 'strict_json', wraps=hypotheses.strict_json):
            run = make(candidate)
            self.assertIsNone(run._accel_provenance_cache)
        run = arm(make(candidate))
        with patch.object(hypotheses, 'strict_json', side_effect=ValueError('parser changed')):
            with self.assertRaisesRegex(ValueError, 'parser changed'): run.consume(fixtures.snapshot())
        self.assertEqual(len(run._policy.calls), 0)

    def test_invalid_or_unbounded_setup_json_keeps_original_dynamic_contract(self):
        deep = 0
        for _ in range(26): deep = [deep]
        for raw in ('{"duplicate":1,"duplicate":2}',
                    json.dumps({'deep': deep})):
            with self.subTest(raw=raw[:30]):
                candidate = frozen_hypothesis(_provenance_json=raw)
                run = make(candidate)
                self.assertIsNone(run._accel_provenance_cache)
                if 'duplicate' in raw:
                    with self.assertRaisesRegex(ValueError, 'Duplicate JSON key'):
                        run.prepare_run(warmup_completed=True)
                    self.assertEqual(run.reset_count, 1)  # Error is still in the requested summary.
                else:
                    arm(run)
                    result = run.consume(fixtures.snapshot())
                    self.assertEqual(tree_bits(result['provenance']['accel_input_hypothesis']),
                                     tree_bits(candidate.provenance()))

    def test_class_binding_replacement_never_turns_a_subclass_into_cached_exact_type(self):
        class Subclass(hypotheses.AccelInputHypothesis):
            pass
        candidate = frozen_hypothesis()
        run = arm(make(candidate))
        with patch.object(hypotheses, 'AccelInputHypothesis', Subclass):
            replacement = Subclass(**vars(candidate))
            self.assertIsNone(make(replacement)._accel_provenance_cache)
            with patch.object(hypotheses, 'strict_json', wraps=hypotheses.strict_json) as parse:
                result = run.consume(fixtures.snapshot())
            self.assertEqual(parse.call_count, 1)
        self.assertEqual(tree_bits(result['provenance']['accel_input_hypothesis']),
                         tree_bits(candidate.provenance()))

    def test_snapshot_digest_remains_dynamic_and_exact_each_tick(self):
        run = arm(make(frozen_hypothesis()))
        hashes = []
        for index in range(3):
            source = fixtures.snapshot(START+index*observer.DT_NS)
            source['source_flags'] = {'changing': index}
            expected = observer._digest(source)
            with patch.object(observer, '_digest', wraps=observer._digest) as digest:
                result = run.consume(source)
            self.assertEqual(digest.call_count, 1)
            self.assertEqual(result['provenance']['snapshot_canonical_json_sha256'], expected)
            self.assertIsNot(digest.call_args.args[0], source)
            hashes.append(expected)
        self.assertEqual(len(set(hashes)), 3)

    def test_unselected_and_reviewed_corrections_do_not_populate_cache(self):
        run = fixtures.make()
        self.assertIsNone(run._accel_provenance_cache)
        arm(run)
        self.assertNotIn('accel_input_hypothesis', run.consume(fixtures.snapshot())['provenance'])
        correction = UncachedCorrection(frozen_hypothesis())
        with patch.object(observer, 'reviewed_acceleration', return_value=correction):
            run = fixtures.make(apply_reviewed_accel_calibration=True)
        self.assertIsNone(run._accel_provenance_cache)
        arm(run)
        result = run.consume(fixtures.snapshot())
        self.assertIn('reviewed_accel_calibration', result['provenance'])
        self.assertEqual(correction.provenance_calls, 1)


if __name__ == '__main__':
    unittest.main()
