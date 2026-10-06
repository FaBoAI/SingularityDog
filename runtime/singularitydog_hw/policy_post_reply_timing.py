"""Bounded post-reply bookkeeping tolerance, never an I/O timeout extension.

The caller must first validate every output reply and physical envelope. This
budget only decides whether completed coordinator work may cross the nominal
period. It grants no permission to send a late command or reuse old feedback.
"""
from collections import deque
import math

PERIOD_NS = 20_000_000
POST_REPLY_POLICY = 'bounded_post_reply_v1'
POST_REPLY_POLICY_V2 = 'bounded_post_reply_input_age_v2'
POST_REPLY_INPUT_AGE_BUDGET_KEY = 'post_reply_input_age_budget_ms'


def _input_age_v2_settings(settings):
    """Validate only the explicit v2 contract; v1 keeps its original behavior."""
    def need(condition, reason):
        if not condition:
            raise RuntimeError('Post-reply timing: '+reason)
    need(type(settings) is dict and set(settings) == {
        'mode', 'max_lateness_ms', 'max_consecutive_misses',
        'rolling_window_cycles', 'max_misses_per_window', POST_REPLY_INPUT_AGE_BUDGET_KEY},
        'invalid input-age v2 settings')
    need(settings['mode'] == POST_REPLY_POLICY_V2, 'invalid input-age v2 mode')
    for key in ('max_lateness_ms', POST_REPLY_INPUT_AGE_BUDGET_KEY):
        value = settings[key]
        need(type(value) in (int, float) and math.isfinite(value) and 0 < value <= 1.,
             'invalid input-age v2 '+key)
    need(settings[POST_REPLY_INPUT_AGE_BUDGET_KEY] <= settings['max_lateness_ms'],
         'input-age budget exceeds coordinator lateness budget')
    for key, expected in (('max_consecutive_misses', 1), ('rolling_window_cycles', 100),
                          ('max_misses_per_window', 1)):
        need(type(settings[key]) is int and settings[key] == expected,
             'invalid input-age v2 '+key)
    return dict(settings)


class PostReplyDeadlineBudget:
    def __init__(self, settings):
        self.settings = dict(settings)
        self._input_age_v2 = self.settings.get('mode') == POST_REPLY_POLICY_V2
        self._input_age_v2_bound = (_input_age_v2_settings(self.settings)
                                    if self._input_age_v2 else None)
        self.misses = deque()
        self.consecutive = 0
        self.previous_index = -1
        self.accepted_misses = 0

    def admit(self, *, index, begin_ns, oldest_input_ns, final_write_ns,
              last_reply_ns, output_sample_start_ns, checked_ns, sample_age_ns,
              startup_allowed=False):
        def need(condition, reason):
            if not condition:
                raise RuntimeError('Post-reply timing: '+reason)

        input_age_budget_ns = 0
        if self._input_age_v2:
            need(_input_age_v2_settings(self.settings) == self._input_age_v2_bound,
                 'input-age v2 settings changed after construction')
            need(all(type(v) is int for v in (index, begin_ns, oldest_input_ns,
                 final_write_ns, last_reply_ns, output_sample_start_ns, checked_ns, sample_age_ns)) and
                 index >= 0 and 0 < sample_age_ns <= PERIOD_NS,
                 'invalid input-age v2 timestamp or sample-age limit')
            input_age_budget_ns = int(self._input_age_v2_bound[POST_REPLY_INPUT_AGE_BUDGET_KEY]*1e6)
        need(index == self.previous_index+1, 'nonsequential cycle')
        need(type(startup_allowed) is bool and (not startup_allowed or index == 0),
             'invalid startup allowance')
        need(0 < begin_ns <= oldest_input_ns <= final_write_ns <= last_reply_ns <= checked_ns,
             'noncausal timestamps')
        need(oldest_input_ns <= output_sample_start_ns <= last_reply_ns,
             'noncausal output feedback')
        # Every command write and reply retains the original hard limit.
        # V2 extends only a completed-input age check after those replies;
        # it never changes dispatch, I/O or the returned feedback's age limit.
        need(final_write_ns <= begin_ns+PERIOD_NS and last_reply_ns <= begin_ns+PERIOD_NS and
             final_write_ns-oldest_input_ns <= sample_age_ns,
             'output write/reply exceeded hard20ms')
        need(checked_ns-oldest_input_ns <= sample_age_ns+input_age_budget_ns and
             checked_ns-output_sample_start_ns <= sample_age_ns,
             'input or output feedback exceeded sample-age deadline')
        lateness_ns = max(0, checked_ns-begin_ns-PERIOD_NS)
        need(lateness_ns <= int(self.settings['max_lateness_ms']*1e6),
             'coordinator lateness exceeded bound')
        missed = lateness_ns > 0
        # The separately reviewed first-cycle allowance does not consume the
        # one-in-100 steady-cycle budget. All hard output and age checks above
        # still apply to that first cycle, with only the explicit v2 post-reply
        # input-age budget when selected.
        startup_missed = missed and startup_allowed
        steady_missed = missed and not startup_missed
        consecutive = self.consecutive+1 if steady_missed else 0
        need(consecutive <= self.settings['max_consecutive_misses'], 'consecutive miss budget exceeded')
        while self.misses and self.misses[0] <= index-self.settings['rolling_window_cycles']:
            self.misses.popleft()
        need(len(self.misses)+int(steady_missed) <= self.settings['max_misses_per_window'],
             'rolling miss budget exceeded')
        self.previous_index = index
        self.consecutive = consecutive
        if steady_missed:
            self.misses.append(index)
            self.accepted_misses += 1
        decision = {'accepted': True, 'checked_ns': checked_ns, 'lateness_ms': lateness_ns/1e6,
                'allowance_used': steady_missed, 'startup_allowance_used': startup_missed,
                'consecutive_misses': consecutive,
                'rolling_misses': len(self.misses)}
        if self._input_age_v2:
            decision.update(mode=POST_REPLY_POLICY_V2,
                post_reply_input_age_budget_ms=input_age_budget_ns/1e6,
                input_sample_age_ms=(checked_ns-oldest_input_ns)/1e6,
                output_feedback_sample_age_ms=(checked_ns-output_sample_start_ns)/1e6,
                input_age_allowance_used=checked_ns-oldest_input_ns > sample_age_ns,
                pre_send_input_and_native_output_limits_unchanged=True,
                output_feedback_sample_age_limit_unchanged=True)
        return decision
