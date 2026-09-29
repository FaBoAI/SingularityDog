"""Bounded post-reply bookkeeping tolerance, never an I/O timeout extension.

The caller must first validate every output reply and physical envelope. This
budget only decides whether completed coordinator work may cross the nominal
period. It grants no permission to send a late command or reuse old feedback.
"""
from collections import deque

PERIOD_NS = 20_000_000
POST_REPLY_POLICY = 'bounded_post_reply_v1'


class PostReplyDeadlineBudget:
    def __init__(self, settings):
        self.settings = dict(settings)
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

        need(index == self.previous_index+1, 'nonsequential cycle')
        need(type(startup_allowed) is bool and (not startup_allowed or index == 0),
             'invalid startup allowance')
        need(0 < begin_ns <= oldest_input_ns <= final_write_ns <= last_reply_ns <= checked_ns,
             'noncausal timestamps')
        need(oldest_input_ns <= output_sample_start_ns <= last_reply_ns,
             'noncausal output feedback')
        # These are unchanged hard limits, including the oldest IMU/input.
        need(final_write_ns <= begin_ns+PERIOD_NS and last_reply_ns <= begin_ns+PERIOD_NS and
             final_write_ns-oldest_input_ns <= sample_age_ns,
             'output write/reply exceeded hard20ms')
        need(checked_ns-oldest_input_ns <= sample_age_ns and
             checked_ns-output_sample_start_ns <= sample_age_ns,
             'input or output feedback exceeded sample-age deadline')
        lateness_ns = max(0, checked_ns-begin_ns-PERIOD_NS)
        need(lateness_ns <= int(self.settings['max_lateness_ms']*1e6),
             'coordinator lateness exceeded bound')
        missed = lateness_ns > 0
        # The separately reviewed first-cycle allowance does not consume the
        # one-in-100 steady-cycle budget. All hard output and age checks above
        # still apply to that first cycle.
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
        return {'accepted': True, 'checked_ns': checked_ns, 'lateness_ms': lateness_ns/1e6,
                'allowance_used': steady_missed, 'startup_allowance_used': startup_missed,
                'consecutive_misses': consecutive,
                'rolling_misses': len(self.misses)}
