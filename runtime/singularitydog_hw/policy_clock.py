"""Explicit policy clocks for the unchanged 50 Hz and retrained 30 Hz models.

This file has no device, process, or clock-reading side effects. A 30 Hz release
must use the rational epoch schedule, rather than adding a rounded period. The
longer cadence does not enlarge the 20 ms input-to-command freshness budget or
the independently reviewed host and actuator watchdog limits.
"""

from dataclasses import dataclass
from collections import deque
import math
from types import MappingProxyType
from typing import Mapping


CONTROL_CLOCK_SCHEMA = "singularitydog.policy-control-clock.v1"
CONTROL_CLOCK_KEY = "control_clock"
RELEASE_RULE = "epoch_plus_floor_index_rational_ns"
NANOSECONDS_PER_SECOND = 1_000_000_000
SUPPORTED_POLICY_HZ = (50, 30)
INPUT_TX_BUDGET_NS = 20_000_000
CADENCE_JITTER_NS = 1_000_000
EARLY_RELEASE_JITTER_NS = 5_000_000
HOST_WATCHDOG_NS = 40_000_000
MOTOR_WATCHDOG_NS = 200_000_000


def _integer(value, name, *, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError(f"Policy clock {name} must be an integer >= {minimum}")
    return value


@dataclass(frozen=True)
class PolicyClock:
    """Immutable rational cadence; all limits are derived from a supported rate.

    ``cycle_budget_ns`` is an elapsed-work upper bound, rounded upward by less
    than one nanosecond at 30 Hz. ``slot_deadline_ns`` is the next exact release
    in the epoch schedule. Those are deliberately distinct deadlines.
    """

    policy_hz: int = 50

    def __post_init__(self):
        if type(self.policy_hz) is not int or self.policy_hz not in SUPPORTED_POLICY_HZ:
            raise ValueError("Policy clock policy_hz must be exactly 50 or 30")

    @property
    def model_dt_s(self):
        return 1.0 / self.policy_hz

    @property
    def period_s(self):
        return self.model_dt_s

    @property
    def period_ms(self):
        return 1000.0 / self.policy_hz

    @property
    def period_numerator_ns(self):
        return NANOSECONDS_PER_SECOND

    @property
    def period_denominator(self):
        return self.policy_hz

    @property
    def cycle_budget_ns(self):
        return (NANOSECONDS_PER_SECOND + self.policy_hz - 1) // self.policy_hz

    @property
    def freshness_budget_ns(self):
        return INPUT_TX_BUDGET_NS

    @property
    def inference_tx_budget_ns(self):
        return INPUT_TX_BUDGET_NS

    @property
    def cadence_gap_ms(self):
        return self.period_ms + CADENCE_JITTER_NS / 1_000_000

    @property
    def command_gap_budget_ns(self):
        return self.cycle_budget_ns + CADENCE_JITTER_NS

    @property
    def minimum_start_separation_ns(self):
        # Preserve the old 50 Hz scheduler's 15 ms minimum separation, and its
        # same 5 ms early-release tolerance when the cadence is explicitly 30 Hz.
        return self.cycle_budget_ns - EARLY_RELEASE_JITTER_NS

    @property
    def host_watchdog_ns(self):
        return HOST_WATCHDOG_NS

    @property
    def motor_watchdog_ns(self):
        return MOTOR_WATCHDOG_NS

    def release_ns(self, epoch_ns, index):
        """Return epoch + floor(index * 1e9 / Hz), without accumulated drift."""
        _integer(epoch_ns, "epoch_ns")
        _integer(index, "index")
        return epoch_ns + index * NANOSECONDS_PER_SECOND // self.policy_hz

    def slot_deadline_ns(self, epoch_ns, index):
        """Return the exact end of this scheduled slot, not an elapsed budget."""
        _integer(index, "index")
        return self.release_ns(epoch_ns, index + 1)

    def cycle_deadline_ns(self, begin_ns):
        _integer(begin_ns, "begin_ns")
        return begin_ns + self.cycle_budget_ns

    def input_tx_deadline_ns(self, begin_ns, oldest_input_start_ns):
        """Keep both release-to-TX and oldest-input-to-TX limits at 20 ms."""
        _integer(begin_ns, "begin_ns")
        _integer(oldest_input_start_ns, "oldest_input_start_ns")
        return min(begin_ns, oldest_input_start_ns) + INPUT_TX_BUDGET_NS

    def select_slot(self, epoch_ns, previous_slot, previous_begin_ns, now_ns):
        """Select one new slot; identify skips instead of replaying missed work.

        The caller must retain and handle a skipped slot explicitly. This
        selector neither executes catch-up cycles nor certifies a skip as safe.
        A selected release can be in the future to maintain minimum separation.
        """
        _integer(epoch_ns, "epoch_ns")
        _integer(now_ns, "now_ns")
        if now_ns < epoch_ns or (previous_slot is None) != (previous_begin_ns is None):
            raise ValueError("Invalid policy absolute-epoch schedule state")
        if previous_slot is None:
            minimum_slot = spaced_slot = 0
        else:
            _integer(previous_slot, "previous_slot")
            _integer(previous_begin_ns, "previous_begin_ns")
            if previous_begin_ns < self.release_ns(epoch_ns, previous_slot) or now_ns < previous_begin_ns:
                raise ValueError("Noncausal policy absolute-epoch schedule state")
            minimum_slot = previous_slot + 1
            earliest_delta = previous_begin_ns + self.minimum_start_separation_ns - epoch_ns
            spaced_slot = (earliest_delta * self.policy_hz + NANOSECONDS_PER_SECOND - 1) // NANOSECONDS_PER_SECOND
        # Invert floor(index*N/Hz) exactly. floor(delta*Hz/N) is wrong at the
        # first 30 Hz boundary: release(1) is 33,333,333 ns, not 33,333,334 ns.
        elapsed_slot = ((now_ns - epoch_ns + 1) * self.policy_hz - 1) // NANOSECONDS_PER_SECOND
        slot = max(minimum_slot, elapsed_slot, spaced_slot)
        return slot, self.release_ns(epoch_ns, slot)

    def as_payload(self):
        return {
            "schema": CONTROL_CLOCK_SCHEMA,
            "policy_hz": self.policy_hz,
            "model_dt_s": self.model_dt_s,
            "period_numerator_ns": NANOSECONDS_PER_SECOND,
            "period_denominator": self.policy_hz,
            "release_rule": RELEASE_RULE,
            "cycle_budget_ns": self.cycle_budget_ns,
            "input_freshness_budget_ns": INPUT_TX_BUDGET_NS,
            "inference_tx_budget_ns": INPUT_TX_BUDGET_NS,
            "cadence_jitter_ns": CADENCE_JITTER_NS,
            "command_gap_budget_ns": self.command_gap_budget_ns,
            "minimum_start_separation_ns": self.minimum_start_separation_ns,
            "host_watchdog_ns": HOST_WATCHDOG_NS,
            "motor_watchdog_ns": MOTOR_WATCHDOG_NS,
        }


DEFAULT_POLICY_CLOCK = PolicyClock(50)


def control_clock_payload(policy_hz=50):
    """Construct the explicit reviewed payload, without changing any profile."""
    return PolicyClock(policy_hz).as_payload()


def validate_control_clock(payload):
    """Parse only a complete explicit contract; reject extra or altered fields."""
    if not isinstance(payload, Mapping):
        raise ValueError("Policy control_clock must be a mapping")
    clock = PolicyClock(payload.get("policy_hz"))
    expected = clock.as_payload()
    if set(payload) != set(expected):
        raise ValueError("Policy control_clock keys differ from the complete contract")
    for name, value in expected.items():
        actual = payload[name]
        if type(actual) is not type(value):
            raise ValueError(f"Policy control_clock {name} has the wrong type")
        if type(actual) is float and not math.isfinite(actual):
            raise ValueError(f"Policy control_clock {name} must be finite")
        if actual != value:
            raise ValueError(f"Policy control_clock {name} differs from the reviewed contract")
    return clock


def policy_clock_settings(profile):
    """An absent clock preserves existing 50 Hz; an explicit null is invalid."""
    if not isinstance(profile, Mapping):
        raise ValueError("Policy profile must be a mapping")
    if CONTROL_CLOCK_KEY not in profile:
        return DEFAULT_POLICY_CLOCK
    return validate_control_clock(profile[CONTROL_CLOCK_KEY])


class Pure30Budget:
    """30 Hz post-reply bookkeeping budget, with a separate hard 20 ms TX gate.

    The caller still validates all output replies and the physical envelope.
    Replies can arrive within the explicitly selected 30 Hz cycle. Once those
    replies are received, this class does not treat the already consumed input
    as the next cycle's input. Its age at the final command write remains a hard
    20 ms limit; the returned feedback's own age remains a hard 20 ms limit.
    Missing replies, stale output feedback and late TX have no allowance.
    """

    def __init__(self, settings, clock):
        if type(clock) is not PolicyClock or clock.policy_hz != 30:
            raise ValueError("Pure30Budget requires the explicit 30 Hz PolicyClock")
        expected_keys = {
            "mode", "max_lateness_ms", "max_consecutive_misses",
            "rolling_window_cycles", "max_misses_per_window",
        }
        if type(settings) is not dict or set(settings) != expected_keys:
            raise ValueError("Invalid 30 Hz post-reply settings")
        if settings["mode"] != "bounded_post_reply_v1":
            raise ValueError("30 Hz requires bounded_post_reply_v1 settings")
        lateness = settings["max_lateness_ms"]
        if type(lateness) not in (int, float) or not math.isfinite(lateness) or not 0 < lateness <= 1:
            raise ValueError("30 Hz post-reply lateness must be finite and <= 1 ms")
        for name, expected in (("max_consecutive_misses", 1),
                               ("rolling_window_cycles", 100),
                               ("max_misses_per_window", 1)):
            if type(settings[name]) is not int or settings[name] != expected:
                raise ValueError(f"Invalid 30 Hz post-reply {name}")
        self._settings = MappingProxyType(dict(settings))
        self._clock = clock
        self.misses = deque()
        self.consecutive = 0
        self.previous_index = -1
        self.accepted_misses = 0

    @property
    def settings(self):
        return self._settings

    @property
    def clock(self):
        return self._clock

    def admit(self, *, index, begin_ns, oldest_input_ns, final_write_ns,
              last_reply_ns, output_sample_start_ns, checked_ns, sample_age_ns,
              startup_allowed=False):
        def need(condition, reason):
            if not condition:
                raise RuntimeError("30 Hz post-reply timing: " + reason)

        stamps = (index, begin_ns, oldest_input_ns, final_write_ns,
                  last_reply_ns, output_sample_start_ns, checked_ns, sample_age_ns)
        need(all(type(v) is int for v in stamps) and index >= 0 and
             0 < sample_age_ns <= INPUT_TX_BUDGET_NS,
             "invalid timestamp or sample-age limit")
        need(index == self.previous_index + 1, "nonsequential cycle")
        need(type(startup_allowed) is bool and (not startup_allowed or index == 0),
             "invalid startup allowance")
        need(0 < begin_ns <= oldest_input_ns <= final_write_ns <= last_reply_ns <= checked_ns,
             "noncausal timestamps")
        need(oldest_input_ns <= output_sample_start_ns <= last_reply_ns,
             "noncausal output feedback")
        prefix_deadline = self.clock.input_tx_deadline_ns(begin_ns, oldest_input_ns)
        cycle_deadline = self.clock.cycle_deadline_ns(begin_ns)
        need(final_write_ns <= prefix_deadline and final_write_ns - oldest_input_ns <= sample_age_ns,
             "output write exceeded hard20ms input-to-TX budget")
        need(last_reply_ns <= cycle_deadline,
             "output reply exceeded the 30 Hz cycle budget")
        need(checked_ns - output_sample_start_ns <= sample_age_ns,
             "output feedback exceeded sample-age deadline")
        lateness_ns = max(0, checked_ns - cycle_deadline)
        need(lateness_ns <= int(self.settings["max_lateness_ms"] * 1e6),
             "coordinator lateness exceeded bound")
        missed = lateness_ns > 0
        startup_missed = missed and startup_allowed
        steady_missed = missed and not startup_missed
        consecutive = self.consecutive + 1 if steady_missed else 0
        need(consecutive <= self.settings["max_consecutive_misses"],
             "consecutive miss budget exceeded")
        current_misses = tuple(i for i in self.misses
                               if i > index - self.settings["rolling_window_cycles"])
        need(len(current_misses) + int(steady_missed) <= self.settings["max_misses_per_window"],
             "rolling miss budget exceeded")
        # Commit counters only once the entire cycle has been admitted.
        self.previous_index = index
        self.consecutive = consecutive
        self.misses = deque(current_misses)
        if steady_missed:
            self.misses.append(index)
            self.accepted_misses += 1
        return {
            "schema": "singularitydog.policy-clock-deadline-decision.v1",
            "mode": self.settings["mode"], "policy_hz": 30,
            "accepted": True, "checked_ns": checked_ns,
            "lateness_ms": lateness_ns / 1e6,
            "allowance_used": steady_missed, "startup_allowance_used": startup_missed,
            "consecutive_misses": consecutive, "rolling_misses": len(self.misses),
            "cycle_budget_ns": self.clock.cycle_budget_ns,
            "cycle_deadline_ns": cycle_deadline,
            "input_to_tx_budget_ns": INPUT_TX_BUDGET_NS,
            "input_to_tx_deadline_ns": prefix_deadline,
            "input_to_tx_20ms_met": True,
            "reply_within_cycle_budget": True,
            "cycle_deadline_missed": missed,
            "input_sample_age_at_tx_ms": (final_write_ns - oldest_input_ns) / 1e6,
            "completed_input_age_at_post_reply_ms": (checked_ns - oldest_input_ns) / 1e6,
            "completed_input_not_reused": True,
            "output_feedback_sample_age_ms": (checked_ns - output_sample_start_ns) / 1e6,
            "output_feedback_sample_age_limit_ns": sample_age_ns,
            "reply_elapsed_ms": (last_reply_ns - begin_ns) / 1e6,
        }
