"""Finite ground-trial command timing; no clock, device access, or motor control.

Use only after ground_trial_plan has validated the physical review and base
profile. This class does not establish contact, load support, balance, or human
readiness. A zero locomotion command does not imply that learned joint targets
or the physical robot have stopped moving.

The caller keeps independent fall protection and all feedback/deadline guards
active throughout. A normal stop decision permits the existing bounded joint
brake/gain-down envelope; an emergency decision requires immediate emergency
STOP and must not wait for resupport. Cue labels are requests, not observations.
"""

from dataclasses import dataclass
import math


STAGES = frozenset(("supported_stance", "partial_load", "stand", "walk"))
MAX_FORWARD_M_S = .05
DECISION_MARGIN_S = .02
SHUTDOWN_RESERVE_MARGIN_S = .04
ZERO_COMMAND = (0., 0., 0.)


class GroundTrajectoryError(ValueError):
    pass


def _number(value, name, minimum=0.):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GroundTrajectoryError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise GroundTrajectoryError(f"{name} must be a finite number") from None
    if not math.isfinite(result) or result < minimum:
        raise GroundTrajectoryError(f"{name} must be finite and >= {minimum}")
    return result


def _quintic(fraction):
    u = min(1., max(0., fraction))
    return u ** 3 * (10. + u * (-15. + 6. * u))


@dataclass(frozen=True)
class GroundCue:
    time_s: float
    key: str
    label: str


@dataclass(frozen=True)
class GroundDecision:
    command: tuple
    phase: str
    cues: tuple
    request_normal_stop: bool
    emergency_stop: bool
    reason: str | None
    resupport_ack_accepted: bool


class GroundTimeline:
    """A command schedule with an explicit, timestamped resupport gate.

    All times are elapsed monotonic seconds from the caller's trial start.
    command_at/cues_between are pure queries. step and request_stop reject
    backwards time. Pass resupport_ack_s only for an actual human confirmation,
    never synthesize it from phase, command, elapsed time, or sensor estimates.

    A graceful early request never prolongs the planned active interval. During
    ramp-up it finishes that ramp before decelerating, preserving smooth joins;
    before activation it skips activation. Ctrl+C/fault handling belongs to the
    immediate emergency path in the caller, not to request_stop.
    """

    def __init__(self, *, stage, duration_s, initial_hold_s, active_duration_s,
                 forward_velocity_m_s, ramp_up_s, ramp_down_s,
                 final_stationary_s, resupport_window_s, shutdown_reserve_s):
        if not isinstance(stage, str) or stage not in STAGES:
            raise GroundTrajectoryError("Unknown ground-trial stage")
        names = ("duration_s", "initial_hold_s", "active_duration_s",
                 "forward_velocity_m_s", "ramp_up_s", "ramp_down_s",
                 "final_stationary_s", "resupport_window_s", "shutdown_reserve_s")
        values = (duration_s, initial_hold_s, active_duration_s,
                  forward_velocity_m_s, ramp_up_s, ramp_down_s,
                  final_stationary_s, resupport_window_s, shutdown_reserve_s)
        v = {name: _number(value, name) for name, value in zip(names, values)}
        if not 0. < v["duration_s"] <= 10. or v["initial_hold_s"] <= 0.:
            raise GroundTrajectoryError("Finite duration <=10s and positive initial hold required")
        minimum_active = 1. if stage == "walk" else .5
        if not minimum_active <= v["active_duration_s"] <= 5.:
            raise GroundTrajectoryError("Active duration outside stage bounds")
        if v["final_stationary_s"] < .5 or v["resupport_window_s"] < 1.:
            raise GroundTrajectoryError("Final zero-command interval >=.5s and resupport window >=1s required")
        if v["shutdown_reserve_s"] <= SHUTDOWN_RESERVE_MARGIN_S:
            raise GroundTrajectoryError("Shutdown reserve must include brake, gain-down and .04s margin")
        if stage == "walk":
            if not 0. < v["forward_velocity_m_s"] <= MAX_FORWARD_M_S:
                raise GroundTrajectoryError("Walk speed must be >0 and <=.05m/s")
            if min(v["ramp_up_s"], v["ramp_down_s"]) < .5:
                raise GroundTrajectoryError("Walk ramps must each be >=.5s")
            if v["ramp_up_s"] + v["ramp_down_s"] > v["active_duration_s"]:
                raise GroundTrajectoryError("Walk ramps exceed active duration")
        elif any(v[name] != 0. for name in ("forward_velocity_m_s", "ramp_up_s", "ramp_down_s")):
            raise GroundTrajectoryError("Non-walk stages require zero command and zero locomotion ramps")
        self.stage = stage
        self._v = v
        self._active_end = v["initial_hold_s"] + v["active_duration_s"]
        self._decel_start = self._active_end - v["ramp_down_s"]
        self._resupport_open = self._active_end + v["final_stationary_s"]
        self._latest_stop = v["duration_s"] - v["shutdown_reserve_s"]
        if self._resupport_open + v["resupport_window_s"] > self._latest_stop + 1e-12:
            raise GroundTrajectoryError("Insufficient total time for resupport and bounded shutdown")
        self._last_t = None
        self._request_t = None
        self._ack = None
        self._normal = False
        self._emergency_reason = None
        self._emitted = set()

    @property
    def timing(self):
        """A new JSON-ready effective schedule; the caller also retains the original plan."""
        return {"stage": self.stage, "duration_s": self._v["duration_s"],
                "active_start_s": self._v["initial_hold_s"],
                "ramp_up_end_s": min(self._active_end, self._v["initial_hold_s"] + self._v["ramp_up_s"]),
                "ramp_down_start_s": self._decel_start,
                "active_end_s": self._active_end,
                "resupport_window_open_s": self._resupport_open,
                "latest_stop_start_s": self._latest_stop,
                "shutdown_reserve_s": self._v["shutdown_reserve_s"],
                "early_stop_requested_s": self._request_t}

    @property
    def resupport_ack_s(self):
        return self._ack

    def _ordered_time(self, t):
        t = _number(t, "elapsed_s")
        if ((self._last_t is not None and t < self._last_t)
                or (self._request_t is not None and t < self._request_t)):
            raise GroundTrajectoryError("Elapsed time moved backwards")
        return t

    def command_at(self, t):
        t = _number(t, "elapsed_s")
        start = self._v["initial_hold_s"]
        if self.stage != "walk" or t <= start or t >= self._active_end:
            return ZERO_COMMAND
        if t < start + self._v["ramp_up_s"]:
            fraction = _quintic((t - start) / self._v["ramp_up_s"])
        elif t >= self._decel_start:
            fraction = 1. - _quintic((t - self._decel_start) / self._v["ramp_down_s"])
        else:
            fraction = 1.
        return (self._v["forward_velocity_m_s"] * fraction, 0., 0.)

    def _cue_schedule(self):
        active = ("active_window_skipped", "Active window was skipped by a graceful stop request") if (
            self._active_end <= self._v["initial_hold_s"]) else (
                "active_window_open", f"Requested {self.stage} observation window; this is not a standing confirmation")
        supported = self.stage == "supported_stance"
        return (GroundCue(0., "initial_hold", "Initial zero-command hold; keep body support"),
                GroundCue(self._v["initial_hold_s"], *active),
                GroundCue(self._active_end, "active_window_close",
                          "Locomotion command is zero; continue guarding the body"),
                GroundCue(self._resupport_open, "resupport_window_open",
                          "Keep full body support throughout bounded gain-down" if supported else
                          "Restore full body support and explicitly confirm; keep independent catch"),
                GroundCue(self._latest_stop, "normal_stop_deadline",
                          "Supported gain-down decision deadline" if supported else
                          "Resupport decision deadline; no confirmation means emergency STOP"))

    def cues_between(self, last_t, t):
        t = _number(t, "elapsed_s")
        if last_t is not None:
            last_t = _number(last_t, "previous_elapsed_s")
            if last_t > t:
                raise GroundTrajectoryError("Cue query time moved backwards")
        return tuple(c for c in self._cue_schedule()
                     if (last_t is None or last_t < c.time_s) and c.time_s <= t)

    def request_stop(self, t):
        """Request a graceful shorter schedule; return the effective timing.

        This never constitutes resupport confirmation. If a complete new
        resupport window cannot fit before the existing deadline, keep the
        already bounded plan. Repeated requests cannot postpone shutdown.
        """
        t = self._ordered_time(t)
        if self._request_t is not None or self._normal or self._emergency_reason:
            return self.timing
        self._request_t = t
        if t >= self._latest_stop:
            return self.timing
        start = self._v["initial_hold_s"]
        active_end, decel_start = self._active_end, self._decel_start
        if t <= start:
            active_end, decel_start = start, start
        elif t < self._active_end:
            if self.stage == "walk":
                decel_start = min(self._decel_start, max(t, start + self._v["ramp_up_s"]))
                active_end = decel_start + self._v["ramp_down_s"]
            else:
                active_end, decel_start = t, t
        resupport_open = max(t, active_end + self._v["final_stationary_s"])
        latest_stop = resupport_open + self._v["resupport_window_s"]
        if latest_stop <= self._latest_stop:
            self._active_end, self._decel_start = active_end, decel_start
            self._resupport_open, self._latest_stop = resupport_open, latest_stop
            # A changed window requires a new human event, never a recycled ack.
            self._ack = None
        return self.timing

    def step(self, t, *, resupport_ack_s=None):
        t = self._ordered_time(t)
        rejection = None
        if resupport_ack_s is not None:
            ack = _number(resupport_ack_s, "resupport_ack_s")
            if ack > t:
                raise GroundTrajectoryError("Resupport acknowledgement is from the future")
            if ack < self._resupport_open:
                rejection = "resupport_ack_before_window"
            elif ack > self._latest_stop:
                rejection = "resupport_ack_after_deadline"
            elif self._ack is None and not self._emergency_reason:
                self._ack = ack
        # Schedule replanning may open a cue at the same time as a prior step.
        cues = tuple(c for c in self._cue_schedule()
                     if c.time_s <= t and (c.key, c.time_s) not in self._emitted)
        self._emitted.update((c.key, c.time_s) for c in cues)
        self._last_t = t
        if not self._normal and not self._emergency_reason and t >= self._latest_stop:
            if t > self._latest_stop + DECISION_MARGIN_S + 1e-12:
                self._emergency_reason = "normal_stop_deadline_missed"
            elif self.stage != "supported_stance" and self._ack is None:
                self._emergency_reason = "resupport_not_confirmed"
            else:
                self._normal = True
        if self._emergency_reason:
            phase, reason = "emergency", self._emergency_reason
        elif self._normal:
            phase, reason = "shutdown_requested", "resupport_confirmed" if self._ack is not None else "support_remains"
        elif t < self._v["initial_hold_s"]:
            phase, reason = "initial_hold", rejection
        elif t < self._active_end:
            phase, reason = "active", rejection
        elif t < self._resupport_open:
            phase, reason = "final_zero_command", rejection
        else:
            phase, reason = "resupport_hold", rejection
        return GroundDecision(ZERO_COMMAND if self._normal or self._emergency_reason else self.command_at(t),
                              phase, cues, self._normal, bool(self._emergency_reason), reason,
                              self._ack is not None)
