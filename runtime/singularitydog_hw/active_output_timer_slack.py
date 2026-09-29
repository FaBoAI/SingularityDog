"""Opt-in timer slack for the three existing active-output I/O owners.

The main thread is unchanged. Each single-thread CAN/IMU executor reads its own
original value, applies 1 us, and later restores that exact value on the same
thread. Call before enable and restore after STOP, before shutting the pools.
Importing this module or selecting the default does not load libc or call prctl.
"""

import threading

from . import thread_timer_slack as base


ROLES = ("front", "rear", "imu")
OPT_IN_NS = 1_000


class ActiveTimerSlackError(base.TimerSlackError):
    """An active-output worker setting or its restoration was not verified."""


class ActiveOutputTimerSlack:
    def __init__(self, requested_ns=None):
        if requested_ns is not None and (type(requested_ns) is not int or requested_ns != OPT_IN_NS):
            raise ValueError("Active-output timer slack must be an explicit 1000 ns selection")
        self.requested_ns = requested_ns
        self.report = {
            "schema": "singularitydog.active-output-timer-slack.v1",
            "scope": "two_bus_owners_and_imu_worker_only",
            "enabled": requested_ns is not None,
            "requested_ns": requested_ns,
            "status": "inactive" if requested_ns is None else "not_applied",
            "workers": {},
            "apply_verified": False,
            "restoration_complete": None,
            "errors": [],
        }
        self._state = "new"
        self._backend = None
        self._pools = None

    @staticmethod
    def _pools_for(workers, imu_pool):
        buses = getattr(workers, "pools", None)
        if type(buses) is not dict or set(buses) != {"front", "rear"}:
            raise ActiveTimerSlackError("Exactly front/rear active bus pools required")
        pools = {"front": buses["front"], "rear": buses["rear"], "imu": imu_pool}
        if len({id(pool) for pool in pools.values()}) != 3 or any(
                getattr(pool, "_max_workers", None) != 1 for pool in pools.values()):
            raise ActiveTimerSlackError("Three distinct single-thread owner pools required")
        return pools

    def _apply_one(self, role):
        row = {"role": role, "native_tid": threading.get_native_id(),
               "original_ns": None, "during_ns": None, "before_restore_ns": None,
               "after_ns": None, "set_attempted": False, "applied": False,
               "restored": None, "errors": []}
        try:
            row["original_ns"] = self._backend.get()
            if type(row["original_ns"]) is not int or row["original_ns"] <= 0:
                raise ActiveTimerSlackError("Exact positive original worker timer slack required")
            row["set_attempted"] = True  # A failed OS call may still have changed state.
            self._backend.set(self.requested_ns)
            row["during_ns"] = self._backend.get()
            if row["during_ns"] != self.requested_ns:
                raise ActiveTimerSlackError("Active worker timer slack readback mismatch")
            row["applied"] = True
        except BaseException as error:
            row["errors"].append(type(error).__name__ + ": " + str(error))
        return row

    def _restore_one(self, row):
        row = dict(row)
        row["errors"] = list(row["errors"])
        row["restored"] = False
        if threading.get_native_id() != row["native_tid"]:
            row["errors"].append("ActiveTimerSlackError: Worker thread identity changed before restoration")
            return row
        try:
            try:
                row["before_restore_ns"] = self._backend.get()
            except BaseException as error:
                row["errors"].append(type(error).__name__ + ": pre-restore readback: " + str(error))
            if row["applied"] and row["before_restore_ns"] != self.requested_ns:
                row["errors"].append("ActiveTimerSlackError: Worker timer slack drifted before restoration")
            self._backend.set(row["original_ns"])
            row["after_ns"] = self._backend.get()
            if row["after_ns"] != row["original_ns"]:
                raise ActiveTimerSlackError("Exact original worker timer slack restoration unconfirmed")
            row["restored"] = True
        except BaseException as error:
            row["errors"].append(type(error).__name__ + ": " + str(error))
        return row

    def apply(self, workers=None, imu_pool=None):
        """Configure front/rear/IMU before enable; roll back every partial setup."""
        if self._state != "new":
            raise ActiveTimerSlackError("Active-output timer slack scope cannot be reused")
        self._state = "applying"
        if self.requested_ns is None:
            self._state = "inactive"
            return self.report
        try:
            self._pools = self._pools_for(workers, imu_pool)
            base.require_supported_platform()
            self._backend = base._load_prctl()
        except BaseException as error:
            self.report["status"] = "apply_failed"
            self.report["errors"].append(type(error).__name__ + ": " + str(error))
            self.report["restoration_complete"] = True  # No setting was attempted.
            self._state = "failed_restored"
            raise

        futures = {}
        for role in ROLES:
            try:
                futures[role] = self._pools[role].submit(self._apply_one, role)
            except BaseException as error:
                self.report["errors"].append(role + " submit: " + type(error).__name__ + ": " + str(error))
        for role, future in futures.items():
            try:
                self.report["workers"][role] = future.result()
            except BaseException as error:
                self.report["errors"].append(role + " result: " + type(error).__name__ + ": " + str(error))
        rows = self.report["workers"]
        for role, row in rows.items():
            self.report["errors"].extend(role + " apply: " + message for message in row["errors"])
        self.report["apply_verified"] = (
            len(rows) == 3 and len({row["native_tid"] for row in rows.values()}) == 3
            and all(row["applied"] and not row["errors"] for row in rows.values())
            and not self.report["errors"])
        if not self.report["apply_verified"]:
            self.report["status"] = "apply_failed"
            self._state = "restore_pending"
            try:
                self.restore()
            except ActiveTimerSlackError:
                pass  # Preserve the original setup failure and rollback evidence.
            raise ActiveTimerSlackError("Three distinct active I/O worker timer slack readbacks required")
        self.report["status"] = "applied"
        self._state = "active"
        return self.report

    def restore(self):
        """Restore all changed workers, including after a partial apply failure."""
        if self._state in ("inactive", "failed_restored", "restored"):
            return self.report
        if self._state not in ("active", "restore_pending", "restore_failed"):
            raise ActiveTimerSlackError("Active-output timer slack was not applied")
        futures = {}
        for role, row in self.report["workers"].items():
            if not row["set_attempted"] or row["restored"] is True:
                continue
            try:
                futures[role] = self._pools[role].submit(self._restore_one, row)
            except BaseException as error:
                self.report["errors"].append(role + " restore submit: " + type(error).__name__ + ": " + str(error))
        for role, future in futures.items():
            try:
                self.report["workers"][role] = future.result()
            except BaseException as error:
                self.report["errors"].append(role + " restore result: " + type(error).__name__ + ": " + str(error))
        changed = [row for row in self.report["workers"].values() if row["set_attempted"]]
        self.report["restoration_complete"] = (
            all(row["restored"] is True for row in changed)
            and not any("restore" in message.lower() or "drifted" in message.lower()
                        for row in changed for message in row["errors"])
            and not any("restore" in message.lower() for message in self.report["errors"]))
        if not self.report["restoration_complete"]:
            self.report["status"] = "restore_failed"
            self._state = "restore_failed"
            raise ActiveTimerSlackError("Exact active I/O worker timer slack restoration unconfirmed")
        self.report["status"] = "restored" if self.report["apply_verified"] else "apply_failed_restored"
        self._state = "restored" if self.report["apply_verified"] else "failed_restored"
        return self.report
