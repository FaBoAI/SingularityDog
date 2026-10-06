"""Opt-in timer slack for the active main thread and three I/O owners.

Each thread reads its own original value, applies 1 us, and later restores that
exact value on the same thread. Start the I/O owners before changing the main
thread, so newly created workers do not inherit the temporary main setting.
Call before enable and restore after STOP, before shutting the pools.
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
            "schema": "singularitydog.active-output-timer-slack.v2",
            "scope": "main_thread_and_three_io_workers",
            "enabled": requested_ns is not None,
            "requested_ns": requested_ns,
            "status": "inactive" if requested_ns is None else "not_applied",
            "workers": {},
            "main": None,
            "apply_verified": False,
            "restoration_complete": None,
            "errors": [],
        }
        self._state = "new"
        self._backend = None
        self._pools = None
        self._worker_rows = {}
        self._worker_rows_lock = threading.Lock()
        self._apply_submitted_roles = set()
        self._restore_ack_roles = set()

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
               "restored": None, "errors": [], "restore_errors": []}
        try:
            row["original_ns"] = self._backend.get()
            if type(row["original_ns"]) is not int or row["original_ns"] <= 0:
                raise ActiveTimerSlackError("Exact positive original thread timer slack required")
            row["set_attempted"] = True  # A failed OS call may still have changed state.
            self._backend.set(self.requested_ns)
            row["during_ns"] = self._backend.get()
            if row["during_ns"] != self.requested_ns:
                raise ActiveTimerSlackError("Active thread timer slack readback mismatch")
            row["applied"] = True
        except BaseException as error:
            row["errors"].append(type(error).__name__ + ": " + str(error))
        if role in ROLES:
            # A caller can be interrupted after executor.submit enqueues the
            # job but before it receives its Future. Retain mutation evidence
            # on the owner independently of that Future.
            with self._worker_rows_lock:
                self._worker_rows[role] = row
        return row

    def _restore_registered_worker(self, role):
        # This is queued on the same single-thread pool after any possibly
        # submitted apply. Reaching it is also a barrier proving whether that
        # apply ran, even when the caller never obtained the original Future.
        with self._worker_rows_lock:
            row = self._worker_rows.get(role)
        if row is None:
            return None
        if row["set_attempted"] and row["restored"] is not True:
            row = self._restore_one(row)
            with self._worker_rows_lock:
                self._worker_rows[role] = row
        return row

    def _restore_one(self, row):
        row = dict(row)
        row["errors"] = list(row["errors"])
        row["restore_errors"] = list(row["restore_errors"])
        row["restored"] = False
        def record_error(message):
            row["errors"].append(message)
            row["restore_errors"].append(message)
        if threading.get_native_id() != row["native_tid"]:
            record_error("ActiveTimerSlackError: Thread identity changed before restoration")
            return row
        try:
            try:
                row["before_restore_ns"] = self._backend.get()
            except BaseException as error:
                record_error(type(error).__name__ + ": pre-restore readback: " + str(error))
            if row["applied"] and row["before_restore_ns"] != self.requested_ns:
                record_error("ActiveTimerSlackError: Thread timer slack drifted before restoration")
            self._backend.set(row["original_ns"])
            row["after_ns"] = self._backend.get()
            if row["after_ns"] != row["original_ns"]:
                raise ActiveTimerSlackError("Exact original thread timer slack restoration unconfirmed")
            row["restored"] = True
        except BaseException as error:
            record_error(type(error).__name__ + ": " + str(error))
        return row

    def apply(self, workers=None, imu_pool=None):
        """Configure four threads before enable; roll back every partial setup."""
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
            self._apply_submitted_roles.add(role)
            try:
                futures[role] = self._pools[role].submit(self._apply_one, role)
            except BaseException as error:
                self.report["errors"].append(role + " submit: " + type(error).__name__ + ": " + str(error))
        for role, future in futures.items():
            while True:
                try:
                    self.report["workers"][role] = future.result()
                    break
                except BaseException as error:
                    self.report["errors"].append(role + " result: " + type(error).__name__ + ": " + str(error))
                    if future.done() and (future.cancelled() or future.exception() is not None):
                        break
                    # A signal in the caller does not cancel an in-flight
                    # worker mutation. Reap its row before rolling it back.
        rows = self.report["workers"]
        for role, row in rows.items():
            self.report["errors"].extend(role + " apply: " + message for message in row["errors"])
        workers_verified = (
            len(rows) == 3 and len({row["native_tid"] for row in rows.values()}) == 3
            and all(row["applied"] and not row["errors"] for row in rows.values())
            and not self.report["errors"])
        if workers_verified:
            self.report["main"] = self._apply_one("main")
            self.report["errors"].extend("main apply: " + message
                                         for message in self.report["main"]["errors"])
        main = self.report["main"]
        self.report["apply_verified"] = bool(workers_verified and main is not None
            and main["applied"] and not main["errors"] and not self.report["errors"]
            and main["native_tid"] not in {row["native_tid"] for row in rows.values()})
        if not self.report["apply_verified"]:
            self.report["status"] = "apply_failed"
            self._state = "restore_pending"
            try:
                self.restore()
            except ActiveTimerSlackError:
                pass  # Preserve the original setup failure and rollback evidence.
            raise ActiveTimerSlackError("Three distinct I/O workers and main-thread timer slack readbacks required")
        self.report["status"] = "applied"
        self._state = "active"
        return self.report

    def restore(self):
        """Restore all changed threads, including after a partial apply failure."""
        if self._state in ("inactive", "failed_restored", "restored"):
            return self.report
        if self._state not in ("active", "restore_pending", "restore_failed"):
            raise ActiveTimerSlackError("Active-output timer slack was not applied")
        futures = {}
        for role in ROLES:
            if role not in self._apply_submitted_roles:
                continue
            row = self.report["workers"].get(role)
            if role in self._restore_ack_roles and (row is None or
                    not row["set_attempted"] or row["restored"] is True):
                continue
            try:
                futures[role] = self._pools[role].submit(self._restore_registered_worker, role)
            except BaseException as error:
                self.report["errors"].append(role + " restore submit: " + type(error).__name__ + ": " + str(error))
        for role, future in futures.items():
            while True:
                try:
                    row = future.result()
                    if row is not None:
                        self.report["workers"][role] = row
                    self._restore_ack_roles.add(role)
                    break
                except BaseException as error:
                    self.report["errors"].append(role + " restore result: " + type(error).__name__ + ": " + str(error))
                    if future.done() and (future.cancelled() or future.exception() is not None):
                        break
        main = self.report["main"]
        if main is not None and main["set_attempted"] and main["restored"] is not True:
            self.report["main"] = self._restore_one(main)
        changed = [row for row in self.report["workers"].values() if row["set_attempted"]]
        if self.report["main"] is not None and self.report["main"]["set_attempted"]:
            changed.append(self.report["main"])
        self.report["restoration_complete"] = (
            self._restore_ack_roles == self._apply_submitted_roles
            and all(row["restored"] is True for row in changed)
            and not any(row["restore_errors"] for row in changed)
            and not any("restore" in message.lower() for message in self.report["errors"]))
        if not self.report["restoration_complete"]:
            self.report["status"] = "restore_failed"
            self._state = "restore_failed"
            raise ActiveTimerSlackError("Exact active main/worker timer slack restoration unconfirmed")
        self.report["status"] = "restored" if self.report["apply_verified"] else "apply_failed_restored"
        self._state = "restored" if self.report["apply_verified"] else "failed_restored"
        return self.report
