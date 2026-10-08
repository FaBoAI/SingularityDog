"""Default-off, file-only queue topology experiment; no runtime selector.

Each owner has one persistent ThreadPoolExecutor worker. Submission returns
that executor's genuine Future, without wrapping the callable or its result.
Existing tasks and collectors remain responsible for absolute deadlines,
cancellation, raw evidence, validation and device lifetime. This module imports
no hardware/model code and neither admits output nor claims a timing gain.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import argparse
import json
import threading
from types import MappingProxyType


OWNERS = ('front', 'rear', 'imu_validation')
SCHEMA = 'singularitydog.experimental-diagnostic-dedicated-owners.v1'


@dataclass(frozen=True)
class Submission:
    owner: str
    deadline_ns: int
    label: str
    future: object


def plan(*, enabled=False):
    if type(enabled) is not bool:
        raise ValueError('Experiment selection must be boolean')
    return {
        'schema': SCHEMA, 'status': 'PLAN_ONLY', 'selected': enabled,
        'proposed_owner_mapping': {name: name for name in OWNERS},
        'proposed_workers_per_owner': {name: 1 for name in OWNERS},
        'proposed_total_worker_count': 3, 'workers_started': 0,
        'hardware_opened': False, 'model_loaded': False,
        'live_activation': False, 'output_allowed': False,
        'approved_for_runtime': False, 'whole_loop_timing_qualified': False,
        'os_or_gil_cause_proven': False, 'timing_speedup_proven': False,
    }


def _worker_identity(owner):
    thread = threading.current_thread()
    return owner, thread, threading.get_native_id()


class DedicatedOwners:
    """Main-caller-owned three queues, explicitly enabled and prestarted.

    The deadline metadata records the caller's original absolute deadline; it
    is not a new wait budget. Task args/kwargs are forwarded unchanged, so the
    original task must still receive/enforce its original deadline. No result
    is admitted by this scheduler. Initializers own rollback on setup failure.
    Optional finalizers run on their same owner thread after queued tasks have
    settled, before shutdown joins all three workers.
    """

    def __init__(self, *, enabled=False, worker_initializer=None,
                 worker_finalizer=None):
        if type(enabled) is not bool:
            raise ValueError('Experiment selection must be boolean')
        for callback in (worker_initializer, worker_finalizer):
            if callback is not None and not callable(callback):
                raise ValueError('Worker lifecycle callback must be callable')
        self.enabled = enabled
        self._caller = threading.current_thread()
        self._initializer = worker_initializer
        self._finalizer = worker_finalizer
        self._state = 'NEW'
        self._pools = {}
        self._workers = {}
        self._submissions = {}
        self._submitted_counts = {owner: 0 for owner in OWNERS}
        self._finalized = set()

    def _caller_check(self):
        if threading.current_thread() is not self._caller:
            raise RuntimeError('Original scheduler caller required')

    def _initialize_owner(self, owner):
        if self._initializer is not None:
            self._initializer(owner)

    def start(self):
        self._caller_check()
        if not self.enabled:
            raise RuntimeError('Explicit file-only experiment selection required')
        if self._state != 'NEW':
            raise RuntimeError('Fresh scheduler required')
        self._state = 'STARTING'
        try:
            for owner in OWNERS:
                pool = ThreadPoolExecutor(max_workers=1,
                    thread_name_prefix='diagnostic-dedicated-'+owner,
                    initializer=self._initialize_owner, initargs=(owner,))
                self._pools[owner] = pool
                name, thread, native_id = pool.submit(_worker_identity, owner).result()
                if name != owner or any(thread is row[0] for row in self._workers.values()):
                    raise RuntimeError('Distinct actual owner workers required')
                self._workers[owner] = (thread, native_id)
            self._state = 'RUNNING'
            return self
        except BaseException as primary:
            try:
                self._finish(cancel_pending=True)
            except BaseException as cleanup:
                primary.add_note('Owner startup cleanup: '+repr(cleanup))
            raise

    def submit(self, owner, fn, /, *args, task_deadline_ns, task_label, **kwargs):
        self._caller_check()
        if self._state != 'RUNNING':
            raise RuntimeError('Running dedicated owners required')
        if owner not in OWNERS:
            raise ValueError('Explicit front/rear/imu_validation owner required')
        if (type(task_deadline_ns) is not int or task_deadline_ns <= 0 or
                type(task_label) is not str or not task_label or not callable(fn)):
            raise ValueError('Original absolute deadline, task label and callable required')
        # Crucially, no callable/result/Future adapter is inserted here.
        future = self._pools[owner].submit(fn, *args, **kwargs)
        self._submissions[future] = Submission(owner, task_deadline_ns, task_label, future)
        self._submitted_counts[owner] += 1
        return future

    def origin(self, future):
        self._caller_check()
        try:
            return self._submissions[future]
        except (KeyError, TypeError) as error:
            raise ValueError('Genuine submitted owner Future required') from error

    def release_settled(self, futures):
        """Retire identity records after the caller retains its original evidence.

        This is not validation/admission, a callback fence, or owner reuse.
        The caller first performs all existing joins/validation/raw journaling.
        Prevalidation is atomic: no record is removed if any Future is foreign,
        repeated or pending. Original Future objects/results are not modified.
        """
        self._caller_check()
        originals = tuple(futures)
        if len({id(future) for future in originals}) != len(originals):
            raise ValueError('Distinct genuine submitted Futures required')
        rows = tuple(self.origin(future) for future in originals)
        if any(not row.future.done() for row in rows):
            raise RuntimeError('Settle all original owners before retiring identity records')
        for future in originals:
            del self._submissions[future]
        return len(originals)

    def cancel_pending(self):
        """Cancel only queued genuine Futures; running native owners are untouched.

        The caller must separately signal original cancellation/gates and settle
        raw evidence. This never closes a device/session or invents a STOP task.
        """
        self._caller_check()
        if self._state != 'RUNNING':
            raise RuntimeError('Running dedicated owners required')
        return tuple(row for future, row in self._submissions.items() if future.cancel())

    def topology(self):
        self._caller_check()
        return {
            'schema': SCHEMA, 'selected': self.enabled, 'state': self._state,
            'owner_mapping': {name: name for name in OWNERS},
            'workers_per_owner': {name: 1 for name in self._pools},
            'actual_workers_started': len(self._workers),
            'actual_workers_alive': sum(row[0].is_alive() for row in self._workers.values()),
            'native_thread_id_by_owner': {name: row[1] for name, row in self._workers.items()},
            'queue_topology': 'three_separate_single_worker_queues',
            'genuine_executor_futures': True,
            'submitted_tasks_by_owner': dict(self._submitted_counts),
            'retained_identity_records_by_owner': {
                name: sum(row.owner == name for row in self._submissions.values()) for name in OWNERS},
            'finalized_owners': sorted(self._finalized),
            'deadline_handling': 'unchanged_task_and_collector_contracts',
            'hardware_access': 'not_provided_by_this_module',
            'live_activation': False, 'output_allowed': False,
            'approved_for_runtime': False, 'whole_loop_timing_qualified': False,
            'os_or_gil_cause_proven': False, 'timing_speedup_proven': False,
        }

    @property
    def worker_threads(self):
        self._caller_check()
        return MappingProxyType({name: row[0] for name, row in self._workers.items()})

    def _finish(self, *, cancel_pending):
        self._state = 'CLOSING'
        errors = []
        if cancel_pending:
            for future in self._submissions:
                future.cancel()
        finalizers = {}
        if self._finalizer is not None:
            for owner in self._workers:
                try:
                    finalizers[owner] = self._pools[owner].submit(self._finalizer, owner)
                except BaseException as error:
                    errors.append(error)
            for owner, future in finalizers.items():
                try:
                    future.result()
                    self._finalized.add(owner)
                except BaseException as error:
                    errors.append(error)
        for pool in self._pools.values():
            try:
                # Finalizer tasks must not be discarded by queued-task cancellation.
                pool.shutdown(wait=True, cancel_futures=False)
            except BaseException as error:
                errors.append(error)
        self._state = 'CLOSED'
        if errors:
            primary = errors[0]
            for extra in errors[1:]:
                primary.add_note('Additional owner cleanup error: '+repr(extra))
            raise primary

    def close(self, *, cancel_pending=False):
        self._caller_check()
        if type(cancel_pending) is not bool:
            raise ValueError('Queued cancellation selection must be boolean')
        if self._state == 'CLOSED':
            return
        self._finish(cancel_pending=cancel_pending)

    def __enter__(self):
        return self.start()

    def __exit__(self, kind, error, traceback):
        try:
            self.close()
        except BaseException as cleanup:
            if error is None:
                raise
            error.add_note('Owner context cleanup: '+repr(cleanup))
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dedicated-owners', action='store_true',
                        help='Explicitly select the proposed topology in this file-only PLAN')
    args = parser.parse_args(argv)
    print(json.dumps(plan(enabled=args.dedicated_owners), sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
