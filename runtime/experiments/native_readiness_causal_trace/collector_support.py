"""Support for an explicitly generated diagnostic module; not a live selector."""
from pathlib import Path
import hashlib
import importlib.util
import os
import stat
import sys

BASELINE_SHA = '0bafa9b92bea7ea641f57239a5ff1cd4784078b7b5bf09b5c917d0019acd5c93'
CANDIDATE_SHA = 'ab299e1379131b4d9fa90817a4eb9303f695d39abbce25e774c896281dcec63b'


def read_regular(path, limit=256_000):
    path = Path(path)
    if (not path.is_absolute() or '..' in path.parts or
            any(p.is_symlink() for p in (path, *path.parents))):
        raise ValueError('Absolute non-symlink regular source required')
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    with os.fdopen(fd, 'rb') as stream:
        before = os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size > limit:
            raise ValueError('Bounded regular source required')
        raw = stream.read(limit + 1)
        after = os.fstat(stream.fileno())
    if (len(raw) > limit or len(raw) != before.st_size or
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) !=
            (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)):
        raise ValueError('Source changed during bounded read')
    return raw


def digest(path):
    return hashlib.sha256(read_regular(path)).hexdigest()


def baseline_runtime():
    from singularitydog_hw import native_pipeline_benchmark as original
    path = Path(original.__file__).resolve()
    if digest(path) != BASELINE_SHA:
        raise ValueError('Exact original K37 dependency required')
    return path.parents[1]


def candidate_module(path):
    path = Path(path)
    raw = read_regular(path)
    if hashlib.sha256(raw).hexdigest() != CANDIDATE_SHA:
        raise ValueError('Readiness instrumenter changed')
    spec = importlib.util.spec_from_file_location('_readiness_causal_candidate_r38', path)
    module = importlib.util.module_from_spec(spec)
    # Execute the verified bytes, never a stale .pyc or a second unpinned read.
    exec(compile(raw, str(path), 'exec'), module.__dict__)
    if digest(path) != CANDIDATE_SHA:
        raise ValueError('Readiness instrumenter changed during loading')
    return module


class TraceBank:
    """Preallocate every cycle/phase before collect enters its measured loop."""
    def __init__(self, cycles, candidate_path, phase):
        if type(cycles) is not int or not 5 <= cycles <= 50:
            raise ValueError('Readiness experiment permits only 5..50 cycles')
        from singularitydog_hw import native_pipeline_benchmark as original
        self.candidate_path = str(Path(candidate_path))
        if phase not in ('voltage', 'output'):
            raise ValueError('Explicit voltage or output phase required')
        self.phase = phase
        module = candidate_module(self.candidate_path)
        self._helper = module.build_traced_helper(original, Path(original.__file__).resolve())
        self._baseline = original._await_owned_ready
        self._phase_wrappers = (original._await_acquisition_ready,
                                original._await_voltage_ready, original._await_output_ready)
        self.cycles = cycles
        self.current = -1
        self._invoked = [0] * cycles
        self._traces = tuple(tuple(module.FixedReadinessTrace(256) for _ in range(3))
                             for _ in range(cycles))
        self.baseline_path = str(Path(original.__file__).resolve())

    def validate(self, *, cycles, mode, fast, overlap, validation, storage, native_wait):
        if not (cycles == self.cycles and mode == 'stop-proxy' and fast is True and
                overlap is True and validation is True and storage == 'trace' and
                callable(native_wait)):
            raise ValueError('Readiness observation requires bounded native fast STOP-proxy trace')

    def select(self, cycle):
        if type(cycle) is not int or not 0 <= cycle < self.cycles or cycle <= self.current:
            raise ValueError('Monotonic causal-trace cycle selection required')
        self.current = cycle

    def _run(self, phase_index, phase, futures, third, options):
        if self.current < 0:
            raise ValueError('Trace cycle not selected')
        if ('acquisition','voltage','output')[phase_index] != self.phase:
            if phase_index == 2:
                return self._phase_wrappers[phase_index](futures, **options)
            return self._phase_wrappers[phase_index](futures, third, **options)
        self._invoked[self.current] += 1
        return self._helper(futures, third, phase=phase,
                            trace=self._traces[self.current][phase_index], **options)

    def acquisition(self, futures, imu_future, **options):
        return self._run(0, 'Acquisition', futures, imu_future, options)

    def voltage(self, futures, validation_future, **options):
        return self._run(1, 'Voltage', futures, validation_future, options)

    def output(self, futures, **options):
        return self._run(2, 'Proxy output', futures, None, options)

    def export_after_cleanup(self):
        rows, errors = [], []
        for cycle, phases in enumerate(self._traces):
            for name, trace in zip(('acquisition', 'voltage', 'output'), phases):
                if trace._futures is None:
                    continue
                try:
                    item = trace.export()
                    rows.append(dict(cycle=cycle + 1, phase=name, trace=item))
                    # All callbacks finished; numeric export is independent.
                    # Break prototype trace/Future/closure cycles after cleanup.
                    trace._futures = None
                    trace._callbacks = None
                except Exception as error:
                    errors.append(dict(cycle=cycle + 1, phase=name,
                                       error=type(error).__name__ + ': ' + str(error)))
        unchanged = (digest(self.baseline_path) == BASELINE_SHA and
                     digest(self.candidate_path) == CANDIDATE_SHA)
        complete = bool(rows) and unchanged and not errors and all(x['trace']['trace_complete'] for x in rows)
        return dict(schema='singularitydog.readiness-causal-trace-bank.v1',
                    selected=True, selected_phase=self.phase, requested_cycles=self.cycles, rows=rows, errors=errors,
                    selected_phase_invocations=sum(self._invoked), selected_phase_observations=len(rows),
                    selected_phase_observed=bool(rows),
                    requested_cycles_all_traced=complete and len(rows) == self.cycles and all(x == 1 for x in self._invoked),
                    original_and_instrumenter_unchanged=unchanged,
                    trace_complete=complete,
                    export_after_original_collector_cleanup=True,
                    measured_overhead_known=False, timing_admission_eligible=False,
                    active_output_eligible=False)


def validate_cli(args, parser):
    if not (args.readiness_cause_trace and type(args.cycles) is int and 5 <= args.cycles <= 50 and
            args.mode == 'stop-proxy' and args.supported_disabled and
            args.request_gap_us == 900 and args.request_window == 3 and args.voltage_max_v == 42 and
            args.v3_voltage_proxy and args.v3_voltage_overlap and args.v3_voltage_validation_overlap and
            args.v3_voltage_fast_pipeline and args.prepare_voltage_before_feedback_publication and
            args.record_storage == 'trace' and args.absolute_epoch_cadence and args.release_spin_us == 500 and
            args.startup_cycle_allowance == 1 and args.output_dispatch_trace and
            args.inference_thread_cpu_trace and args.provenance_mode and args.power_epoch and
            not args.acquisition_only and not args.compare_feedback and not args.v3_voltage_pipeline and
            not args.native_boot_guard_artifact and not args.native_boot_guard_artifact_sha256 and
            not args.retain_gil_trace_copy and args.readiness_cause_candidate and
            args.readiness_cause_phase in ('voltage','output')):
        parser.error('Separate readiness module requires explicit trace, 5..50 STOP-only cycles, 900us/window3 and the fixed diagnostic scope')


def provenance(module_path, candidate_path, phase):
    module = Path(module_path).resolve()
    candidate = Path(candidate_path).resolve()
    original = baseline_runtime() / 'singularitydog_hw/native_pipeline_benchmark.py'
    if digest(candidate) != CANDIDATE_SHA:
        raise ValueError('Readiness instrumenter changed')
    if phase not in ('voltage','output'):
        raise ValueError('Explicit phase required')
    return dict(schema='singularitydog.experimental-readiness-executing-sources.v1',
                actual_executing_module=dict(path=str(module), sha256=digest(module)),
                support_module=dict(path=str(Path(__file__).resolve()), sha256=digest(__file__)),
                instrumenter=dict(path=str(candidate), sha256=CANDIDATE_SHA),
                baseline_dependency=dict(path=str(original), sha256=BASELINE_SHA),
                copied_module_is_original=False, hidden_global_monkeypatch=False,
                selected_phase=phase,
                owner_callback_is_exact_publication_time=False,
                no_extra_trace_clocks_in_original_module=True,
                measured_overhead_known=False, timing_admission_eligible=False,
                active_output_eligible=False, sources_unchanged_after_run=None)


def finalize_report(report, proof):
    """Do not expose baseline dependency pins as the executing cadence graph."""
    changed = []
    for key in ('actual_executing_module', 'support_module', 'instrumenter', 'baseline_dependency'):
        reference = proof[key]
        if digest(reference['path']) != reference['sha256']:
            changed.append(key)
    proof['sources_unchanged_after_run'] = not changed
    if changed:
        report['status'] = 'ABORTED'
        report.setdefault('errors', []).append('Readiness experiment source changed: ' + ','.join(changed))
    for obj in (report, report.get('source_provenance', {}),
                report.get('plan', {}).get('source_provenance', {})):
        if 'cadence_source_sha256' in obj:
            obj['baseline_dependency_source_sha256'] = obj.pop('cadence_source_sha256')
            obj['source_map_role'] = 'original dependencies only; actual module is separately identified'
    report['experimental_readiness_cause_trace'] = proof
    report['timing_admission_eligible'] = False
    report['active_output_eligible'] = False
    report['full_controller_50Hz_verified'] = False
