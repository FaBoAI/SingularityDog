"""Pre/post source contracts for a separately generated FK STOP diagnostic."""
from pathlib import Path

from native_policy_overnight.target_tail_fk_cache import diagnostic_loader as loader

BASELINE_SHA = '0bafa9b92bea7ea641f57239a5ff1cd4784078b7b5bf09b5c917d0019acd5c93'
INTEGRATION = {'diagnostic_loader.py', 'diagnostic_support.py', 'diagnostic_generate.py',
               'diagnostic_child.py', 'fk_cache_diagnostic_benchmark.py'}


def baseline_runtime():
    from singularitydog_hw import policy_live_profile
    return Path(policy_live_profile.__file__).absolute().parents[1]


def validate_cli(args, parser, executing_module):
    if not (args.fk_cache_manifest and args.fk_cache_manifest_sha256 and
            args.mode == 'stop-proxy' and args.supported_disabled and
            args.cycles in (5, 501) and args.startup_cycle_allowance == 1 and
            args.request_gap_us == 900 and args.request_window == 3 and args.voltage_max_v == 42 and
            args.v3_voltage_proxy and args.v3_voltage_overlap and args.v3_voltage_validation_overlap and
            args.v3_voltage_fast_pipeline and args.prepare_voltage_before_feedback_publication and
            args.record_storage == 'trace' and args.absolute_epoch_cadence and args.release_spin_us == 500 and
            args.main_thread_cpu == 4 and args.exclude_policy_cpu_from_workers and args.single_thread_math and
            args.require_pinned_fast_model and args.provenance_mode and args.power_epoch and
            args.scalar_step_manifest and args.scalar_step_manifest_sha256 and
            args.native_policy_manifest and args.native_policy_manifest_sha256 and
            not args.acquisition_only and not args.compare_feedback and not args.v3_voltage_pipeline and
            not args.view_cache_manifest and not args.view_cache_manifest_sha256 and
            not args.native_boot_guard_artifact and not args.native_boot_guard_artifact_sha256 and
            not args.retain_gil_trace_copy):
        parser.error('Explicit FK selection requires fixed5-or501/900us disabled V3 STOP diagnostic scope')
    try:
        kwargs = dict(scalar_manifest=args.scalar_step_manifest, scalar_sha=args.scalar_step_manifest_sha256,
                      baseline_manifest=args.native_policy_manifest, baseline_sha=args.native_policy_manifest_sha256)
        candidate_plan = loader.plan(args.fk_cache_manifest,
                                    expected_sha256=args.fk_cache_manifest_sha256, **kwargs)
        data = loader._json({'path': str(args.fk_cache_manifest), 'sha256': args.fk_cache_manifest_sha256})
        integration = data['integration_sources']
        if set(integration) != INTEGRATION:
            raise ValueError('Exact separate executing source inventory required')
        if (loader.reference(integration['fk_cache_diagnostic_benchmark.py']) != Path(executing_module).absolute()
                or loader.reference(integration['diagnostic_support.py']) != Path(__file__).absolute()):
            raise ValueError('Executing copy/support source binding differs')
        original = baseline_runtime() / 'singularitydog_hw/native_pipeline_benchmark.py'
        baseline = {'path': str(original), 'sha256': BASELINE_SHA}
        loader.read(baseline)
        return dict(schema='singularitydog.fk-stop-diagnostic-selection.v1',
            candidate_plan=candidate_plan,
            candidate_manifest={'path':str(args.fk_cache_manifest), 'sha256':args.fk_cache_manifest_sha256},
            actual_executing_sources=integration, baseline_dependency=baseline,
            copied_module_is_original=False, model_is_original_scalar=False,
            sources_unchanged_after_run=None, active_controller_qualification=False,
            timing_admission_eligible=False, output_allowed=False, approved_for_runtime=False)
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.error(str(error))


def finalize_report(report, proof):
    """Keep the original cadence graph as dependency data, never qualification."""
    changed = []
    for name, ref in {'candidate_manifest': proof['candidate_manifest'],
                      'baseline_dependency': proof['baseline_dependency'],
                      **proof['actual_executing_sources']}.items():
        try:
            loader.read(ref)
        except (ValueError, OSError):
            changed.append(name)
    proof['sources_unchanged_after_run'] = not changed
    if changed:
        report['status'] = 'ABORTED'
        report.setdefault('errors', []).append('FK diagnostic source changed: ' + ','.join(changed))
    for value in (report, report.get('source_provenance', {}),
                  report.get('plan', {}).get('source_provenance', {})):
        if 'cadence_source_sha256' in value:
            value['baseline_dependency_source_sha256'] = value.pop('cadence_source_sha256')
            value['source_map_role'] = 'original dependencies; executing module is separately identified'
    report['experimental_fk_cache_diagnostic'] = proof
    report.update(active_controller_qualification=False, actual_output_qualification=False,
                  timing_admission_eligible=False, approved_for_runtime=False,
                  full_controller_50Hz_verified=False, output_allowed=False)
