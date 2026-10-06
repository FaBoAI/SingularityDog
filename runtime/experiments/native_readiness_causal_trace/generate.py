"""Generate a separate diagnostic source. Does not import or execute it."""
import argparse
import ast
import hashlib
import json
import os
import stat
from pathlib import Path

BASELINE_SHA = '0bafa9b92bea7ea641f57239a5ff1cd4784078b7b5bf09b5c917d0019acd5c93'


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Expected unique source anchor: ' + old[:70])
    return text.replace(old, new, 1)


def derive(raw):
    if hashlib.sha256(raw).hexdigest() != BASELINE_SHA:
        raise ValueError('Exact baseline source required')
    edits = []
    def apply(text, old, new):
        edits.append((old, new))
        return once(text, old, new)
    source = raw.decode()
    source = apply(source, 'PERIOD_NS = 20_000_000',
                  'import readiness_cause_collector_support_r38 as _readiness_cause\n\nPERIOD_NS = 20_000_000')
    source = apply(source, '            prepare_voltage_before_feedback_publication=False):',
                  '            prepare_voltage_before_feedback_publication=False,readiness_cause_recorder=None):')
    source = apply(source, '    records=[];measurements=[];errors=[]',
        "    await_acquisition=_await_acquisition_ready;await_voltage=_await_voltage_ready;await_output=_await_output_ready\n"
        "    if readiness_cause_recorder is not None:\n"
        "        if type(readiness_cause_recorder) is not _readiness_cause.TraceBank:raise ValueError('Exact trace bank required')\n"
        "        readiness_cause_recorder.validate(cycles=cycles,mode=mode,fast=v3_voltage_fast_pipeline,\n"
        "            overlap=v3_voltage_overlap,validation=v3_voltage_validation_overlap,storage=record_storage,native_wait=deadline_wait)\n"
        "        await_acquisition=readiness_cause_recorder.acquisition\n"
        "        await_voltage=readiness_cause_recorder.voltage\n"
        "        await_output=readiness_cause_recorder.output\n"
        "    records=[];measurements=[];errors=[]")
    source = apply(source, '        for cycle in range(cycles):\n',
        '        for cycle in range(cycles):\n            if readiness_cause_recorder is not None:readiness_cause_recorder.select(cycle)\n')
    for old, new in (('_await_acquisition_ready(', 'await_acquisition('),
                     ('_await_voltage_ready(', 'await_voltage('),
                     ('_await_output_ready(', 'await_output(')):
        # Only collector calls change; original helper definitions are retained.
        source = apply(source, '=' + old, '=' + new)
    source = apply(source, "    if trace_copy_proof is not None:report['trace_copy_provenance']=trace_copy_proof",
        "    if readiness_cause_recorder is not None:\n"
        "        report['readiness_causal_trace']=readiness_cause_recorder.export_after_cleanup()\n"
        "        report['timing_admission_eligible']=False;report['active_output_eligible']=False\n"
        "    if trace_copy_proof is not None:report['trace_copy_provenance']=trace_copy_proof")
    source = apply(source, '    p=argparse.ArgumentParser(description=__doc__)\n',
        "    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)\n"
        "    p.add_argument('--readiness-cause-trace',action='store_true')\n"
        "    p.add_argument('--readiness-cause-candidate')\n"
        "    p.add_argument('--readiness-cause-phase',choices=('voltage','output'),required=True)\n")
    source = apply(source, '    args=p.parse_args(argv)\n',
        "    args=p.parse_args(argv)\n    _readiness_cause.validate_cli(args,p)\n"
        "    cause_proof=_readiness_cause.provenance(__file__,args.readiness_cause_candidate,args.readiness_cause_phase)\n")
    source = apply(source, "    if not args.execute:\n        print(json.dumps(plan,indent=2));return 0",
        "    plan['experimental_readiness_cause_trace']=cause_proof\n"
        "    plan['timing_admission_eligible']=False;plan['active_output_eligible']=False\n"
        "    if source_provenance is not None:source_provenance['source_map_role']='original dependency graph only'\n"
        "    if not args.execute:\n        print(json.dumps(plan,indent=2));return 0")
    source = apply(source, "Path(__file__).resolve().parents[1]/'experiments'",
                  "_readiness_cause.baseline_runtime()/'experiments'")
    source = apply(source, '                    result,saved=collect(sessions,device,run,**options)',
        '                    options[\'readiness_cause_recorder\']=_readiness_cause.TraceBank(args.cycles,args.readiness_cause_candidate,args.readiness_cause_phase)\n'
        '                    result,saved=collect(sessions,device,run,**options)')
    source = apply(source, '        _finish_source_provenance(report,source_provenance)\n',
        '        _finish_source_provenance(report,source_provenance)\n'
        '        _readiness_cause.finalize_report(report,cause_proof)\n')
    ast.parse(source)
    original = {n.name:ast.dump(n, include_attributes=False) for n in ast.parse(raw).body
                if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
    changed = {n.name:ast.dump(n, include_attributes=False) for n in ast.parse(source).body
               if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
    if original.keys() != changed.keys():
        raise ValueError('Changed source function/class inventory')
    delta = sorted(key for key in original if original[key] != changed[key])
    if delta != ['collect', 'main']:
        raise ValueError('Unexpected structural source changes: ' + repr(delta))
    restored = source
    for old, new in reversed(edits):
        restored = once(restored, new, old)
    if restored.encode() != raw or ast.dump(ast.parse(restored)) != ast.dump(ast.parse(raw)):
        raise ValueError('Inverse source and AST identity proof failed')
    return source.encode(), delta


KIT_SHA = 'f3d48a7cabae2e502863f5e87aed2ba1b076faa292d661f59572a79b81f896fe'
OUTER_SHA = '01c108e32445b3d484e39afb20c14e6e46869d4a9e088c7e1dbbb3cf46ebba99'
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


def absolute_location(path):
    """A target-host declaration, not a claim about this host's filesystem."""
    path = Path(path)
    if not path.is_absolute() or '..' in path.parts:
        raise ValueError('Absolute location without parent traversal required')
    return path


def private_destination(path):
    path = absolute_location(path)
    if (any(p.is_symlink() for p in (path, *path.parents)) or
            any((p / '.git').exists() for p in (path, *path.parents))):
        raise ValueError('Absolute non-symlink private destination required')
    return path


def derive_launcher(raw, target_bundle, manifest_sha, runner_sha):
    """Preserve the pinned outer admission/performance/cleanup code by inversion."""
    if hashlib.sha256(raw).hexdigest() != OUTER_SHA:
        raise ValueError('Exact original scoped R37 launcher required')
    target_bundle = str(absolute_location(target_bundle))
    edits = []
    def apply(text, old, new):
        edits.append((old, new))
        return once(text, old, new)
    source = raw.decode()
    source = apply(source, 'parser=argparse.ArgumentParser(description=__doc__)',
                   'parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)')
    source = apply(source, "mode=parser.add_mutually_exclusive_group(required=True)",
                   "mode=parser.add_mutually_exclusive_group(required=False)")
    source = apply(source, "parser.add_argument('--variant',required=True,choices=('default',))",
                   "parser.add_argument('--variant',default='default',choices=('default',))\n"
                   "parser.add_argument('--cycles',type=int,default=5)\n"
                   "parser.add_argument('--phase',choices=('voltage','output'),required=True)")
    source = apply(source, 'execute=args.execute\n',
        "execute=args.execute\nneed(5<=args.cycles<=50,'Only 5..50 cycles permitted')\n"
        f"TRACE_BUNDLE=P({target_bundle!r})\nTRACE_BUNDLE_SHA={manifest_sha!r}\n"
        f"TRACE_RUNNER_SHA={runner_sha!r}\n"
        "need(sha(TRACE_BUNDLE/'readiness_cause_child_r38.py')==TRACE_RUNNER_SHA,'Trace runner changed')\n"
        "# Source-only preflight; the pinned child verifies all bundle/kit bytes again.\n"
        "need(sha(TRACE_BUNDLE/'manifest.json')==TRACE_BUNDLE_SHA,'Trace bundle changed')\n")
    source = apply(source, "option('--cycles','501')", "option('--cycles',str(args.cycles))")
    source = apply(source, "'--cycles','501','--request-window'", "'--cycles',str(args.cycles),'--request-window'")
    source = apply(source,
        "argv=[py,'-B',str(kit/'tools/python_thread_switch_scope.py'),'--module','singularitydog_hw.native_pipeline_benchmark','--interval-us','100','--',",
        "argv=[py,'-B',str(TRACE_BUNDLE/'readiness_cause_child_r38.py'),'--bundle',str(TRACE_BUNDLE),'--bundle-sha256',TRACE_BUNDLE_SHA,'--interval-us','100','--',\n"
        " '--readiness-cause-trace','--readiness-cause-phase',args.phase,'--readiness-cause-candidate',str(TRACE_BUNDLE/'candidate.py'),")
    source = apply(source, "corrected-active-timing-r37-adaptive-cache-gap900-cpu04-",
                   "readiness-cause-r38-gap900-cpu04-" )
    source = apply(source, "scope['performance_command']=performance_command",
        "scope['performance_command']=performance_command\n"
        "scope['qualification']='EXPERIMENTAL_READINESS_OBSERVATION_ONLY'\n"
        "scope['historical_disabled_pacing_preserved']=True\n"
        "scope['original_canonical_benchmark_is_executing']=False\n"
        "scope['readiness_trace']={'phase':args.phase,'cycles':args.cycles,'bundle_sha256':TRACE_BUNDLE_SHA,'runner_sha256':TRACE_RUNNER_SHA,'native_woke_is_verified_waiter_return':True,'future_callback_is_exact_publication_time':False,'measurement_overhead_known_on_target':False}\n"
        f"scope['baseline_outer_launcher_sha256']={OUTER_SHA!r}")
    restored = source
    for old, new in reversed(edits):
        restored = once(restored, new, old)
    if restored.encode() != raw or ast.dump(ast.parse(restored)) != ast.dump(ast.parse(raw)):
        raise ValueError('Launcher inverse source/AST proof failed')
    ast.parse(source)
    return source.encode()


def build_bundle(baseline, output, *, kit_path, launcher_template=None, target_bundle=None):
    """Write only new source files. Never import benchmark/launchers or open hardware."""
    output = private_destination(output)
    kit_path = absolute_location(kit_path)
    if output.exists():
        raise ValueError('Fresh private output required')
    if (launcher_template is None) != (target_bundle is None):
        raise ValueError('Launcher template and target bundle must be supplied together')
    original = read_regular(baseline)
    result, delta = derive(original)
    here = Path(__file__).resolve().parent
    sources = {name:read_regular(here/name) for name in ('candidate.py','collector_support.py','child_runner.py')}
    if hashlib.sha256(sources['candidate.py']).hexdigest() != CANDIDATE_SHA:
        raise ValueError('Frozen instrumenter changed')
    members = {'candidate.py':sources['candidate.py'],
        'readiness_cause_collector_support_r38.py':sources['collector_support.py'],
        'readiness_cause_child_r38.py':sources['child_runner.py'],
        'native_readiness_cause_benchmark_r38.py':result}
    manifest = dict(schema='private.readiness-cause-bundle.v1',
        files={name:hashlib.sha256(raw).hexdigest() for name,raw in members.items()},
        baseline_kit_path=str(kit_path), baseline_kit_manifest_sha256=KIT_SHA,
        baseline_source_sha256=BASELINE_SHA,
        required_phase_choices=['voltage','output'], initial_plan_cycles=5, cycle_min=5, cycle_max=50,
        default_plan=True, hardware_opened=False, timing_admission_eligible=False, active_output_eligible=False)
    manifest_raw = (json.dumps(manifest,indent=2,allow_nan=False)+'\n').encode()
    manifest_sha = hashlib.sha256(manifest_raw).hexdigest()
    outer = read_regular(launcher_template) if launcher_template else None
    if outer is not None:
        constants={node.targets[0].id:ast.literal_eval(node.value) for node in ast.parse(outer).body
            if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name)
            and node.targets[0].id in ('KIT_PATH','KIT_MANIFEST_SHA256','BENCHMARK_SHA256')}
        if constants != dict(KIT_PATH=str(kit_path),KIT_MANIFEST_SHA256=KIT_SHA,BENCHMARK_SHA256=BASELINE_SHA):
            raise ValueError('Outer launcher and bundle baseline dependency differ')
    launcher = derive_launcher(outer,target_bundle,manifest_sha,manifest['files']['readiness_cause_child_r38.py']) if outer else None
    # Recheck original file inputs before publishing a complete bundle.
    if read_regular(baseline) != original or any(read_regular(here/name) != raw for name,raw in sources.items()):
        raise ValueError('Generation input changed')
    if outer is not None and read_regular(launcher_template) != outer:
        raise ValueError('Launcher input changed')
    output.mkdir(mode=0o700,parents=True,exist_ok=False)
    for name,raw in members.items():
        (output/name).write_bytes(raw)
    if launcher is not None:
        (output/'scoped-readiness-diagnostic.py').write_bytes(launcher)
    receipt=dict(schema='private.readiness-cause-source-derivation.v2',status='SOURCE_ONLY_NOT_DEPLOYED',
        baseline_path=str(baseline),baseline_sha256=BASELINE_SHA,
        generated_module_sha256=manifest['files']['native_readiness_cause_benchmark_r38.py'],
        changed_function_nodes=delta,unchanged_helpers=True,inverse_source_bytes_and_ast_identical=True,
        source_inputs_unchanged=True,manifest_sha256=manifest_sha,
        outer_launcher_sha256=hashlib.sha256(launcher).hexdigest() if launcher else None,
        outer_baseline_sha256=OUTER_SHA if outer else None,
        target_bundle_path=str(target_bundle) if target_bundle else None,
        target_deployed=False,target_source_verified=False,target_overhead_measured=False,
        execute_launcher_source_sealed=launcher is not None,hardware_opened=False,library_loaded=False,
        timing_admission_eligible=False,active_output_eligible=False)
    (output/'derivation.json').write_text(json.dumps(receipt,indent=2)+'\n')
    # Manifest last: an interrupted partial output is not a complete bundle.
    (output/'manifest.json').write_bytes(manifest_raw)
    return receipt


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--kit-path',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--launcher-template',type=Path)
    p.add_argument('--target-bundle',type=Path)
    args=p.parse_args(argv)
    receipt=build_bundle(args.baseline,args.output,kit_path=args.kit_path,
        launcher_template=args.launcher_template,target_bundle=args.target_bundle)
    print(json.dumps(receipt,sort_keys=True))


if __name__=='__main__':main()
