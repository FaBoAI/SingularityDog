"""Bounded-phase Jetson CPU replay of a previously built step candidate.

Run each phase under an external `timeout 30s`. Files only; no hardware API.
"""
import argparse
import json
import os
from pathlib import Path
import time

from ..contracts import environment, pinned, require, sha
from ..loader import load_verified
from ..verification import compare_state, compare_tensor, frame, rejections, reject_reason
from ..view_cache.generator import verify_aliases
from ..view_cache.loader import load_file_only_verified
from .run import distribution, load_records

STEP_SOURCE_SHA256 = "69b34790df67f00314c638199efba99abc0469969cfda52e31f4a48ad8a6332d"


def _models(torch, args, *, with_cached=False):
    baseline, _ = load_verified(args.baseline_manifest,
        expected_manifest_sha256=args.baseline_sha, bundle=args.bundle)
    cached = None
    if with_cached:
        require(args.view_manifest and args.view_sha, "Timing requires pinned cached-view model")
        cached, _ = load_file_only_verified(args.view_manifest,
            expected_sha256=args.view_sha, baseline_manifest=args.baseline_manifest,
            baseline_sha=args.baseline_sha, bundle=args.bundle)
    pinned(args.candidate_library, args.candidate_library_sha)
    torch.ops.load_library(str(Path(args.candidate_library).resolve()))
    pinned(args.candidate_model, args.candidate_model_sha)
    candidate = torch.jit.load(args.candidate_model, map_location="cpu").eval()
    require("sd_step_fileonly_r1::step" in str(candidate.inlined_graph) and
            "sd_projection_fileonly_r1::project" in str(candidate.inlined_graph),
            "Expected native operators missing")
    verify_aliases(candidate.controller)
    return baseline, cached, candidate


def _parity(torch, baseline, candidate, frames):
    ids = torch.tensor([0], dtype=torch.long)
    maxima = {}
    with torch.inference_mode():
        baseline.reset(ids); candidate.reset(ids)
        for index, inputs in enumerate(frames):
            saved = tuple(x.clone() for x in inputs)
            a, b = baseline(*inputs), candidate(*inputs)
            compare_tensor(torch, a, b, "target", maxima, exact=True)
            compare_state(torch, baseline, candidate, maxima, exact=True)
            require(all(torch.equal(x,y) for x,y in zip(saved,inputs)),
                    "Input mutated at cycle " + str(index+1))
        rejected = []
        for name, inputs in rejections(torch, frames[0]).items():
            reasons = []
            for model in (baseline,candidate):
                model.reset(ids)
                try:
                    model(*inputs)
                except (ValueError,RuntimeError,torch.jit.Error) as error:
                    reasons.append(reject_reason(error))
                else:
                    raise ValueError("Invalid input accepted: " + name)
            require(reasons[0] == reasons[1], "Rejection differs: " + name)
            compare_state(torch, baseline, candidate, maxima, exact=True)
            rejected.append({"case":name,"reason":reasons[0]})
        verify_aliases(candidate.controller)
    require(len(rejected) == 26 and all(x == 0. for x in maxima.values()),
            "Exact parity incomplete")
    return {"saved_recurrent_calls":len(frames), "all_named_state_each_call":True,
            "observation_actor_target_exact":True, "input_mutation":False,
            "rejection_count":len(rejected), "rejections":rejected, "max_errors":maxima}


def _synthetic(torch, baseline, candidate):
    ids = torch.tensor([0], dtype=torch.long)
    nominal = baseline.controller.nominal.float().reshape(1,12)
    maxima = {}
    with torch.inference_mode():
        for index in range(240):
            if index in (0,125):
                baseline.reset(ids); candidate.reset(ids)
            inputs = frame(torch,nominal,index)
            a,b = baseline(*inputs), candidate(*inputs)
            compare_tensor(torch,a,b,"target",maxima,exact=True)
            compare_state(torch,baseline,candidate,maxima,exact=True)
    require(all(x == 0. for x in maxima.values()), "Synthetic parity incomplete")
    return {"synthetic_frames":240,"reset_before":[0,125],
            "all_named_state_each_frame":True,"max_errors":maxima}


def _timing(torch, cached, candidate, frames):
    ids=torch.tensor([0],dtype=torch.long)
    models=(cached,candidate)
    wall=[[],[]]; cpu=[[],[]]
    with torch.inference_mode():
        for model in models:
            model.reset(ids)
            for inputs in frames[:10]: model(*inputs)
            model.reset(ids)
        for index,inputs in enumerate(frames):
            targets=[None,None]
            for offset in range(2):
                slot=(index+offset)%2
                start_wall=time.perf_counter_ns();start_cpu=time.thread_time_ns()
                targets[slot]=models[slot](*inputs)
                cpu[slot].append(time.thread_time_ns()-start_cpu)
                wall[slot].append(time.perf_counter_ns()-start_wall)
            require(torch.equal(targets[0],targets[1]),"Timed target differs")
        compare_state(torch,cached,candidate,{},exact=True)
    return {name:{"wall":distribution(wall[index]),"thread_cpu":distribution(cpu[index])}
            for index,name in enumerate(("cached","step_fused_cached"))}


def run(args):
    import torch
    require(sha((Path(__file__).resolve().parent/"step.cpp").read_bytes()) == STEP_SOURCE_SHA256,
            "Step source changed")
    output=Path(args.output)
    require(output.parent.is_dir() and not output.exists(),"Require new report file")
    report={"schema":"native-step-fusion-short-replay-r1","phase":args.phase,
            "status":"INCOMPLETE","hardware_opened":False,"output_allowed":False,
            "approved_for_runtime":False,"live_50hz_verified":False,
            "environment":environment(torch),"step_source_sha256":STEP_SOURCE_SHA256,
            "baseline_manifest_sha256":args.baseline_sha,
            "candidate_model_sha256":args.candidate_model_sha,
            "candidate_library_sha256":args.candidate_library_sha,
            "records_sha256":args.records_sha}
    try:
        torch.set_num_threads(1);torch.set_num_interop_threads(1)
        frames=load_records(torch,args.records,args.records_sha)
        baseline,cached,candidate=_models(torch,args,with_cached=args.phase=="timing")
        if args.phase=="parity":
            report["validation"]=_parity(torch,baseline,candidate,frames)
        elif args.phase=="synthetic":
            report["validation"]=_synthetic(torch,baseline,candidate)
        else:
            report["timing"]=_timing(torch,cached,candidate,frames)
            report["view_manifest_sha256"]=args.view_sha
        report["status"]="PASS_FILE_ONLY"
    except BaseException as error:
        report["status"]="FAILED"
        report["error"]=type(error).__name__+": "+str(error)
        raise
    finally:
        fd=os.open(output,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,"w") as stream:
            json.dump(report,stream,indent=2,allow_nan=False)
            stream.write("\n")
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase",choices=("parity","synthetic","timing"),required=True)
    parser.add_argument("--baseline-manifest",required=True)
    parser.add_argument("--baseline-sha",required=True)
    parser.add_argument("--view-manifest")
    parser.add_argument("--view-sha")
    parser.add_argument("--bundle",required=True)
    parser.add_argument("--records",required=True)
    parser.add_argument("--records-sha",required=True)
    parser.add_argument("--candidate-model",required=True)
    parser.add_argument("--candidate-model-sha",required=True)
    parser.add_argument("--candidate-library",required=True)
    parser.add_argument("--candidate-library-sha",required=True)
    parser.add_argument("--output",required=True)
    result=run(parser.parse_args())
    print(json.dumps({"status":result["status"],"phase":result["phase"]}))


if __name__=="__main__":
    main()
