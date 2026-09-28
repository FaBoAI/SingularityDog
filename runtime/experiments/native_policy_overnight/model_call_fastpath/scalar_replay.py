"""File-only, bounded-phase comparison of a scalar step-op implementation.

Run in a fresh process with a caller-pinned scalar C++ source and library. The
TorchScript model is the established step-fused graph; this experiment replaces
only the implementation registered as sd_step_fileonly_r1::step. The scalar and
original libraries cannot be registered in one process.
"""
import argparse
import json
import os
from pathlib import Path

from ..contracts import environment, pinned, require, sha
from .run import load_records
from .short_replay import _models, _parity, _synthetic, _timing


def run(args):
    import torch
    here = Path(__file__).resolve().parent
    pinned(here / "step_scalar.cpp", args.scalar_source_sha)
    output = Path(args.output)
    require(output.parent.is_dir() and not output.exists(), "Require new report file")
    report = {
        "schema": "native-step-scalar-short-replay-r1",
        "phase": args.phase,
        "status": "INCOMPLETE",
        "hardware_opened": False,
        "output_allowed": False,
        "approved_for_runtime": False,
        "live_50hz_verified": False,
        "environment": environment(torch),
        "scalar_step_source_sha256": args.scalar_source_sha,
        "scalar_replay_source_sha256": sha(Path(__file__).read_bytes()),
        "candidate_model_sha256": args.candidate_model_sha,
        "candidate_library_sha256": args.candidate_library_sha,
        "baseline_manifest_sha256": args.baseline_sha,
        "records_sha256": args.records_sha,
        "operator": "sd_step_fileonly_r1::step",
        "implementation": "scalar_cpp_file_only",
        "same_model_graph_as_aten_step_candidate": True,
        "fresh_process_required": True,
    }
    try:
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        frames = load_records(torch, args.records, args.records_sha)
        baseline, cached, candidate = _models(torch, args,
            with_cached=args.phase == "timing")
        if args.phase == "parity":
            report["validation"] = _parity(torch, baseline, candidate, frames)
        elif args.phase == "synthetic":
            report["validation"] = _synthetic(torch, baseline, candidate)
        else:
            report["timing"] = _timing(torch, cached, candidate, frames)
            report["view_manifest_sha256"] = args.view_sha
        report["status"] = "PASS_FILE_ONLY"
    except BaseException as error:
        report["status"] = "FAILED"
        report["error"] = type(error).__name__ + ": " + str(error)
        raise
    finally:
        fd = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("parity", "synthetic", "timing"), required=True)
    parser.add_argument("--scalar-source-sha", required=True)
    parser.add_argument("--baseline-manifest", required=True)
    parser.add_argument("--baseline-sha", required=True)
    parser.add_argument("--view-manifest")
    parser.add_argument("--view-sha")
    parser.add_argument("--bundle", required=True)
    parser.add_argument("--records", required=True)
    parser.add_argument("--records-sha", required=True)
    parser.add_argument("--candidate-model", required=True)
    parser.add_argument("--candidate-model-sha", required=True)
    parser.add_argument("--candidate-library", required=True)
    parser.add_argument("--candidate-library-sha", required=True)
    parser.add_argument("--output", required=True)
    result = run(parser.parse_args())
    print(json.dumps({"status": result["status"], "phase": result["phase"]}))


if __name__ == "__main__":
    main()
