"""Build/replay are separate explicit commands, both strictly CPU/file-only."""
import argparse
import json
from pathlib import Path
from .contracts import environment, reference_policy, require
from .loader import load_verified
from .build import build, write_json
from .verification import benchmark, saved_inputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    new = commands.add_parser("build")
    new.add_argument("--bundle", type=Path, required=True)
    new.add_argument("--output", type=Path, required=True)
    new.add_argument("--compiler", default="c++")
    replay = commands.add_parser("replay")
    replay.add_argument("--bundle", type=Path, required=True)
    replay.add_argument("--manifest", type=Path, required=True)
    replay.add_argument("--manifest-sha256", required=True)
    replay.add_argument("--input-report", type=Path, required=True)
    replay.add_argument("--output", type=Path, required=True)
    replay.add_argument("--samples", type=int, default=60)
    args = parser.parse_args()
    if args.command == "build":
        report = build(args.bundle, args.output, compiler=args.compiler)
        print(json.dumps({k: report[k] for k in ("status", "hardware_opened", "manifest_sha256")}))
        return
    require(not args.output.exists(), "Output report must be new")
    import torch
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    candidate, provenance = load_verified(args.manifest,
        expected_manifest_sha256=args.manifest_sha256, bundle=args.bundle)
    reference, reference_sha = reference_policy(args.bundle)
    inputs, input_sha = saved_inputs(torch, args.input_report.read_bytes())
    result = benchmark(torch, reference, candidate, inputs, samples=args.samples)
    result.update(provenance=provenance, environment=environment(torch),
                  saved_input_sha256=input_sha, reference_loader_sha256=reference_sha)
    write_json(args.output, result)
    print(json.dumps(dict(status=result["status"], distributions=result["distributions"], output_allowed=False)))


if __name__ == "__main__":
    main()
