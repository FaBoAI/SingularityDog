"""Compile the separate scalar CPU step operator for file-only validation."""
import argparse
import json
from pathlib import Path

from ..contracts import require, sha
from .run import compile_actor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output).expanduser().absolute()
    require(output.parent.is_dir() and not output.exists(), "Require new output file")
    import torch
    source = Path(__file__).resolve().parent / "step_scalar.cpp"
    milliseconds = compile_actor(torch, output, source)
    print(json.dumps({"library_sha256": sha(output.read_bytes()),
                      "source_sha256": sha(source.read_bytes()),
                      "compile_ms": milliseconds,
                      "hardware_opened": False, "output_allowed": False}))


if __name__ == "__main__":
    main()
