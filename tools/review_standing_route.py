"""Review two saved JSON captures without opening any robot device."""

import argparse
import json
from pathlib import Path

from singularitydog_hw.staged_stance_review import review_historical_stand


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def _load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      object_pairs_hook=_unique_pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(
                          ValueError(f"Nonfinite JSON constant: {value}")))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current", required=True, help="Read-only twelve-axis snapshot JSON")
    parser.add_argument("--historical", required=True, help="Offline D17 stand geometry JSON")
    parser.add_argument("--output", required=True, help="Review-only JSON output")
    args = parser.parse_args(argv)
    review = review_historical_stand(_load(args.current), _load(args.historical))
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(review, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                           encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
