#!/usr/bin/env python3
"""Verify that completed corpus records cannot hide incomplete or incorrect work."""
import copy
import json
from pathlib import Path
import sys

from validate import validate

if not __debug__:
    raise RuntimeError("validation requires Python assertions enabled")

rows = [json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
rows = [row for row in rows if row.get("kind", "certificate") == "certificate"]
validate(rows)
assert rows[0]["case"] == "constant_point" and rows[0]["encoding"] == "dyadic"
mutations = {
    "wrong_initial_set": lambda r: r[0]["initial"].__setitem__(0, ["0", "0", 0]),
    "wrong_endpoint": lambda r: r[0]["end"].__setitem__(0, ["1", "1", 0]),
    "endpoint_only_tube": lambda r: r[0]["tube"].__setitem__(0, ["3", "3", -1]),
    "too_wide_endpoint": lambda r: r[0]["end"].__setitem__(0, ["0", "100", 0]),
    "wrong_horizon": lambda r: r[0].__setitem__("end_time", ["1", "1", 0]),
    "no_segments": lambda r: r[0].__setitem__("pieces", 0),
    "missing_case": lambda r: r.pop(),
    "duplicate_case": lambda r: r.__setitem__(0, copy.deepcopy(r[1])),
    "oversized_step": lambda r: (r[0].__setitem__("pieces", 2), r[0].__setitem__(
        "time_domains", [["0", "0", 0], ["1", "1", -3], ["1", "1", -2]])),
}
for name, mutate in mutations.items():
    incorrect = copy.deepcopy(rows)
    mutate(incorrect)
    try:
        validate(incorrect)
    except AssertionError:
        print(f"rejected {name}")
    else:
        raise RuntimeError(f"incorrect workload accepted: {name}")
