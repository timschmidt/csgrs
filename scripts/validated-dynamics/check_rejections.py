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
def case(records, name):
    return next(row for row in records if row["case"] == name)


mutations.update({
    "turnaround_endpoint_as_tube": lambda r: case(r, "turnaround_point").__setitem__(
        "tube", copy.deepcopy(case(r, "turnaround_point")["end"])),
    "noncommuting_missed_interior_minimum": lambda r: case(r, "noncommuting_box")["tube"].__setitem__(
        0, ["1023", "1040", -10]),
    "noncommuting_assumed_commutativity": lambda r: case(r, "noncommuting_point")["end"].__setitem__(
        0, ["257", "257", -8]),
    "noncommuting_wrong_dimension": lambda r: case(r, "noncommuting_point")["end"].pop(),
    "time_growth_ignored_time": lambda r: case(r, "time_growth_point")["end"].__setitem__(0, ["1", "1", 0]),
    "two_rate_wrong_initial": lambda r: case(r, "two_rate_point")["initial"].__setitem__(1, ["0", "0", 0]),
    "two_rate_oversized_step": lambda r: (case(r, "two_rate_point").__setitem__("pieces", 4),
        case(r, "two_rate_point").__setitem__("time_domains", [[str(i), str(i), -4] for i in range(5)])),
    "stopped_with_endpoint": lambda r: case(r, "two_rate_box").__setitem__("status", "stopped"),
})
for name, mutate in mutations.items():
    incorrect = copy.deepcopy(rows)
    mutate(incorrect)
    try:
        validate(incorrect)
    except AssertionError:
        print(f"rejected {name}")
    else:
        raise RuntimeError(f"incorrect workload accepted: {name}")
