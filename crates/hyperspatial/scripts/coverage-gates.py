#!/usr/bin/env python3
"""Regress independent coverage floors; optionally replay retained measurements.

Replay changes reporting/gating only, after verifying measured source hashes.
It never claims to execute tests or regenerate LLVM measurements.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path

from coverage_common import native_elliptic_coverage, check_native_elliptic_coverage, native_source_groups, check_native_function_coverage

root = Path(__file__).resolve().parents[3]
sources = native_source_groups(root)
files = [{"filename": str(path),
          "summary": {metric: {"count": 10000, "covered": 10000}
                      for metric in ("lines", "regions", "functions")}}
         for paths in sources.values() for path in paths]
export = {"data": [{"files": files}]}
check_native_elliptic_coverage(native_elliptic_coverage(export, root))
for label, paths in sources.items():
    masked = copy.deepcopy(export)
    for file in masked["data"][0]["files"]:
        if Path(file["filename"]) in paths:
            file["summary"]["lines"] = {"count": 100, "covered": 94}
    combined = [f["summary"]["lines"] for f in masked["data"][0]["files"]]
    assert 100 * sum(row["covered"] for row in combined) / sum(row["count"] for row in combined) >= 95
    try:
        check_native_elliptic_coverage(native_elliptic_coverage(masked, root))
    except SystemExit as error:
        assert label in str(error)
    else:
        raise AssertionError("combined coverage masked a failing independent floor")
    for path in paths:
        missing = copy.deepcopy(export)
        missing["data"][0]["files"] = [file for file in missing["data"][0]["files"]
                                      if file["filename"] != str(path)]
        try:
            native_elliptic_coverage(missing, root)
        except SystemExit as error:
            assert f"every {label}" in str(error)
        else:
            raise AssertionError("missing source was accepted")
print("Independent native coverage floors and missing-file regressions passed")

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--report", type=Path)
parser.add_argument("--output", type=Path)
args = parser.parse_args()
if bool(args.report) != bool(args.output):
    parser.error("replay requires both --report and --output")
if args.report:
    report = json.loads(args.report.read_text())
    for label, base in [(None, root / "crates/hyperspatial"),
                        ("native_inverse", root), ("native_exponential", root)]:
        group = report if label is None else report[label]
        for path, digest in group["source_sha256"].items():
            if hashlib.sha256((base / path).read_bytes()).hexdigest() != digest:
                raise SystemExit(f"Measured production source changed: {path}")
    measured = {}
    for label in sources:
        group = report.get(label, {})
        if set(group.get("files", {})) != set(group.get("source_sha256", {})):
            raise SystemExit(f"Incomplete source hashes for {label}")
        for path, digest in group.get("source_sha256", {}).items():
            if hashlib.sha256((root / path).read_bytes()).hexdigest() != digest:
                raise SystemExit(f"Measured production source changed: {path}")
        measured.update(group.get("files", {}))
    export = {"data": [{"files": [{"filename": str(root / path), "summary": summary}
                                   for path, summary in measured.items()]}]}
    report.update(native_elliptic_coverage(export, root))
    check_native_elliptic_coverage(report)
    check_native_function_coverage(report)
    for label in ("native_atan2", "native_atan_deferred"):
        for path, digest in report[label]["source_sha256"].items():
            if hashlib.sha256((root / path).read_bytes()).hexdigest() != digest:
                raise SystemExit(f"Measured production source changed: {path}")
    assert report["totals"]["lines"]["percent"] >= 95
    assert report["native_inverse"]["physical_executable_lines"]["percent"] >= 95
    report["gate_replay"] = {"input": str(args.report),
                           "input_sha256": hashlib.sha256(args.report.read_bytes()).hexdigest(),
                           "tests_reexecuted": False,
                           "source": "unchanged LLVM per-file measurements; independent group floors"}
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print("Retained measurements pass all independent line gates")
