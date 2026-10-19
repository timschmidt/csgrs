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

from coverage_common import native_elliptic_coverage, check_native_elliptic_coverage

root = Path(__file__).resolve().parents[3]
files = [{"filename": str(root / "crates/hyperreal/src" / part / name),
          "summary": {metric: {"count": 50, "covered": 50}
                      for metric in ("lines", "regions", "functions")}}
         for name in ("elliptic_integrals.rs", "legendre_integrals.rs", "inverse_legendre.rs", "jacobi.rs")
         for part in ("computable/approximation", "computable/node", "real/arithmetic")]
files.append({"filename": str(root / "crates/hyperreal/src/computable/node/log_argument.rs"),
              "summary": {metric: {"count": 50, "covered": 50}
                          for metric in ("lines", "regions", "functions")}})
export = {"data": [{"files": files}]}
check_native_elliptic_coverage(native_elliptic_coverage(export, root))
groups = [(0, 3, "native_elliptic"), (3, 3, "native_legendre"),
          (6, 3, "native_inverse_legendre"), (9, 4, "native_jacobi")]
for start, count, label in groups:
    masked = copy.deepcopy(export)
    for offset, count, covered in [(0, 51, 46), (1, 50, 45), (2, 50, 45)]:
        masked["data"][0]["files"][start + offset]["summary"]["lines"] = {
            "count": count, "covered": covered}
    assert 100 * (136 + 150) / (151 + 150) >= 95
    try:
        check_native_elliptic_coverage(native_elliptic_coverage(masked, root))
    except SystemExit as error:
        assert label in str(error)
    else:
        raise AssertionError("combined coverage masked a failing independent floor")
for start, count, label in groups:
    for offset in range(count):
        missing = copy.deepcopy(export)
        missing["data"][0]["files"].pop(start + offset)
        try:
            native_elliptic_coverage(missing, root)
        except SystemExit as error:
            assert f"every {label}" in str(error)
        else:
            raise AssertionError("missing source was accepted")
print("Independent Carlson/Legendre/inverse/Jacobi coverage floors and missing-file regression passed")

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
    for label in ("native_elliptic", "native_legendre", "native_inverse_legendre", "native_jacobi"):
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
    assert report["totals"]["lines"]["percent"] >= 95
    assert report["native_inverse"]["physical_executable_lines"]["percent"] >= 95
    report["gate_replay"] = {"input": str(args.report),
                           "input_sha256": hashlib.sha256(args.report.read_bytes()).hexdigest(),
                           "tests_reexecuted": False,
                           "source": "unchanged LLVM per-file measurements; independent group floors"}
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print("Retained measurements pass all six independent line gates")
