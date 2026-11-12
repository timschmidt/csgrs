#!/usr/bin/env python3
"""Inventory interval-related declarations without claiming an implementation audit.

Run before each migration, then review the affected rows. Rust declarations are
selected lexically, including inline tests; macros and unnamed endpoint tuples
need manual review. Missing sibling repositories are reported, never omitted.
The output is deterministic for identical source trees.
"""

import argparse
import csv
from pathlib import Path
import re
import subprocess


DECLARATION = re.compile(
    r"^\s*(?P<visibility>pub(?:\([^)]*\))?\s+)?"
    r"(?:(?:async|const|unsafe)\s+)?"
    r"(?P<kind>struct|enum|type|fn)\s+(?P<name>[A-Za-z_][A-Za-z_0-9]*)",
    re.MULTILINE,
)
KEYWORD = re.compile(r"interval|enclosure|ball|bounds|range", re.IGNORECASE)
FIELDS = [
    "repository", "path", "line", "symbol", "kind", "endpoint_representation",
    "semantics", "allocation", "platform", "visibility", "serialization",
    "consumers", "owner", "disposition", "baseline_tests", "baseline_benchmarks",
    "review_status",
]

# These are disposition rules, not claims that every matched body was reviewed.
def disposition(repository, path, name):
    if repository == "alumina-firmware":
        return "domain set / evidence", "retain wrapper; review fixed arithmetic for delegation"
    if repository == "alumina-interface":
        return "domain uncertainty / evidence", "retain units and evidence; review rational arithmetic"
    if "/hyperreal/" in path:
        return "point enclosure implementation", "retain point authority; consume neutral bridge"
    if "/hyperlimit/" in path:
        return "predicate / certified filter", "retain policy and decision authority"
    if "/hyperpath/" in path:
        return "path domain", "review dynamic extraction; retain static geometry"
    if path.endswith("hypersolve/src/real_interval.rs"):
        return "exact Real proof adapter", "retain strict symbolic adapter; review rational helpers"
    if "/hypersolve/" in path:
        if name == "rational_interval_product":
            return "exact rational set hull", "move arithmetic to hyperinterval after differential gates"
        return "solver set / certificate", "retain solver semantics; review generic arithmetic extraction"
    return "requires body review", "retain in owner pending consumer-specific admission"


def scan(root, repository):
    if not root.is_dir():
        return [{"repository": repository, "review_status": "repository unavailable"}]
    files = subprocess.check_output(
        ["rg", "--files", "-g", "*.rs", "-g", "!target/**", "-g", "!vendor/**"],
        cwd=root, text=True,
    ).splitlines()
    rows = []
    for filename in sorted(files):
        parts = Path(filename).parts
        if any(part in {"tests", "benches", "fuzz", "examples", "verification"} for part in parts):
            continue
        source = (root / filename).read_text()
        owner = parts[1] if len(parts) > 2 and parts[0] == "crates" else repository
        for match in DECLARATION.finditer(source):
            name = match["name"]
            if not KEYWORD.search(name):
                continue
            semantics, action = disposition(repository, filename, name)
            rows.append(dict(zip(FIELDS, [
                repository, filename, source.count("\n", 0, match.start()) + 1,
                name, match["kind"], "requires body review", semantics,
                "requires body review", "requires manifest/feature review",
                (match["visibility"] or "private").strip(), "requires derive/codec review",
                "requires call-site review", owner, action,
                f"cargo test -p {owner} (select features after review)",
                "requires benchmark mapping", "lexical candidate; not yet audited",
            ])))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for repository in ("csgrs", "alumina-interface", "alumina-firmware"):
        rows.extend(scan(args.workspace / repository, repository))
    with args.output.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=FIELDS, delimiter="\t", lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} candidate rows to {args.output}; body review remains required")


if __name__ == "__main__":
    main()
