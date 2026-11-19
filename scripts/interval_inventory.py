#!/usr/bin/env python3
"""Inventory interval-related declarations without claiming an implementation audit.

Refresh before each migration, then review the affected rows. Include the
plan's source modules, public declarations in the interval and dynamics crates,
keyword matches elsewhere, and declarations named by a preserved review.
Inline tests are included. Macros, reexports and unnamed endpoint tuples still
need manual review. Keep revision-specific reviews separate from the current
scan. Missing repositories remain visible.
"""

import argparse
import csv
from functools import cache
import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory
import tomllib


DECLARATION = re.compile(
    r"^[ \t]*(?P<visibility>pub(?:\([^)]*\))?\s+)?"
    r"(?:(?:async|const|unsafe)\s+)*"
    r'(?:extern(?:\s+"[^"\r\n]*")?\s+(?=fn\b))?'
    r"(?P<kind>struct|enum|trait|type|fn)\s+(?P<name>[A-Za-z_][A-Za-z_0-9]*)",
    re.MULTILINE,
)
KEYWORD = re.compile(r"interval|enclosure|ball|bounds|range|krawczyk", re.IGNORECASE)
FIELDS = [
    "repository", "path", "line", "symbol", "kind", "endpoint_representation",
    "semantics", "allocation", "platform", "visibility", "serialization",
    "consumers", "owner", "disposition", "baseline_tests", "baseline_benchmarks",
    "review_status",
]

CANDIDATE = "lexical candidate; not yet audited"
UNAVAILABLE = "repository unavailable"
INTERVAL_PACKAGES = {"hyperinterval", "hyperlattice-interval", "hyperdynamics"}
# Paths are relative to their package manifests (plan sections 7–10 and 12).
SEED_FILES = {
    "hypersolve": {
        "src/interval.rs", "src/real_interval.rs", "src/jacobian.rs", "src/linalg.rs",
        "src/alpha.rs", "src/root_isolation.rs", "src/certification.rs", "src/residual_replay.rs",
        # Test modules included by these owners with an explicit #[path].
        "src/root_isolation_monic_tests.rs", "src/interval_migration/root_isolation.rs",
        "src/interval_migration/algebraic.rs",
    },
    "alumina-interface-core": {
        "src/boundary.rs", "src/machine_profile.rs", "src/motion_schedule.rs", "src/compiler.rs",
        "src/direct_motion.rs", "src/servo_motion.rs", "src/shared_timing_evidence.rs", "src/graph.rs",
    },
    "alumina-foc": {"src/lib.rs", "src/angle.rs", "src/current.rs", "src/servo.rs", "src/pwm.rs"},
    "alumina-clock": {"src/lib.rs"},
    "alumina-sim": {"src/foc.rs"},
    "hyperlimit": {"src/predicate.rs", "src/resolve.rs"},
    "hyperpath": {"src/routing.rs", "src/tangent.rs", "src/solve.rs"},
}
SEED_DIRECTORIES = {
    "alumina-interface-core": "src/graph",
    "hyperpath": "src/routing",
    "hyperlimit": "src/predicates",
}


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


def scan(root, repository, reviewed_names):
    if not root.is_dir():
        yield {"repository": repository, "review_status": UNAVAILABLE}
        return

    @cache
    def package(directory):
        manifest = directory / "Cargo.toml"
        if manifest.is_file():
            with manifest.open("rb") as stream:
                name = tomllib.load(stream).get("package", {}).get("name")
            if name:
                return name, manifest.relative_to(root).as_posix()
        if directory == root:
            return repository, None
        return package(directory.parent)

    listing = subprocess.run(
        ["rg", "--files", "-0", "-g", "*.rs", "-g", "!target/**", "-g", "!vendor/**"],
        cwd=root, capture_output=True, check=False,
    )
    if listing.returncode != 1:  # rg reports an empty selection with exit 1.
        listing.check_returncode()
    files = os.fsdecode(listing.stdout).split("\0")
    for filename in sorted(filter(None, files)):
        parts = Path(filename).parts
        if any(part in {"benches", "fuzz", "examples", "verification"} for part in parts):
            continue
        owner, manifest = package((root / filename).parent)
        relative = Path(filename).relative_to(Path(manifest).parent) if manifest else Path(filename)
        # Section 7 includes the algebraic*.rs family and its split Rust modules.
        algebraic_module = (owner == "hypersolve" and len(relative.parts) >= 2
                            and relative.parts[0] == "src" and relative.parts[1].startswith("algebraic"))
        if "tests" in parts and not (algebraic_module and "tests" in relative.parts[2:]):
            continue
        seed = (
            algebraic_module or relative.as_posix() in SEED_FILES.get(owner, ())
            or (owner in SEED_DIRECTORIES and relative.is_relative_to(SEED_DIRECTORIES[owner]))
        )
        source = (root / filename).read_text()
        test_command = (f"cargo test --manifest-path {manifest} (select features after review)"
                        if manifest else "requires test mapping")
        line = 1
        previous = 0
        for match in DECLARATION.finditer(source):
            name = match["name"]
            visibility = (match["visibility"] or "private").strip()
            if not (
                seed or KEYWORD.search(name)
                or (owner in INTERVAL_PACKAGES and visibility == "pub")
                or (repository, filename, name) in reviewed_names
            ):
                continue
            semantics, action = disposition(repository, filename, name)
            position = match.start("name")
            line += source.count("\n", previous, position)
            previous = position
            yield dict(zip(FIELDS, [
                repository, filename, line,
                name, match["kind"], "requires body review", semantics,
                "requires body review", "requires manifest/feature review",
                visibility, "requires derive/codec review",
                "requires call-site review", owner, action,
                test_command, "requires benchmark mapping", CANDIDATE,
            ], strict=True))


def review_rows(path):
    if not path.exists():
        return []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream, delimiter="\t", strict=True)
        if reader.fieldnames is None:
            return []
        if reader.fieldnames != FIELDS:
            raise ValueError(f"unexpected inventory columns in {path}")
        reviews = []
        for row in reader:
            if None in row or None in row.values() or not row["review_status"]:
                raise ValueError(f"incomplete inventory row in {path} at line {reader.line_num}")
            if row["review_status"] not in (CANDIDATE, UNAVAILABLE):
                reviews.append(row)
        return reviews


def refresh(workspace, csgrs, output):
    output = output.resolve()
    # Include additional hand reviews from an out-of-place destination. A
    # matching name or line number alone cannot transfer a historical review.
    paths = ((csgrs / "interval-inventory.tsv").resolve(), output)
    paths = dict.fromkeys(paths)
    inputs = {path: review_rows(path) for path in paths}
    reviews = {tuple(row.values()): row for records in inputs.values() for row in records}
    reviewed_names = {(r["repository"], r["path"], r["symbol"]) for r in reviews.values()}
    checkouts = (("csgrs", csgrs), ("alumina-interface", workspace / "alumina-interface"),
                 ("alumina-firmware", workspace / "alumina-firmware"))
    count = 0
    with TemporaryDirectory(dir=output.parent) as directory:
        temporary = Path(directory) / "inventory.tsv"
        with temporary.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS, delimiter="\t", lineterminator="\n")
            writer.writeheader()
            for repository, root in checkouts:
                for row in scan(root, repository, reviewed_names):
                    writer.writerow(row)
                    count += 1
            writer.writerows(reviews.values())
        if output.exists():
            temporary.chmod(output.stat().st_mode & 0o777)
        for path, records in inputs.items():
            if review_rows(path) != records:
                raise RuntimeError(f"review records changed during scan of {path}; retry the refresh")
        temporary.replace(output)
    print(f"Wrote {count} scan rows and {len(reviews)} review records to {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, help="parent of the named checkouts")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    csgrs = (args.workspace / "csgrs" if args.workspace else Path(__file__).resolve().parents[1])
    refresh(csgrs.parent, csgrs, args.output)


if __name__ == "__main__":
    main()
