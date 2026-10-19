"""Independent native scalar and complex production denominators and line gates."""

import hashlib
from pathlib import Path


def native_source_groups(root):
    groups = {}
    for label, name in (("native_elliptic", "elliptic_integrals.rs"),
                        ("native_legendre", "legendre_integrals.rs"),
                        ("native_inverse_legendre", "inverse_legendre.rs"),
                        ("native_jacobi", "jacobi.rs")):
        sources = [root / "crates/hyperreal/src" / part / name
                   for part in ("computable/approximation", "computable/node", "real/arithmetic")]
        if label == "native_jacobi":
            sources.append(root / "crates/hyperreal/src/computable/node/log_argument.rs")
        groups[label] = sources
    groups["native_complex_elementary"] = [root / "crates/hyperlattice/src/complex/elementary.rs"]
    groups["native_complex_jacobi"] = [root / "crates/hyperlattice/src/complex/jacobi.rs"]
    groups["native_square_quotient"] = [root / "crates/hyperreal/src/computable/node/shared_square_quotient.rs"]
    groups["native_relations"] = [root / "crates/hyperreal/src/computable/node" / name
                                 for name in ("ring_native_relations.rs", "ring_native_reduction.rs",
                                              "ring_native_projection.rs", "ring_parameter_atoms.rs",
                                              "ring_native_division.rs")]
    return groups


def native_elliptic_coverage(export, root):
    groups = {}
    for label, sources in native_source_groups(root).items():
        files = {
            str(Path(file["filename"]).resolve().relative_to(root)): file["summary"]
            for dataset in export["data"] for file in dataset["files"]
            if Path(file["filename"]).resolve() in sources
        }
        if len(files) != len(sources):
            raise SystemExit(f"LLVM did not report every {label} implementation file")
        totals = {metric: {key: sum(file[metric][key] for file in files.values())
                           for key in ("count", "covered")}
                  for metric in ("lines", "regions", "functions")}
        for row in totals.values():
            if not row["count"]:
                raise SystemExit(f"LLVM reported an empty {label} denominator")
            row["percent"] = 100 * row["covered"] / row["count"]
        groups[label] = {
            "scope": f"{label} implementation files and listed helpers; shared dispatch and structural matching excluded",
            "files": files, "totals": totals,
            "source_sha256": {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in sources},
        }
    return groups


def check_native_elliptic_coverage(groups):
    # Keep the existing Carlson floor independent: new lines cannot mask a
    # regression in its denominator. Each later family gets its own floor.
    for label in ("native_elliptic", "native_legendre", "native_inverse_legendre", "native_jacobi",
                  "native_complex_jacobi", "native_square_quotient", "native_relations", "native_complex_elementary"):
        if groups[label]["totals"]["lines"]["percent"] < 95:
            raise SystemExit(f"{label} production line coverage is below the 95% gate")


def native_function_ranges(root):
    selectors = {
        "native_atan2": [
            ("crates/hyperreal/src/real/arithmetic/elementary_functions.rs", "    pub fn atan2(", "    /// Checked counterpart"),
            ("crates/hyperreal/src/real/arithmetic/elementary_functions.rs", "    pub fn try_atan2(", "    /// The hyperbolic sine"),
        ],
        "native_atan_deferred": [
            ("crates/hyperreal/src/computable/node/roots_inverse_hyperbolic.rs", "    fn atan_deferred(", "    fn asin_rational_deferred("),
        ],
    }
    groups = {}
    for label, entries in selectors.items():
        groups[label] = []
        for name, start_marker, end_marker in entries:
            path = root / name
            lines = path.read_text().splitlines()
            start = next(i for i, line in enumerate(lines, 1) if line.startswith(start_marker))
            end = next(i for i, line in enumerate(lines, 1) if i > start and line.startswith(end_marker))
            groups[label].append({"path": str(path), "start": start, "end_exclusive": end})
    return groups


def count_function_lines(annotated, groups, root):
    """Count each physical executable line once in every complete named function."""
    counts = {label: [dict(entry, measured={}) for entry in entries]
              for label, entries in groups.items()}
    paths = {entry["path"] for entries in groups.values() for entry in entries}
    active = None
    for line in annotated.splitlines():
        if line.endswith(":") and line[:-1] in paths:
            active = line[:-1]
            continue
        fields = line.split("|", 2)
        if active is None or len(fields) != 3 or not fields[0].strip().isdigit():
            continue
        number, count = int(fields[0].strip()), fields[1].strip()
        if not count:
            continue
        for entries in counts.values():
            for entry in entries:
                if entry["path"] == active and entry["start"] <= number < entry["end_exclusive"]:
                    entry["measured"][number] = count != "0"
    report = {}
    for label, entries in counts.items():
        files = {}
        for entry in entries:
            if not entry["measured"]:
                raise SystemExit(f"LLVM reported no {label} function executable lines")
            path = str(Path(entry["path"]).relative_to(root))
            row = {key: entry[key] for key in ("start", "end_exclusive")}
            row.update(count=len(entry["measured"]), covered=sum(entry["measured"].values()))
            files.setdefault(path, []).append(row)
        total = {key: sum(row[key] for rows in files.values() for row in rows)
                 for key in ("count", "covered")}
        total["percent"] = 100 * total["covered"] / total["count"]
        report[label] = {
            "scope": "complete named production functions; physical executable lines",
            "regions_functions_branches": "not measured separately for these function ranges",
            "files": files,
            "physical_executable_lines": total,
            "source_sha256": {path: hashlib.sha256((root / path).read_bytes()).hexdigest()
                              for path in files},
        }
    return report


def check_native_function_coverage(report):
    for label in ("native_atan2", "native_atan_deferred"):
        group = report[label]
        if (not group["files"] or any(not row["count"] for rows in group["files"].values() for row in rows)
                or group["physical_executable_lines"]["percent"] < 95):
            raise SystemExit(f"{label} production function line coverage is below the 95% gate or missing")
