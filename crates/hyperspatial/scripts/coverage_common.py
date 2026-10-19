"""Independent Carlson and Legendre production denominators and line gates."""

import hashlib
from pathlib import Path


def native_elliptic_coverage(export, root):
    groups = {}
    for label, name in (("native_elliptic", "elliptic_integrals.rs"),
                        ("native_legendre", "legendre_integrals.rs"),
                        ("native_inverse_legendre", "inverse_legendre.rs"),
                        ("native_jacobi", "jacobi.rs")):
        sources = [root / "crates/hyperreal/src" / part / name
                   for part in ("computable/approximation", "computable/node", "real/arithmetic")]
        if label == "native_jacobi":
            sources.append(root / "crates/hyperreal/src/computable/node/log_argument.rs")
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
            "scope": f"{name} implementation files and listed helpers; shared dispatch and structural matching excluded",
            "files": files, "totals": totals,
            "source_sha256": {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in sources},
        }
    return groups


def check_native_elliptic_coverage(groups):
    # Keep the existing Carlson floor independent: new lines cannot mask a
    # regression in its denominator. Each later family gets its own floor.
    for label in ("native_elliptic", "native_legendre", "native_inverse_legendre", "native_jacobi"):
        if groups[label]["totals"]["lines"]["percent"] < 95:
            raise SystemExit(f"{label} production line coverage is below the 95% gate")
