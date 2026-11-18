#!/usr/bin/env python3
"""Measure all-feature native library and integration test coverage for an interval plan package."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


WORKSPACE = Path(__file__).resolve().parents[1]
TEST_MODULES = {
    "hyperinterval": {"src/dyadic_quotient_tests.rs", "src/ieee_tests.rs"},
    "hyperlattice-interval": {"src/tests.rs"},
    "hyperdynamics": set(),
}
METRICS = ("lines", "regions", "functions", "branches")


def summarize(export, crate, sources, test_modules):
    files = {
        Path(row["filename"]).relative_to(crate).as_posix(): row["summary"]
        for dataset in export["data"] for row in dataset["files"]
    }
    # lib.rs declares modules and reexports. Include any records it acquires.
    missing = sources - files.keys()
    extra = files.keys() - sources
    if missing - {"src/lib.rs"} or extra:
        raise ValueError(f"source inventory differs from LLVM: {missing=}, {extra=}")
    production = {name: row for name, row in files.items() if name not in test_modules}
    totals = {
        metric: {key: sum(row[metric][key] for row in production.values())
                 for key in ("count", "covered")}
        for metric in METRICS
    }
    if any(not totals[key]["count"] for key in ("lines", "regions", "functions")):
        raise ValueError("empty production coverage denominator")
    return {"files": files, "without_records": sorted(missing),
            "excluded_test_modules": sorted(test_modules), "production": totals}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", choices=sorted(TEST_MODULES), default="hyperinterval")
    package = parser.parse_args().package
    crate = WORKSPACE / "crates" / package
    test_modules = TEST_MODULES[package]
    rustc = subprocess.check_output(["rustc", "-vV"], text=True)
    host = next(line.removeprefix("host: ") for line in rustc.splitlines() if line.startswith("host: "))
    sysroot = subprocess.check_output(["rustc", "--print", "sysroot"], text=True).strip()
    llvm = Path(sysroot) / "lib/rustlib" / host / "bin"
    suffix = ".exe" if os.name == "nt" else ""
    cov, profdata = (llvm / (name + suffix) for name in ("llvm-cov", "llvm-profdata"))
    if not cov.is_file() or not profdata.is_file():
        sys.exit("Install llvm-tools-preview for the active Rust toolchain.")
    target = Path(os.environ.get("CARGO_TARGET_DIR", WORKSPACE / "target")).resolve()
    target.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=f"{package}-coverage-", dir=target))
    print(f"Coverage artifacts in {output}", flush=True)
    env = dict(os.environ, CARGO_TARGET_DIR=str(target))
    # Keep mappings for cross-crate inline functions together with the counters.
    # The default link can leave zero-hash records in individual test binaries.
    coverage_flags = ["-C", "instrument-coverage", "-C", "link-dead-code=yes", "-C", "codegen-units=1"]
    if "CARGO_ENCODED_RUSTFLAGS" in env:
        env["CARGO_ENCODED_RUSTFLAGS"] = "\x1f".join(
            filter(None, [env["CARGO_ENCODED_RUSTFLAGS"], *coverage_flags])
        )
    else:
        env["RUSTFLAGS"] = " ".join([env.get("RUSTFLAGS", ""), *coverage_flags]).strip()
    cargo = ["cargo", "test", "--locked", "-p", package, "--target", host,
             "--all-features", "--lib", "--tests"]
    with tempfile.TemporaryDirectory(dir=output) as profiles:
        env["LLVM_PROFILE_FILE"] = str(Path(profiles) / "%p-%m.profraw")
        manifest = subprocess.check_output(cargo + ["--no-run", "--message-format=json-render-diagnostics"],
                                           cwd=crate, env=env, text=True)
        objects = sorted({row["executable"] for row in map(json.loads, manifest.splitlines())
                          if row["reason"] == "compiler-artifact" and row["executable"] and row["profile"]["test"]})
        if not objects:
            sys.exit("Cargo returned no test executables.")
        subprocess.run(cargo, cwd=crate, env=env, check=True)
        raw = sorted(Path(profiles).glob("*.profraw"))
        if not raw:
            sys.exit("No test profiles were produced.")
        merged = output / "merged.profdata"
        merge = subprocess.run([profdata, "merge", "--failure-mode=any", "-sparse", *raw, "-o", merged],
                               stderr=subprocess.PIPE, text=True, check=True)
        if merge.stderr:
            sys.exit(merge.stderr)

    sources = sorted((crate / "src").rglob("*.rs"))
    args = [objects[0], *(arg for obj in objects[1:] for arg in ("--object", obj)),
            f"--instr-profile={merged}"]
    result = subprocess.run([cov, "export", "--summary-only", *args, *map(str, sources)], capture_output=True,
                            text=True, check=True)
    source_names = {p.relative_to(crate).as_posix() for p in sources}
    summary = summarize(json.loads(result.stdout), crate, source_names, test_modules)
    summary.update(rustc=rustc, command=cargo, objects=objects, llvm_diagnostics=result.stderr,
                   flags=env.get("CARGO_ENCODED_RUSTFLAGS", env.get("RUSTFLAGS")), profiles=len(raw))
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Production coverage ({host}, all features)")
    for metric, row in summary["production"].items():
        value = f"{row['covered']}/{row['count']}" if row["count"] else "unmeasured"
        print(f"  {metric}: {value}")
    if result.stderr:
        sys.stderr.write(result.stderr)
        sys.exit("LLVM emitted diagnostics. Inspect the profiles before accepting coverage.")
    if any(row["covered"] != row["count"] for row in summary["production"].values()):
        production_sources = [str(crate / name) for name in sorted(summary["files"])
                              if name not in test_modules]
        branch_detail = ["--show-branches=count"] if summary["production"]["branches"]["count"] else []
        with (output / "uncovered.txt").open("w", encoding="utf-8") as report:
            subprocess.run([cov, "show", "--show-line-counts-or-regions",
                            *branch_detail, *args, *production_sources], stdout=report, check=True)
        sys.exit("Production coverage is below 100% on the instrumented metrics.")


if __name__ == "__main__":
    main()
