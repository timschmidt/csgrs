#!/usr/bin/env python3
"""Lexicase search over independent scalar-family buckets; development only."""

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import random
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[3]
CRATE = ROOT / "crates/hyperspatial"
METRICS = ("evaluation_ns", "dispatch_count", "steps")


def run_text(*command):
    return subprocess.check_output(command, cwd=ROOT, text=True).strip()


def select_worst(rows, rng):
    """Exact lexicase: shuffle cases/metrics, keep maxima, break ties randomly."""
    order = list(METRICS)
    rng.shuffle(order)
    survivors = rows[:]
    filters = []
    for metric in order:
        maximum = max(row[metric] for row in survivors)
        survivors = [row for row in survivors if row[metric] == maximum]
        filters.append({"metric": metric, "maximum": maximum,
                        "survivors": [row["case"] for row in survivors]})
        if len(survivors) == 1:
            break
    return rng.choice(survivors), filters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "target/hyperspatial-directed")
    parser.add_argument("--seed", type=int, default=20261018)
    parser.add_argument("--workload", choices=("normal_offset", "mercator", "inverse_isometric", "ellipsoidal_mercator", "transverse_mercator", "elliptic", "inverse_legendre", "jacobi", "complex_jacobi", "jacobi_epsilon", "complex_atanh", "parameter_encodings"), default="normal_offset")
    parser.add_argument("--candidates", type=int, default=4)
    parser.add_argument("--generations", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=15)
    args = parser.parse_args()
    if args.candidates < 2 or args.generations < 1 or args.timeout <= 0:
        parser.error("use at least two candidates, one generation and a positive timeout")
    args.output.mkdir(parents=True, exist_ok=True)
    corpus = args.output / "corpus"
    corpus.mkdir(exist_ok=True)
    build = ["cargo", "build", "--locked", "--release", "-p", "hyperspatial",
             "--example", "e1_probe", "--features", "dispatch-trace"]
    subprocess.run(build, cwd=ROOT, check=True)
    metadata = json.loads(run_text("cargo", "metadata", "--no-deps", "--format-version", "1"))
    executable = Path(metadata["target_directory"]) / "release/examples/e1_probe"
    source_files = sorted((CRATE / "src").glob("*.rs")) + [
        CRATE / "dev/normal_offset.rs", CRATE / "dev/mercator.rs", CRATE / "dev/ellipsoidal_mercator.rs", CRATE / "dev/inverse_isometric.rs", CRATE / "dev/transverse_mercator.rs", CRATE / "dev/elliptic.rs", CRATE / "dev/inverse_legendre.rs", CRATE / "dev/jacobi.rs", CRATE / "dev/complex_jacobi.rs", CRATE / "dev/jacobi_epsilon.rs", CRATE / "dev/complex_atanh.rs", CRATE / "dev/parameter_encodings.rs",
        CRATE / "examples/e1_probe.rs",
        Path(__file__).resolve(), CRATE / "Cargo.toml", ROOT / "Cargo.lock",
        ROOT / "crates/hyperreal/tests/support/representations.rs",
    ] + sorted((ROOT / "crates/hyperreal/src").rglob("*.rs")) + sorted((ROOT / "crates/hyperlattice/src").rglob("*.rs"))
    report = {
        "schema": 1, "seed": args.seed, "workload": args.workload, "build": build,
        "compiler": run_text("rustc", "-vV"), "platform": platform.platform(),
        "processor": platform.processor(), "source_revision": run_text("git", "rev-parse", "HEAD"),
        "build_environment": {key: os.environ.get(key) for key in
                              ("RUSTUP_TOOLCHAIN", "RUSTFLAGS", "CARGO_TARGET_DIR")},
        "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in source_files},
        "features": ["dispatch-trace", "dev:hyperreal/serde"],
        "cache": ("fresh process per case; optional display warming from byte 5 "
                  + ("bit 2" if args.workload in ("mercator", "ellipsoidal_mercator", "transverse_mercator", "elliptic", "inverse_legendre", "jacobi", "complex_jacobi", "jacobi_epsilon", "complex_atanh", "parameter_encodings") else "bit 1")
                  + "; scalar global caches follow native defaults"),
        "policy": "STRICT", "families": 22,
        "candidates_per_family": args.candidates, "generations": args.generations,
        "timeout_seconds_per_process": args.timeout,
        "timing": "one full independently checked workflow including recipe construction and trace instrumentation; excludes process startup",
        "selection": "independent first-scalar family buckets; shuffled exact-maximum lexicase over " + ", ".join(METRICS),
        "limitations": "Cost-directed discovery, not a comparative performance result. Timing is noisy; processes have a wall timeout but no new native heap bound. ASan replay is separate.",
        "selections": [], "failures": [], "executions": 0,
    }
    rng = random.Random(args.seed)
    parents = {}
    start = time.monotonic()
    # Raw traces retain every observation, including failures and timeouts.
    with gzip.open(args.output / "observations.jsonl.gz", "wt", encoding="utf-8") as raw:
        for generation in range(args.generations):
            for family in range(22):
                rows = []
                for candidate in range(args.candidates):
                    parent = parents.get(family)
                    if parent is None:
                        data = bytes([family, rng.randrange(22), rng.randrange(256),
                                      rng.randrange(256), rng.randrange(8), rng.randrange(4)])
                    else:
                        mutated = bytearray.fromhex(parent["case"])
                        if candidate:
                            index = rng.randrange(1, 6)  # Preserve the independent family bucket.
                            mutated[index] = rng.randrange(256)
                        data = bytes(mutated)
                    encoded = data.hex()
                    try:
                        process = subprocess.run([str(executable), encoded, args.workload], cwd=ROOT,
                                                 capture_output=True, text=True, timeout=args.timeout)
                        try:
                            row = json.loads(process.stdout)
                        except json.JSONDecodeError:
                            row = {"status": "invalid_probe_output", "stdout": process.stdout}
                        row.update(returncode=process.returncode, stderr=process.stderr)
                    except subprocess.TimeoutExpired as error:
                        row = {"status": "timeout", "stdout": (error.stdout or b"").decode(errors="replace"),
                               "stderr": (error.stderr or b"").decode(errors="replace")}
                    row.update(case=encoded, generation=generation, family=family, candidate=candidate)
                    report["executions"] += 1
                    raw.write(json.dumps(row, sort_keys=True) + "\n")
                    if row["status"] == "certified_completion" and row.get("returncode") == 0:
                        rows.append(row)
                    else:
                        report["failures"].append(row)
                        (args.output / ("failure-" + encoded)).write_bytes(data)
                if rows:
                    winner, filters = select_worst(rows, rng)
                    parents[family] = winner
                    report["selections"].append({"generation": generation, "family": family,
                                                 "families": winner["families"], "case": winner["case"],
                                                 "scores": {k: winner[k] for k in METRICS}, "filters": filters})
                    (corpus / winner["case"]).write_bytes(bytes.fromhex(winner["case"]))
                print(f"generation={generation} family={family} completed={len(rows)}/{args.candidates}", flush=True)
    report["elapsed_seconds"] = time.monotonic() - start
    report["selected_corpus"] = sorted(p.name for p in corpus.iterdir() if p.is_file())
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return 1 if report["failures"] else 0


if __name__ == "__main__":
    sys.exit(main())
