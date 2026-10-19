#!/usr/bin/env python3
"""Seed both E1 fuzz domains and run bounded, address-sanitized campaigns."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[3]
CRATE = ROOT / "crates/hyperspatial"


def snapshot(directory):
    return {p.name: p.read_bytes().hex() for p in sorted(directory.iterdir()) if p.is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "target/hyperspatial-asan")
    parser.add_argument("--selected-corpus", type=Path)
    parser.add_argument("--seconds", type=int, default=60)
    parser.add_argument("--seed", type=int, default=20261018)
    parser.add_argument("--toolchain", default="nightly")
    args = parser.parse_args()
    if args.seconds < 1:
        parser.error("positive campaign duration required")
    args.output.mkdir(parents=True, exist_ok=True)
    report = {
        "seed": args.seed, "sanitizer": "address", "max_seconds_per_campaign": args.seconds,
        "seconds_per_input": 10, "rss_limit_mib": 2048, "max_input_bytes": 6,
        "compiler": subprocess.check_output(["rustc", "+" + args.toolchain, "-vV"], text=True),
        "cargo_fuzz": subprocess.check_output(["cargo", "+" + args.toolchain, "fuzz", "--version"], text=True),
        "source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted((CRATE / "src").glob("*.rs")) +
                          sorted((CRATE / "fuzz/fuzz_targets").glob("*.rs")) +
                          [CRATE / "dev/normal_offset.rs", CRATE / "fuzz/Cargo.lock",
                           Path(__file__).resolve(),
                           ROOT / "crates/hyperreal/tests/support/representations.rs"]},
        "limitations": "External timeout/RSS containment, not native resource bounds. Counter/feature coverage is for the instrumented binary, not production line/branch coverage.",
        "campaigns": [],
    }
    failed = False
    for target in ("forward_local", "normal_offset"):
        corpus = args.output / (target + "-corpus")
        corpus.mkdir(exist_ok=True)
        if target == "forward_local":
            seeds = [bytes([a, b, latitude, longitude, axes, 3])
                     for a, b in [(4, 2), (2, 4), (3, 3)]
                     for latitude in range(9) for longitude in range(17) for axes in range(2)]
        else:
            seeds = [bytes([family, (family + 7) % 22, 0, 0, 0, flags])
                     for family in range(22) for flags in (0, 3)]
            if args.selected_corpus:
                seeds += [p.read_bytes() for p in sorted(args.selected_corpus.iterdir()) if p.is_file()]
        for seed in seeds:
            (corpus / seed.hex()).write_bytes(seed)
        command = ["cargo", "+" + args.toolchain, "fuzz", "run", target,
                   "--fuzz-dir", "crates/hyperspatial/fuzz", "--sanitizer", "address", str(corpus),
                   "--", f"-max_total_time={args.seconds}", "-timeout=10", "-rss_limit_mb=2048",
                   "-max_len=6", f"-seed={args.seed}"]
        campaign = {"target": target, "command": command, "input_corpus": snapshot(corpus)}
        artifact_dir = CRATE / "fuzz/artifacts" / target
        campaign["preexisting_artifacts"] = snapshot(artifact_dir) if artifact_dir.exists() else {}
        print(" ".join(command), flush=True)
        with (args.output / (target + ".log")).open("w") as log:
            try:
                result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                        timeout=1200 + args.seconds)
                campaign["returncode"] = result.returncode
            except subprocess.TimeoutExpired:
                campaign["returncode"] = "outer_timeout"
        campaign["output_corpus"] = snapshot(corpus)
        campaign["artifacts"] = snapshot(artifact_dir) if artifact_dir.exists() else {}
        report["campaigns"].append(campaign)
        failed |= campaign["returncode"] != 0
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        if failed:
            break
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
