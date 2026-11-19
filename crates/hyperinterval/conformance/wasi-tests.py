#!/usr/bin/env python3
"""Run the scalar owner's original test executables with GMP/MPFR on WASI Preview 1."""

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys

from wasi_libtest import ROOT, RUNNER, TARGET, build_tests, compare_dispatch, completed, execute, run_replay_child, test_names

# Probe these cases individually. A trap in one guest must not prevent the
# remaining tests in its harness from running. If a probe succeeds on WASI
# it counts as a guest pass. Only a diagnosed missing OS facility permits
# running the exact original case on the host.
PROCESS = 'Error { kind: Unsupported, message: "operation not supported on this platform" }'
REPLAYS = {
    "dyadic_zero_grid": ("fresh_process_replay_then_operation", "HYPERINTERVAL_ZERO_GRID_REPLAY"),
    "fractional_power": ("exact_replay_then_further_power_in_fresh_process", "HYPERINTERVAL_POWER_REPLAY"),
    "integer_division": ("replayed_output_feeds_widening_and_further_division", "HYPERINTERVAL_INTEGER_DIV_REPLAY"),
    "split_division": ("replay_in_fresh_process", "HYPERINTERVAL_SPLIT_REPLAY_PAYLOAD"),
    "tangent": ("replay_then_another_pole_operation_in_fresh_process", "HYPERINTERVAL_TANGENT_REPLAY"),
}
OS_PROBES = {
    **{suite: (case, PROCESS) for suite, (case, _) in REPLAYS.items()},
    "rational": ("product_dispatch_and_reducer_counts_match_baseline", PROCESS),
    "mpfr_cache_lifecycle": ("mpfr_oracle_caches_release_on_normal_and_unwinding_thread_exit", "failed to spawn thread:"),
    "real_bridge": ("mpfr::real_bridges_enclose_independently_directed_mpfr_bounds", "failed to spawn thread:"),
    "real_dyadic_lifecycle": ("concurrent_readers_share_the_translated_root_while_a_cancelled_clone_retries", "failed to spawn thread:"),
}


def missing_os_facility(transcript, target, reason):
    panic = re.search(r"^thread '[^']+'[^\n]* panicked at (.+):(\d+):\d+:\n([^\n]+)", transcript, re.MULTILINE)
    if not panic or "RuntimeError: unreachable" not in transcript:
        return False
    source, line, message = panic.groups()
    if "kind: Unsupported" not in message or reason not in message:
        return False
    if reason != PROCESS:
        return "/library/std/src/thread/" in source
    path = Path("crates/hyperinterval/tests") / f"{target}.rs"
    call = "replay_binary::path()" if target in REPLAYS else "current_exe()"
    return source.endswith(str(path)) and call in (ROOT / path).read_text().splitlines()[int(line) - 1]


def replay_child(args):
    case, key = REPLAYS[os.environ["SCALAR_WASI_SUITE"]]
    if args[args.index("--exact") + 1] != case:
        raise SystemExit(f"Unexpected scalar replay child: {args}")
    run_replay_child(os.environ["SCALAR_WASI_GUEST"], args, case,
                     ["--env", key, os.environ[key]], Path(os.environ["SCALAR_WASI_RECEIPT"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default="stable")
    parser.add_argument("--cc", required=True, help="WASI-capable Clang executable")
    parser.add_argument("--sysroot", type=Path, help="WASI sysroot when not built into Clang")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    compiler = [args.cc, "--target=wasm32-wasi"]
    if args.sysroot:
        compiler.append(f"--sysroot={args.sysroot.resolve()}")
    signal = Path(subprocess.check_output([*compiler, "-print-file-name=libwasi-emulated-signal.a"], text=True).strip())
    if not signal.is_absolute() or not signal.is_file():
        raise SystemExit("The WASI C compiler must resolve libwasi-emulated-signal.a in its sysroot.")

    host_env = os.environ | {"CARGO_TARGET_DIR": str(output / "target")}
    host_env.pop("HYPERINTERVAL_REPLAY_DRIVER", None)
    guest_env = host_env | {
        "CC": shlex.join(compiler), "CC_FOR_BUILD": os.environ.get("CC_FOR_BUILD", "cc"),
        "CPPFLAGS": os.environ.get("CPPFLAGS", "") + " -D_WASI_EMULATED_SIGNAL",
        "LIBS": os.environ.get("LIBS", "") + " -lwasi-emulated-signal",
        "GMP_MPFR_SYS_CACHE": str(output / "gmp-cache"),
    }
    flags = guest_env.get("CARGO_ENCODED_RUSTFLAGS")
    if flags is None:
        flags = "\x1f".join(shlex.split(guest_env.get("RUSTFLAGS", "")))
    guest_env["CARGO_ENCODED_RUSTFLAGS"] = "\x1f".join(filter(None, [flags, "-C", f"link-arg={signal}"]))

    cargo = ["cargo", f"+{args.toolchain}"]
    version = subprocess.check_output(["rustc", f"+{args.toolchain}", "-vV"], text=True)
    (output / "rustc.txt").write_text(version)
    # Hyperreal-backed features have a separate minimum compiler.
    metadata = json.loads(subprocess.check_output([*cargo, "metadata", "--locked", "--no-deps",
                                                   "--format-version", "1"], cwd=ROOT, text=True))
    minimum = next(p["rust_version"] for p in metadata["packages"] if p["name"] == "hyperreal")
    release = re.search(r"^release: (\d+\.\d+\.\d+)", version, re.MULTILINE)
    supports_real = tuple(map(int, release[1].split("."))) >= tuple(map(int, minimum.split(".")))
    features = ["--all-features"] if supports_real else [
        "--no-default-features", "--features", "integer,fixed,dyadic,float-proposal,float-enclosure-wasip1,serde,dispatch-trace"]
    cargo += ["test", "--locked", "--release", "-p", "hyperinterval", *features]
    guests = build_tests(cargo, ["--target", TARGET, "--lib", "--tests"], guest_env)
    host_cases = {}
    report = {"guest": {}, "host_required": {}, "fresh_replays": {}}
    for target, binary in sorted(guests.items()):
        guest = ["node", str(RUNNER), binary]
        names = test_names(guest)
        case, reason = OS_PROBES.get(target, (None, None))
        probe = {case} & names
        if target in REPLAYS and not probe:
            raise SystemExit(f"Missing original replay test {target}::{case}")
        options = ["--exact", "--test-threads=1", "--nocapture"]
        log = output / f"{target}.log"
        run = execute([*guest, *options, *(["--skip", case] if probe else [])], log)
        if run.returncode or not completed(run.stdout, names - probe):
            raise SystemExit(f"Guest test failure or inventory mismatch in {log}\n{run.stdout}")
        passed = names - probe
        if probe:
            log = output / f"{target}-probe.log"
            run = execute([*guest, *options, case], log)
            if not run.returncode and completed(run.stdout, probe):
                passed.add(case)
            elif run.returncode and f"test {case} ..." in run.stdout and missing_os_facility(run.stdout, target, reason):
                host_cases[target] = case
            else:
                raise SystemExit(f"Unexpected guest probe result in {log}\n{run.stdout}")
        report["guest"][target] = sorted(passed)

    required = {
        "arctangent": "directed_mpfr_across_reductions_and_scales",
        "dyadic_bisection": "both_children_match_integer_midpoint_and_preserve_the_source",
        "trigonometry": "periodic_extrema_and_exact_sources_match_mpfr",
    }
    if supports_real:
        required["exponential"] = "rational_exp_preserves_exact_source_and_matches_mpfr"
    for target, case in required.items():
        if case not in report["guest"].get(target, []):
            raise SystemExit(f"Missing required guest oracle {target}::{case}")
    host_targets = set(host_cases)
    if supports_real:
        host_targets.add("rational")
    if host_targets:
        selection = [arg for target in sorted(host_targets) for arg in ("--test", target)]
        hosts = build_tests(cargo, selection, host_env)
        if supports_real:
            case = OS_PROBES["rational"][0]
            report["guest_dispatch"] = {"rational": compare_dispatch(
                guests["rational"], hosts["rational"], case, "HYPERINTERVAL_TRACE_BACKEND", "TRACE", 450, output)}
        for target, case in host_cases.items():
            log = output / f"{target}-host.log"
            run = execute([hosts[target], "--exact", case, "--nocapture"], log)
            if run.returncode or not completed(run.stdout, {case}):
                raise SystemExit(f"Native OS-dependent test failure in {log}\n{run.stdout}")
            report["host_required"][target] = case
    if not REPLAYS.keys() <= guests.keys():
        raise SystemExit("Missing scalar replay executables")
    parents = build_tests(cargo, [arg for suite in REPLAYS for arg in ("--test", suite)],
                          host_env | {"HYPERINTERVAL_REPLAY_DRIVER": str(Path(__file__).resolve())})
    for suite, (case, key) in REPLAYS.items():
        receipt = output / f"{suite}.children"
        env = host_env | {"SCALAR_WASI_SUITE": suite, "SCALAR_WASI_GUEST": guests[suite],
                          "SCALAR_WASI_RECEIPT": str(receipt)}
        env.pop(key, None)
        log = output / f"{suite}-fresh-replay.log"
        run = execute([parents[suite], "--exact", case, "--nocapture"], log, env)
        if run.returncode or not completed(run.stdout, {case}) or not receipt.is_file() or receipt.read_text().splitlines() != [case]:
            raise SystemExit(f"Incomplete native-parent/WASI-child replay for {suite}::{case}. See {log}")
        report["fresh_replays"][suite] = case
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"{sum(map(len, report['guest'].values()))} scalar tests passed in WASI. "
          f"{len(host_cases)} required and passed on the native host. "
          f"{len(report['fresh_replays'])} parent-produced sources replayed in fresh WASI guests.")


if __name__ == "__main__":
    if "--exact" in sys.argv[1:]:
        replay_child(sys.argv[1:])
    else:
        main()
