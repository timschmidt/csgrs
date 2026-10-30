#!/usr/bin/env python3
"""Run the reviewed Hypercurve capability witnesses, preserving typed gaps.

Each row names one ordinary integration test. A passing blocker assertion is
reported as that blocker, never as geometric completion. Output must be new and
outside the source tree. This Unix driver uses only Python's standard library.
"""

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CATALOG = ROOT / "crates/hypercurve/qualification/capabilities.json"
STAGES = (
    "representation", "evaluation", "differentials", "incidence_order",
    "classification", "region_admission", "booleans", "offsets", "repeated_offsets",
    "corner_edits", "restriction_reconstruction", "transforms", "exact_replay",
    "resource_recovery",
)
OUTCOMES = {
    "exact_success", "successful_empty", "proved_no_solution", "invalid",
    "underconstrained", "unsupported", "undecided", "exhausted", "unbounded",
    "qualified_approximation", "blocked_unspecified",
}
FAMILIES = {
    "Line", "CircularArc", "QuadraticBezier", "CubicBezier",
    "RationalQuadraticBezier", "RationalBezier", "PolynomialBSpline", "Nurbs",
    "AnalyticParallel", "ImplicitPolynomial",
}
FIELDS = {
    "id", "target", "test", "owner", "properties", "domain", "origins",
    "families", "policies", "semantics", "claims", "oracle", "timeout_seconds",
}
SAFE_NAME = re.compile(r"[a-z][a-z0-9_]*\Z")
SUMMARY = re.compile(
    r"^test result: ok\. 1 passed; 0 failed; 0 ignored; 0 measured; "
    r"\d+ filtered out; finished in .+$", re.MULTILINE,
)
spec = importlib.util.spec_from_file_location(
    "successive_evidence", ROOT / "scripts/successive-operations.py"
)
evidence = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evidence)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def nonempty(value):
    return isinstance(value, str) and bool(value.strip())


def strings(value):
    return isinstance(value, list) and bool(value) and all(map(nonempty, value))


def unique_keys(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f"duplicate JSON key: {key}")
        result[key] = value
    return result


def load_catalog(path):
    return parse_catalog(path.read_bytes())


def parse_catalog(content):
    catalog = json.loads(content, object_pairs_hook=unique_keys)
    require(isinstance(catalog, dict) and set(catalog) == {
        "schema_version", "scope", "target_domain", "unqualified_targets", "fixtures",
    }, "unexpected catalog fields")
    require(type(catalog["schema_version"]) is int and catalog["schema_version"] == 1,
            "unsupported catalog schema")
    for key in ["scope", "target_domain"]:
        require(nonempty(catalog[key]), f"missing {key}")
    require(strings(catalog["unqualified_targets"]), "unqualified targets must be explicit")
    require(isinstance(catalog["fixtures"], list) and catalog["fixtures"], "empty catalog")
    ids, tests = set(), set()
    for row in catalog["fixtures"]:
        require(isinstance(row, dict) and set(row) == FIELDS, "unexpected fixture fields")
        for key in ["id", "target"]:
            require(isinstance(row[key], str) and SAFE_NAME.fullmatch(row[key]), f"invalid {key}")
        for key in ["test", "owner", "domain", "semantics", "oracle"]:
            require(nonempty(row[key]), f"missing {key} in {row['id']}")
        require(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_:]*", row["test"]), "invalid test name")
        for key in ["properties", "origins", "families", "policies"]:
            require(strings(row[key]), f"missing {key} in {row['id']}")
        require(set(row["origins"]) <= {"authored", "generated", "selected_implicit"},
                "unknown origin")
        require(set(row["policies"]) <= {"STRICT", "APPROXIMATE_512"}, "unknown policy")
        require(set(row["families"]) <= FAMILIES, "unknown curve family")
        require(all(re.fullmatch(r"P(?:[1-9]|[12][0-9]|3[0-9])", p)
                    for p in row["properties"]), "unknown property")
        require(type(row["timeout_seconds"]) is int and row["timeout_seconds"] > 0,
                "fixture timeout must be a positive integer")
        require(row["id"] not in ids, "duplicate fixture id")
        require((row["target"], row["test"]) not in tests, "duplicate test witness")
        ids.add(row["id"]); tests.add((row["target"], row["test"]))
        require(isinstance(row["claims"], list) and row["claims"], "empty claims")
        for claim in row["claims"]:
            require(isinstance(claim, dict) and set(claim) == {"stage", "outcome", "assertion"},
                    "unexpected claim fields")
            require(isinstance(claim["stage"], str) and claim["stage"] in STAGES, "unknown stage")
            require(isinstance(claim["outcome"], str) and claim["outcome"] in OUTCOMES,
                    "unknown geometric outcome")
            require(nonempty(claim["assertion"]), "missing assertion scope")
    return catalog


def command_run(command, log_path, timeout, environment):
    """Contain a process group; timeout/interruption never implies exhaustion."""
    start = time.monotonic()
    with log_path.open("w") as log:
        process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        status = "exited"
        try:
            code = process.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
            status = "timeout" if isinstance(error, subprocess.TimeoutExpired) else "interrupted"
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                code = process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                code = process.wait()
            # A child can outlive a leader that exited on SIGTERM.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            if status == "interrupted":
                raise
    return {"command": command, "status": status, "returncode": code,
            "elapsed_seconds": time.monotonic() - start, "log": log_path.name,
            "log_sha256": evidence.digest_file(log_path)}


def registered_tests(text):
    return {line.removesuffix(": test") for line in text.splitlines() if line.endswith(": test")}


def witness_passed(result, output):
    # A misspelled --exact filter exits zero with zero tests. Require one test,
    # one summary and no ignored test, independently of the process exit code.
    summaries = [line for line in output.splitlines() if line.startswith("test result:")]
    return (result["status"] == "exited" and result["returncode"] == 0
            and len(summaries) == 1 and bool(SUMMARY.fullmatch(summaries[0])))


def build_binaries(rows, output, environment, timeout):
    targets = sorted({row["target"] for row in rows})
    command = ["cargo", "test", "--locked", "-p", "hypercurve", "--release",
               "--all-features", "--no-run", "--message-format=json"]
    for target in targets:
        command += ["--test", target]
    result = command_run(command, output / "build.log", timeout, environment)
    binaries = {}
    if result["status"] == "exited" and result["returncode"] == 0:
        with (output / "build.log").open() as log:
            for line in log:
                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if message.get("reason") != "compiler-artifact" or not message.get("executable"):
                    continue
                target = message["target"]
                if target["name"] not in targets or target["kind"] != ["test"]:
                    continue
                path = Path(message["executable"]).resolve()
                require(target["name"] not in binaries or binaries[target["name"]] == path,
                        "ambiguous Cargo test artifact")
                binaries[target["name"]] = path
    result["all_requested_artifacts"] = set(binaries) == set(targets)
    return result, binaries


def run_fixture(row, binary, registered, ignored, output, environment, expected_digest):
    result = {"id": row["id"], "status": "unregistered", "witness": row}
    if row["test"] not in registered:
        return result
    if row["test"] in ignored:
        result["status"] = "ignored"
        return result
    before = evidence.digest_file(binary)
    if before != expected_digest:
        result["status"] = "binary_changed"
        return result
    run = command_run([str(binary), row["test"], "--exact", "--nocapture", "--test-threads=1"],
                      output / (row["id"] + ".log"), row["timeout_seconds"], environment)
    result.update(run=run, binary_sha256=before)
    if not binary.exists() or evidence.digest_file(binary) != before:
        result["status"] = "binary_changed"
    elif run["status"] != "exited":
        result["status"] = run["status"]
    else:
        result["status"] = ("witness_passed" if witness_passed(
            run, (output / run["log"]).read_text()) else "test_failed")
    return result


def stage_matrix(results, stable):
    return [{"id": row["id"], "stages": {
        stage: ([claim for claim in row["witness"]["claims"] if claim["stage"] == stage]
                if stable and row["status"] == "witness_passed" else [])
        for stage in STAGES
    }} for row in results]


def run(args):
    catalog_path = args.catalog.resolve()
    # One capture owns the parsed claims, digest and archived evidence.
    catalog_bytes = catalog_path.read_bytes()
    catalog_digest = hashlib.sha256(catalog_bytes).hexdigest()
    catalog = parse_catalog(catalog_bytes)
    rows = catalog["fixtures"]
    if args.fixture:
        require(set(args.fixture) <= {row["id"] for row in rows}, "unknown requested fixture")
        rows = [row for row in rows if row["id"] in args.fixture]
    output = evidence.new_output(args.output)
    before, files, patch = evidence.source_snapshot()
    evidence.save_json(output / "source-files.json", files)
    (output / "dirty.patch").write_bytes(patch)
    for relative in before["untracked_inputs"]:
        source = ROOT / relative
        destination = output / "untracked" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if source.is_symlink():
            destination.symlink_to(os.readlink(source))
        else:
            shutil.copy2(source, destination)
    (output / "catalog.json").write_bytes(catalog_bytes)
    environment = {**os.environ, "RUST_TEST_THREADS": "1", "CARGO_TERM_COLOR": "never"}
    report = {
        "schema_version": 1, "status": "running", "source_before": before,
        "catalog_sha256": catalog_digest, "selected_ids": [row["id"] for row in rows],
        "scope": catalog["scope"], "target_domain": catalog["target_domain"],
        "unqualified_targets": catalog["unqualified_targets"], "fixtures": [],
        "compiler": subprocess.check_output(["rustc", "-Vv"], env=environment, text=True),
        "environment": {name: environment[name] for name in [
            "RUSTUP_TOOLCHAIN", "RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS", "RUSTC",
            "RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "CARGO_TARGET_DIR", "CARGO_BUILD_JOBS",
            "RUST_TEST_THREADS", "CARGO_TERM_COLOR",
        ] if name in environment},
        "resource_contract": "External per-process elapsed-time containment; no total work or memory bound. Timeout is incomplete, not typed geometric exhaustion.",
    }
    evidence.save_json(output / "report.json", report)
    binaries, binary_digests = {}, {}
    try:
        build, binaries = build_binaries(rows, output, environment, args.build_timeout)
        report["build"] = build
        require(build["all_requested_artifacts"], "build did not produce every requested test binary")
        listings = {}
        for target, binary in binaries.items():
            digest = evidence.digest_file(binary)
            binary_digests[target] = digest
            names = []
            for suffix, options in [("registered", []), ("ignored", ["--ignored"])]:
                log = output / f"{target}-{suffix}.log"
                result = command_run([str(binary), "--list", "--format", "terse", *options],
                                     log, 30, environment)
                require(result["status"] == "exited" and result["returncode"] == 0,
                        "failed test discovery")
                names.append(registered_tests(log.read_text()))
            require(evidence.digest_file(binary) == digest, "binary changed during discovery")
            listings[target] = names
        report["binaries"] = {target: {"path": str(binary), "sha256": binary_digests[target]}
                              for target, binary in binaries.items()}
        for row in rows:
            report["active_fixture"] = row["id"]
            evidence.save_json(output / "report.json", report)
            registered, ignored = listings[row["target"]]
            result = run_fixture(row, binaries[row["target"]], registered, ignored, output,
                                 environment, binary_digests[row["target"]])
            report["fixtures"].append(result)
            report.pop("active_fixture")
            evidence.save_json(output / "report.json", report)
            print(row["id"], result["status"], flush=True)
        report["status"] = ("witnesses_passed" if all(
            row["status"] == "witness_passed" for row in report["fixtures"]
        ) else "witness_failures")
    except KeyboardInterrupt:
        report["status"] = "interrupted"
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        report["status"] = "driver_failed"
        report["error"] = str(error)
    finally:
        after, _, _ = evidence.source_snapshot()
        stable = (before == after and catalog_path.exists()
                  and evidence.digest_file(catalog_path) == catalog_digest
                  and all(binaries[target].exists()
                          and evidence.digest_file(binaries[target]) == digest
                          for target, digest in binary_digests.items()))
        report["source_after"] = after
        report["evidence_inputs_stable"] = stable
        report["matrix"] = stage_matrix(report["fixtures"], stable)
        if not stable:
            report["status"] = "evidence_inputs_changed"
        evidence.save_json(output / "report.json", report)
    return 0 if report["status"] == "witnesses_passed" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixture", action="append")
    parser.add_argument("--build-timeout", type=int, default=1800)
    args = parser.parse_args()
    require(args.build_timeout > 0, "build timeout must be positive")
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        return run(args)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
