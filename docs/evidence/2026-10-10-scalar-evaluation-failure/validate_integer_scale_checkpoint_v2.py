#!/usr/bin/env python3
"""Validate the integer-scale candidate without overlapping workloads."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
EXPECTED_SOURCE = "32b816463eacd2d2cef2f905c0455557c015117690a8a8dd635229a57726d2b2"


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def completed(phase):
    path = EVIDENCE / phase
    result = read(path / "result.json")
    assert result["exit_code"] == 0, (phase, result)
    assert result["source_unchanged"] and result["main_unchanged"], phase
    assert digest(path / "source-hashes.json.gz") == EXPECTED_SOURCE, phase
    return path


def summary(phase, name):
    result = parse_phase(completed(phase))
    assert result["top_level_failed"] == result["child_process_failed"] == 0
    out = EVIDENCE / name
    if out.exists():
        assert read(out) == result, out
    else:
        out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"phase": phase, "passed": result["top_level_passed"],
                      "child_passed": result["child_process_passed"],
                      "ignored": result["top_level_ignored"]}), flush=True)


def validate(phase, *arguments):
    if (EVIDENCE / phase).exists():
        path = completed(phase)
        expected_command = list(arguments[1:]) if arguments[0] == "--exec" else ["cargo", *arguments]
        assert read(path / "command.json")["command"] == expected_command, phase
        print(f"Reusing terminal passing phase: {phase}", flush=True)
        return path
    subprocess.run([sys.executable, str(EVIDENCE / "validate_candidate_v2.py"),
                    phase, *arguments], cwd=ROOT, check=True)
    # The retained runner records Cargo failures without returning them itself.
    return completed(phase)


def main():
    proof = "typed-api-prototype-v1368-budget-verus"
    validate(proof, "--exec", sys.executable, str(EVIDENCE / "run_candidate_verus.py"), proof)
    acceptance = "typed-api-prototype-v1369-proof-acceptance-tests"
    validate(acceptance, "--exec", sys.executable,
             str(EVIDENCE / "run_candidate_proof_checks.py"), acceptance, "test_acceptance.py")
    rejection = "typed-api-prototype-v1370-proof-rejections"
    validate(rejection, "--exec", sys.executable,
             str(EVIDENCE / "run_candidate_proof_checks.py"), rejection, "test_rejections.py")
    validate("typed-api-prototype-v1355-integer-scale-clippy", "clippy", "--release", "-p", "hyperreal",
             "-p", "hyperlimit", "-p", "hyperlattice", "-p", "hypercurve", "-p", "hyperdrc",
             "--all-targets", "--all-features", "--", "-D", "warnings")
    validate("typed-api-prototype-v1371-debug-scalar-unit-tests", "test", "-p", "hyperreal",
             "--lib", "--all-features")
    summary("typed-api-prototype-v1371-debug-scalar-unit-tests", "typed-api-prototype-v1371-test-summary.json")
    validate("typed-api-prototype-v1372-stack-tests", "test", "--release", "-p", "hyperreal",
             "-p", "hyperlimit", "-p", "hyperlattice", "-p", "hypercurve", "-p", "hyperdrc",
             "--all-features")
    stack = completed("typed-api-prototype-v1372-stack-tests")
    summary(stack.name, "typed-api-prototype-v1372-test-summary.json")
    binaries = re.findall(r"Running unittests src/lib\.rs \(([^)\n]*/hyperreal-[^)]+)\)",
                          (stack / "stderr.log").read_text())
    assert len(binaries) == 1, binaries
    binary = Path(binaries[0])
    binary_hash = digest(binary)

    validate("typed-api-prototype-v1373-default-tests", "test", "--release", "-p", "hyperreal")
    summary("typed-api-prototype-v1373-default-tests", "typed-api-prototype-v1373-test-summary.json")
    validate("typed-api-prototype-v1374-minimal-clippy", "clippy", "--release", "-p", "hyperreal",
             "--all-targets", "--no-default-features", "--", "-D", "warnings")
    validate("typed-api-prototype-v1375-workspace-fmt", "fmt", "--all", "--", "--check")

    source = json.loads(gzip.decompress((stack / "source-hashes.json.gz").read_bytes()))
    original = json.loads(gzip.decompress(
        (EVIDENCE / "candidate-preparation/source-hashes.json.gz").read_bytes()))
    changed_rust = sorted(name for name, value in source.items()
                          if name.endswith(".rs") and original.get(name) != value)
    print(f"Checking {len(changed_rust)} changed Rust files from the frozen source map.", flush=True)
    validate("typed-api-prototype-v1376-included-fmt", "--exec", "rustfmt", "--edition", "2024",
             "--config-path", "crates/.rustfmt.toml", "--config", "skip_children=true",
             "--check", *changed_rust)

    # No builds, tests or other measurements overlap either capped workload.
    for phase, selected in [
        ("typed-api-prototype-v1377-private-kernel-cap", "private_kernel_domain_tests"),
        ("typed-api-prototype-v1378-magnitude-cap", "magnitude_upper_bound_tests"),
        ("typed-api-prototype-v1379-publication-cap", "cache_publication_tests"),
        ("typed-api-prototype-v1380-fractional-cap", "fractional_separation_tests"),
        ("typed-api-prototype-v1381-scaled-lower-cap", "scaled_lower_magnitude_tests"),
        ("typed-api-prototype-v1382-trig-range-cap", "private_trig_range_tests"),
        ("typed-api-prototype-v1383-trig-lattice-cap", "trig_lattice_tests"),
        ("typed-api-prototype-v1384-trig-residual-cap", "trig_periodic_tests"),
        ("typed-api-prototype-v1385-trig-operand-cap", "trig_operands_tests"),
        ("typed-api-prototype-v1386-evaluation-budget-cap", "evaluation_budget_tests"),
        ("typed-api-prototype-v1387-algebraic-budget-cap", "algebraic_budget_tests"),
        ("typed-api-prototype-v1388-normal-form-budget-cap", "normal_form_budget_tests"),
        ("typed-api-prototype-v1389-inverse-trig-budget-cap", "inverse_trig_budget_tests"),
        ("typed-api-prototype-v1390-algebraic-leaf-cap", "algebraic_leaf_tests"),
        ("typed-api-prototype-v1391-inverse-dag-cap", "inverse_trig_dag_tests"),
        ("typed-api-prototype-v1392-structural-comparison-cap", "structural_comparison_budget_tests"),
        ("typed-api-prototype-v1393-algebraic-preflight-cap", "algebraic_preflight_tests"),
        ("typed-api-prototype-v1394-rational-preflight-cap", "rational_preflight_tests"),
        ("typed-api-prototype-v1395-inverse-constructor-cap", "inverse_constructor_preflight_tests"),
    ]:
        assert digest(binary) == binary_hash
        validate(phase, "--exec", sys.executable, str(EVIDENCE / "run_limited_unit_test.py"),
                 str(binary), binary_hash, f"computable::node::{selected}::",
                 "--nocapture", "--test-threads=1")

    build = "typed-api-prototype-v1356-integer-scale-build"
    replay = "typed-api-prototype-v1396-replay"
    boundary = "typed-api-prototype-v1397-precision-replay"
    validate(build, "build", "--release", "-p", "hyperreal", "-p", "hypercurve", "--all-features")
    if not (EVIDENCE / replay / "summary.json").exists():
        subprocess.run([sys.executable, str(EVIDENCE / "reproduce_candidate.py"), build, replay],
                       cwd=ROOT, check=True)
    result = read(EVIDENCE / replay / "summary.json")
    assert result["source_hashes_sha256"] == EXPECTED_SOURCE and result["guards_unchanged"]
    assert result["ordinary_cases"] == 400
    assert result["invalid_first_contracts"] == result["invalid_retries"] == 0
    assert result["typed_interruption_errors"] == 300
    assert len(result["extremes"]) == 6 and all(row["valid"] for row in result["extremes"])
    assert len(result["cache"]) == 9 and all(row["valid"] for row in result["cache"])
    assert all(row["exit_code"] == 0 for row in result["commands"])

    if not (EVIDENCE / boundary / "summary.json").exists():
        subprocess.run([sys.executable, str(EVIDENCE / "replay_precision.py"), build, replay, boundary],
                       cwd=ROOT, check=True)
    result = read(EVIDENCE / boundary / "summary.json")
    assert result["source_hashes_sha256"] == EXPECTED_SOURCE and result["guards_unchanged"]
    assert result["cases"] == result["valid_cases"] == 60
    assert not result["nonzero_exit_cases"] and not result["invalid_numeric_cases"]
    for script, phase in [("reproduce_private_kernel.py", "typed-api-prototype-v1398-private-kernel-replay"),
                          ("reproduce_private_trig.py", "typed-api-prototype-v1399-private-trig-replay")]:
        if not (EVIDENCE / phase / "summary.json").exists():
            subprocess.run([sys.executable, str(EVIDENCE / script), build, phase], cwd=ROOT, check=True)
        data = read(EVIDENCE / phase / "summary.json")
        assert data["source_hashes_sha256"] == EXPECTED_SOURCE and data["guards_unchanged"]
        if script == "reproduce_private_kernel.py":
            assert data["cases"] == 78 and data["incomplete_cases"] == data["unexpected_result_shapes"] == 0
        else:
            assert data["cases"] == 600 and data["invalid_numeric_samples"] == data["incomplete_cases"] == data["typed_errors"] == 0
    wide = "typed-api-prototype-v1400-private-trig-wide-replay"
    if not (EVIDENCE / wide / "summary.json").exists():
        subprocess.run([sys.executable, str(EVIDENCE / "reproduce_private_trig_wide.py"),
                        build, wide, stack.name], cwd=ROOT, check=True)
    data = read(EVIDENCE / wide / "summary.json")
    assert data["source_hashes_sha256"] == EXPECTED_SOURCE and data["guards_unchanged"]
    assert data["cases"] == 384 and data["numeric_samples"] == 1536
    assert data["incomplete_cases"] == data["invalid_numeric_samples"] == data["typed_errors"] == 0
    lattice = "typed-api-prototype-v1401-symbolic-trig-scale-replay"
    if not (EVIDENCE / lattice / "summary.json").exists():
        subprocess.run([sys.executable, str(EVIDENCE / "reproduce_symbolic_trig_scales.py"),
                        build, lattice], cwd=ROOT, check=True)
    data = read(EVIDENCE / lattice / "summary.json")
    assert data["source_hashes_sha256"] == EXPECTED_SOURCE and data["guards_unchanged"]
    assert data["cases"] == 168 and data["valid_samples"] == 672
    assert data["incomplete_cases"] == data["invalid_samples"] == 0
    residual = "typed-api-prototype-v1402-symbolic-trig-residual-replay"
    if not (EVIDENCE / residual / "summary.json").exists():
        subprocess.run([sys.executable, str(EVIDENCE / "reproduce_symbolic_trig_residuals.py"),
                        build, residual, stack.name], cwd=ROOT, check=True)
    data = read(EVIDENCE / residual / "summary.json")
    assert data["source_hashes_sha256"] == EXPECTED_SOURCE and data["guards_unchanged"]
    assert data["cases"] == 456 and data["numeric_samples"] == data["observations"] == 1824
    assert data["incomplete_cases"] == data["invalid_numeric_samples"] == data["typed_errors"] == 0
    operand = "typed-api-prototype-v1403-symbolic-trig-nonrational-replay"
    if not (EVIDENCE / operand / "summary.json").exists():
        subprocess.run([sys.executable, str(EVIDENCE / "reproduce_symbolic_trig_nonrational.py"),
                        build, operand, stack.name], cwd=ROOT, check=True)
    data = read(EVIDENCE / operand / "summary.json")
    assert data["source_hashes_sha256"] == EXPECTED_SOURCE and data["guards_unchanged"]
    assert data["cases"] == 384 and data["numeric_samples"] == data["observations"] == 1536
    assert data["incomplete_cases"] == data["invalid_numeric_samples"] == data["typed_errors"] == 0
    budgeted = "typed-api-prototype-v1363-integer-scale-evaluation-replay"
    if not (EVIDENCE / budgeted / "summary.json").exists():
        subprocess.run([sys.executable, str(EVIDENCE / "reproduce_evaluation_budget.py"),
                        build, budgeted, stack.name], cwd=ROOT, check=True)
    data = read(EVIDENCE / budgeted / "summary.json")
    assert data["source_hashes_sha256"] == EXPECTED_SOURCE and data["guards_unchanged"]
    assert data["cases"] == 384 and data["numeric_samples"] == data["observations"] == 1536
    assert data["warm_zero_budget_checks"] == 384
    assert data["incomplete_cases"] == data["invalid_numeric_samples"] == data["typed_errors"] == 0
    print("All final-source integer-scale checkpoint validations passed.", flush=True)


if __name__ == "__main__":
    main()
