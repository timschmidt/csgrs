"""Run digit-scan guards, acceptance and formatting against the final source."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())


def main():
    expected = "a83cff31d79bb8bbbb5100829d14a5683ca081410433e95b32d22460a062a730"
    proof = E / "typed-api-prototype-v2367-digit-scan-final-verus"
    result = read(proof / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert sha(proof / "source-hashes.json.gz") == expected
    report = read(proof / "proof-reports/evidence.json")
    assert report["verification_results"]["verified"] == 716 and report["verification_results"]["errors"] == 0
    assert len(report["scope"]["verified_model_theorems"]) == 160
    wrapper = E / "run_digit_scan_proof_checks_v2368.py"
    parent = E / "run_candidate_proof_checks.py"
    protocol = E / "digit-scan-validation-protocol-v2371.json"
    record = E / "digit-scan-validation-v2371.json"
    assert all(not p.exists() for p in [wrapper, protocol, record])
    text = parent.read_text()
    before = '    assert entrypoint in {"test_rejections.py", "test_acceptance.py"}'
    assert text.count(before) == 1
    text = text.replace(before, '    assert entrypoint == "test_digit_scan_rejections.py"')
    ast.parse(text)
    wrapper.write_text(text)
    guard_phase = "typed-api-prototype-v2368-digit-scan-rejections"
    acceptance_phase = "typed-api-prototype-v2369-digit-scan-acceptance"
    runs = [
        {"phase": guard_phase, "command": [sys.executable, str(wrapper), guard_phase, "test_digit_scan_rejections.py"]},
        {"phase": acceptance_phase, "command": [sys.executable, str(parent), acceptance_phase, "test_acceptance.py"]},
        {"phase": "typed-api-prototype-v2370-digit-scan-workspace-fmt",
         "command": read(E / "typed-api-prototype-v2355-trailing-metadata-workspace-fmt/command.json")["command"]},
    ]
    formatting = read(E / "typed-api-prototype-v2356-trailing-metadata-included-fmt/command.json")["command"]
    extra = ["crates/hyperreal/verification/digit_scan_model.rs", "crates/hyperreal/verification/lib.rs"]
    assert all(n not in formatting for n in extra)
    runs.append({"phase": "typed-api-prototype-v2371-digit-scan-included-fmt", "command": [*formatting, *extra]})
    assert all(not (E / r["phase"]).exists() for r in runs)
    protocol.write_text(json.dumps({
        "status": "Frozen before these guard and formatting executions",
        "source_record_sha256": expected, "runner_sha256": sha(Path(__file__)),
        "wrapper_sha256": sha(wrapper), "predecessor_wrapper_sha256": sha(parent),
        "wrapper_change": "Only the permitted entrypoint changes; invocation capture and rejection criteria remain unchanged.",
        "runs": runs, "new_negative_controls": 19, "new_positive_baselines": 1,
        "acceptance_tests": 10, "included_format_files": sum(n.endswith(".rs") for n in formatting) + len(extra),
        "scope": "The nineteen-control suite and its unmutated target, acceptance and both format commands execute on the final frozen source. Prior 267 guards/five baselines and historical runtime results require a separate exact-input/dependency/tool audit. No SOURCE edits or overlapping validations; no property or runtime admission.",
    }, indent=2) + "\n")
    phases = {}
    for row in runs:
        phase = E / row["phase"]
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase.name,
                        "--exec", *row["command"]], cwd=ROOT, check=True)
        result = read(phase / "result.json")
        phases[phase.name] = row | {"result": result, "source_record_sha256": sha(phase / "source-hashes.json.gz")}
        record.write_text(json.dumps({"protocol_sha256": sha(protocol), "source_record_sha256": expected,
            "phases": phases, "complete": len(phases) == len(runs) and result["exit_code"] == 0,
            "scope": "Fresh digit guards, acceptance and formatting; prior guard/runtime reuse is audited separately."}, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
        if phase.name == guard_phase:
            tooling = read(phase / "proof-checks/tooling.json")
            assert tooling["exit_code"] == 0 and tooling["tool_unchanged"]
            assert tooling["rejected_invocations"] == 19 and tooling["baseline_successes"] == 1
            assert tooling["verus_invocations"] == 20
        print(phase.name, "passed", flush=True)


if __name__ == "__main__":
    main()
