"""Run typed-word guards, acceptance and formatting against the final source."""
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
    expected = "67e3d14b5d7a319778d56b701498e97225c16134eca65bb14a7586c727f2fb5a"
    proof = E / "typed-api-prototype-v2380-word-scan-verus"
    result = read(proof / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert sha(proof / "source-hashes.json.gz") == expected
    report = read(proof / "proof-reports/evidence.json")
    assert report["verification_results"]["verified"] == 736 and report["verification_results"]["errors"] == 0
    assert len(report["scope"]["verified_model_theorems"]) == 178
    wrapper = E / "run_word_scan_proof_checks_v2381.py"
    parent = E / "run_candidate_proof_checks.py"
    protocol = E / "word-scan-validation-protocol-v2384.json"
    record = E / "word-scan-validation-v2384.json"
    assert all(not p.exists() for p in [wrapper, protocol, record])
    text = parent.read_text()
    before = '    assert entrypoint in {"test_rejections.py", "test_acceptance.py"}'
    assert text.count(before) == 1
    text = text.replace(before, '    assert entrypoint == "test_word_scan_rejections.py"')
    ast.parse(text)
    wrapper.write_text(text)
    guard_phase = "typed-api-prototype-v2381-word-scan-rejections"
    acceptance_phase = "typed-api-prototype-v2382-word-scan-acceptance"
    runs = [
        {"phase": guard_phase, "command": [sys.executable, str(wrapper), guard_phase, "test_word_scan_rejections.py"]},
        {"phase": acceptance_phase, "command": [sys.executable, str(parent), acceptance_phase, "test_acceptance.py"]},
        {"phase": "typed-api-prototype-v2383-word-scan-workspace-fmt",
         "command": read(E / "typed-api-prototype-v2370-digit-scan-workspace-fmt/command.json")["command"]},
    ]
    formatting = read(E / "typed-api-prototype-v2371-digit-scan-included-fmt/command.json")["command"]
    extra = ["crates/hyperreal/verification/word_scan_model.rs"]
    assert all(n not in formatting for n in extra)
    runs.append({"phase": "typed-api-prototype-v2384-word-scan-included-fmt", "command": [*formatting, *extra]})
    assert all(not (E / r["phase"]).exists() for r in runs)
    protocol.write_text(json.dumps({
        "status": "Frozen before these guard and formatting executions",
        "source_record_sha256": expected, "runner_sha256": sha(Path(__file__)),
        "wrapper_sha256": sha(wrapper), "predecessor_wrapper_sha256": sha(parent),
        "wrapper_change": "Only the permitted entrypoint changes; invocation capture and rejection criteria remain unchanged.",
        "runs": runs, "new_negative_controls": 20, "new_positive_baselines": 1,
        "acceptance_tests": 10, "included_format_files": sum(n.endswith(".rs") for n in formatting) + len(extra),
        "scope": "The twenty-control suite and its unmutated target, acceptance and both format commands execute on the final frozen source. Prior 286 guards/six baselines and historical runtime results require a separate exact-input/dependency/tool audit. No SOURCE edits or overlapping validations; no property or runtime admission.",
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
            "scope": "Fresh word guards, acceptance and formatting; prior guard/runtime reuse is audited separately."}, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
        if phase.name == guard_phase:
            tooling = read(phase / "proof-checks/tooling.json")
            assert tooling["exit_code"] == 0 and tooling["tool_unchanged"]
            assert tooling["rejected_invocations"] == 20 and tooling["baseline_successes"] == 1
            assert tooling["verus_invocations"] == 21
        print(phase.name, "passed", flush=True)


if __name__ == "__main__":
    main()
