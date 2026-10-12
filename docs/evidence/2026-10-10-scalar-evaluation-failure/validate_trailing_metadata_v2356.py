"""Run the new metadata guard group, acceptance and complete format commands."""
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
    expected = "57fe3139119d43abbdb76bcbee47c1f78245107a397d9c3672132d9d0eadd979"
    proof = E / "typed-api-prototype-v2352-trailing-metadata-verus"
    result = read(proof / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert sha(proof / "source-hashes.json.gz") == expected
    report = read(proof / "proof-reports/evidence.json")
    assert report["verification_results"]["verified"] == 682 and report["verification_results"]["errors"] == 0
    wrapper = E / "run_trailing_metadata_proof_checks_v2353.py"
    parent = E / "run_candidate_proof_checks.py"
    protocol = E / "trailing-metadata-validation-protocol-v2356.json"
    record = E / "trailing-metadata-validation-v2356.json"
    assert not wrapper.exists() and not protocol.exists() and not record.exists()
    text = parent.read_text()
    before = '    assert entrypoint in {"test_rejections.py", "test_acceptance.py"}'
    assert text.count(before) == 1
    text = text.replace(before, '    assert entrypoint == "test_bit_metadata_rejections.py"')
    ast.parse(text)
    wrapper.write_text(text)
    guard_phase = "typed-api-prototype-v2353-trailing-metadata-rejections"
    acceptance_phase = "typed-api-prototype-v2354-trailing-metadata-acceptance"
    runs = [
        {"phase": guard_phase, "command": [sys.executable, str(wrapper), guard_phase, "test_bit_metadata_rejections.py"]},
        {"phase": acceptance_phase, "command": [sys.executable, str(parent), acceptance_phase, "test_acceptance.py"]},
        {"phase": "typed-api-prototype-v2355-trailing-metadata-workspace-fmt",
         "command": read(E / "typed-api-prototype-v2340-approximation-metadata-workspace-fmt/command.json")["command"]},
    ]
    formatting = read(E / "typed-api-prototype-v2341-approximation-metadata-included-fmt/command.json")["command"]
    extra = "crates/hyperreal/verification/bit_length_model.rs"
    assert extra not in formatting
    runs.append({"phase": "typed-api-prototype-v2356-trailing-metadata-included-fmt",
                 "command": [*formatting, extra]})
    assert all(not (E / r["phase"]).exists() for r in runs)
    protocol.write_text(json.dumps({"status": "Frozen before these guard and formatting executions",
        "source_record_sha256": expected, "runner_sha256": sha(Path(__file__)),
        "wrapper_sha256": sha(wrapper), "predecessor_wrapper_sha256": sha(parent),
        "wrapper_change": "Only the allowed entrypoint changes to the new metadata guard; actual invocation capture and rejection checks are untouched.",
        "runs": runs, "new_negative_controls": 12, "new_positive_baselines": 1,
        "acceptance_tests": 10, "included_format_files": sum(n.endswith(".rs") for n in formatting) + 1,
        "scope": "Run the new twelve-control suite and its unmutated target once, plus acceptance and both formatting commands. The existing 255 guards/four baselines and historical runtime results require a separate exact-input/dependency/tool audit before reuse. No SOURCE edits or overlapping validations; no property or runtime admission."}, indent=2) + "\n")
    phases = {}
    for row in runs:
        phase = E / row["phase"]
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase.name,
                        "--exec", *row["command"]], cwd=ROOT, check=True)
        result = read(phase / "result.json")
        phases[phase.name] = row | {"result": result,
            "source_record_sha256": sha(phase / "source-hashes.json.gz")}
        record.write_text(json.dumps({"protocol_sha256": sha(protocol), "source_record_sha256": expected,
            "phases": phases, "complete": len(phases) == len(runs) and result["exit_code"] == 0,
            "scope": "Fresh new guards, acceptance and formatting; old guard/runtime reuse remains a separate audit."}, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
        if phase.name == guard_phase:
            tooling = read(phase / "proof-checks/tooling.json")
            assert tooling["exit_code"] == 0 and tooling["tool_unchanged"]
            assert tooling["rejected_invocations"] == 12 and tooling["baseline_successes"] == 1
            assert tooling["verus_invocations"] == 13
        print(phase.name, "passed", flush=True)


if __name__ == "__main__":
    main()
