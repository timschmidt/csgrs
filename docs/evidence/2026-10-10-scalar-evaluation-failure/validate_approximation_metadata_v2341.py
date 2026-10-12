"""Run every proof guard and both formatting checks on the frozen proposal."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    expected = "090aedf5a30f3cbaf202a47a4c5deca5e437fbe2e248d23ed76004dda1303e39"
    proposal = E / "approximation-metadata-baseline-v2336/protocol.json"
    assert read(proposal)["source_record_sha256"] == expected
    full = E / "typed-api-prototype-v2337-approximation-metadata-verus"
    result = read(full / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert sha(full / "source-hashes.json.gz") == expected
    protocol = E / "approximation-metadata-validation-protocol-v2341.json"
    record = E / "approximation-metadata-validation-v2341.json"
    assert not protocol.exists() and not record.exists()
    runs = []
    for version, suffix, entry in [(2338, "acceptance", "test_acceptance.py"),
                                    (2339, "rejections", "test_rejections.py")]:
        phase = f"typed-api-prototype-v{version}-approximation-metadata-{suffix}"
        runs.append({"phase": phase, "command": [sys.executable,
            str(E / "run_candidate_proof_checks.py"), phase, entry]})
    for version, old, suffix in [(2340, 2258, "workspace-fmt"), (2341, 2259, "included-fmt")]:
        previous = E / f"typed-api-prototype-v{old}-eager-bound-decoder-{suffix}"
        runs.append({"phase": f"typed-api-prototype-v{version}-approximation-metadata-{suffix}",
            "command": read(previous / "command.json")["command"],
            "unchanged_command_from": previous.name})
    assert all(not (E / r["phase"]).exists() for r in runs)
    protocol.write_text(json.dumps({"status": "Frozen before these guard and format runs",
        "runner_sha256": sha(Path(__file__)), "proposal_protocol_sha256": sha(proposal),
        "source_record_sha256": expected, "runs": runs,
        "proof_wrapper_sha256": sha(E / "run_candidate_proof_checks.py"),
        "expected_rejected_mutations": 255, "expected_positive_baselines": 4,
        "expected_acceptance_tests": 10,
        "scope": "Run the entire existing entrypoint with ten additional metadata mutations; no historical proof guard is relabelled as a fresh run. No SOURCE edits or overlapping validations. Runtime/cost/replay reuse still requires a separate unchanged-input audit; this controller makes no empirical or whole-crate admission claim."
    }, indent=2) + "\n")
    phases = {}
    for row in runs:
        phase = E / row["phase"]
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase.name,
            "--exec", *row["command"]], cwd=ROOT, check=True)
        result = read(phase / "result.json")
        phases[phase.name] = row | {"result": result,
            "source_record_sha256": sha(phase / "source-hashes.json.gz")}
        record.write_text(json.dumps({"protocol_sha256": sha(protocol),
            "source_record_sha256": expected, "phases": phases,
            "complete": len(phases) == len(runs) and result["exit_code"] == 0,
            "scope": "Fresh proof guards and formatting only; runtime evidence reuse is separate."
        }, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
        if "rejections" in phase.name:
            tooling = read(phase / "proof-checks/tooling.json")
            assert tooling["tool_unchanged"] and tooling["exit_code"] == 0
            assert tooling["verus_invocations"] == 259
            assert tooling["rejected_invocations"] == 255 and tooling["baseline_successes"] == 4
        print(phase.name, "passed", flush=True)


if __name__ == "__main__":
    main()
