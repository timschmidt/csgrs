"""Run the unchanged nine runtime commands after the complete merge proof gates."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    proof = read(E / "nth-root-enclosure-proof-validation-v2004.json")
    expected = proof["source_record_sha256"]
    assert expected == "f5fa38edebe8eb5ecdf831098567fab0cf0d8d6f588b56f1c93c4d5f56a5cac3"
    for key in ["fresh_verification_phase", "fresh_acceptance_phase"]:
        path = E / proof[key]
        result = read(path / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(path / "source-hashes.json.gz") == expected
    for name, digest in proof["proof_and_guard_inputs_sha256"].items():
        assert sha(ROOT / "target/scalar-evaluation-candidate/source" / name) == digest
    rejection = E / proof["reused_rejection_phase"]
    assert sha(rejection / "proof-checks/tooling.json") == proof["reused_rejection_tooling_sha256"]
    result = read(rejection / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    checkpoint_path = E / proof["retained_runtime_checkpoint"]
    assert sha(checkpoint_path) == proof["retained_runtime_checkpoint_sha256"]
    checkpoint = read(checkpoint_path)
    previous = json.loads(gzip.decompress((E / checkpoint["source_record"]).read_bytes()))
    current = json.loads(gzip.decompress((E / proof["source_record"]).read_bytes()))
    assert previous.keys() == current.keys()
    assert {name for name in current if previous[name] != current[name]} == set(proof["changed_sources_sha256"])
    baseline_path = E / "domain-predicate-candidate-validation-v1772.json"
    assert sha(baseline_path) == checkpoint["artifacts_sha256"][baseline_path.name]
    baseline = read(baseline_path)
    assert baseline["complete"] and len(baseline["phases"]) == 9
    out = E / "nth-root-enclosure-candidate-validation-v2013.json"
    assert not out.exists()
    rows = {}
    for version, (prior, item) in zip(range(2005, 2014), baseline["phases"].items()):
        suffix = prior.split("-domain-predicate-", 1)[1]
        phase = f"typed-api-prototype-v{version}-nth-root-enclosure-{suffix}"
        assert not (E / phase).exists()
        command = item["command"]
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase,
                        "--exec", *command], cwd=ROOT, check=True)
        result = read(E / phase / "result.json")
        assert sha(E / phase / "source-hashes.json.gz") == expected
        rows[phase] = {"baseline_phase": prior, "command": command, "result": result}
        out.write_text(json.dumps({
            "source_record_sha256": expected, "runner_sha256": sha(Path(__file__)),
            "complete": len(rows) == 9 and result["exit_code"] == 0,
            "phases": rows,
            "scope": "Unchanged nine runtime commands from the v1836 retained candidate; the combined kernel/selector proposal remains unretained pending cost and replay gates.",
        }, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"], (phase, result)
        print(phase, "passed", flush=True)


if __name__ == "__main__":
    main()
