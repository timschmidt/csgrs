"""Run the retained nine runtime commands on the fully checked decoder proposal."""
import gzip
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
    proof = read(E / "bound-decoder-proof-validation-v2171.json")
    expected = proof["source_record_sha256"]
    assert proof["verified_units"] == 655 and proof["errors"] == 0
    assert proof["fresh_rejected_mutations"] == 245 and proof["fresh_passing_guard_baselines"] == 4
    assert proof["fresh_acceptance_tests"] == 10 and not proof["runtime_retained"]
    for key in ["fresh_verification_phase", "fresh_acceptance_phase", "fresh_rejection_phase"]:
        phase = E / proof[key]
        result = read(phase / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
    checkpoint_path = E / proof["retained_runtime_checkpoint"]
    assert sha(checkpoint_path) == proof["retained_runtime_checkpoint_sha256"]
    checkpoint = read(checkpoint_path)
    before = json.loads(gzip.decompress((E / checkpoint["source_record"]).read_bytes()))
    current = json.loads(gzip.decompress((E / proof["source_record"]).read_bytes()))
    assert before.keys() == current.keys()
    assert {n for n in current if current[n] != before[n]} == set(proof["changed_sources_sha256"])
    baseline_path = E / "domain-predicate-candidate-validation-v1772.json"
    assert sha(baseline_path) == checkpoint["artifacts_sha256"][baseline_path.name]
    baseline = read(baseline_path)
    assert baseline["complete"] and len(baseline["phases"]) == 9
    out = E / "bound-decoder-candidate-validation-v2180.json"
    assert not out.exists()
    rows = {}
    for version, (prior, item) in zip(range(2172, 2181), baseline["phases"].items()):
        suffix = prior.split("-domain-predicate-", 1)[1]
        phase = f"typed-api-prototype-v{version}-bound-decoder-{suffix}"
        assert not (E / phase).exists()
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase,
                        "--exec", *item["command"]], cwd=ROOT, check=True)
        result = read(E / phase / "result.json")
        assert sha(E / phase / "source-hashes.json.gz") == expected
        rows[phase] = {"baseline_phase": prior, "command": item["command"], "result": result}
        out.write_text(json.dumps({"source_record_sha256": expected, "runner_sha256": sha(Path(__file__)),
            "complete": len(rows) == 9 and result["exit_code"] == 0, "phases": rows,
            "scope": "Unchanged nine runtime commands from retained v1836; no whole-repository CI or runtime retention claim."}, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"], phase
        print(phase, "passed", flush=True)


if __name__ == "__main__":
    main()
