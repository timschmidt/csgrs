"""Run fresh acceptance and complete negative guards on the proved decoder source."""
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
    proof = E / "typed-api-prototype-v2169-bound-decoder-verus"
    report = read(proof / "proof-reports/verus.json")["verification-results"]
    assert report["success"] and report["verified"] == 655 and report["errors"] == 0
    result = read(proof / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    expected = sha(proof / "source-hashes.json.gz")
    assert expected == "80bb17679a0891ff93eba8ed10b9f63c205f3ccb39e35dbb3e187e23bb0b7162"
    output = E / "bound-decoder-focused-validation-v2171.json"
    assert not output.exists()
    rows = []
    for version, label, entrypoint in [(2170, "acceptance", "test_acceptance.py"), (2171, "rejections", "test_rejections.py")]:
        phase = f"typed-api-prototype-v{version}-bound-decoder-{label}"
        command = [sys.executable, str(E / "run_candidate_proof_checks.py"), phase, entrypoint]
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase, "--exec", *command], cwd=ROOT, check=True)
        result = read(E / phase / "result.json")
        assert sha(E / phase / "source-hashes.json.gz") == expected
        rows.append({"phase": phase, "command": command, "result": result})
        output.write_text(json.dumps({"source_record_sha256": expected, "runner_sha256": sha(Path(__file__)),
            "complete": len(rows) == 2 and result["exit_code"] == 0, "phases": rows}, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"], phase
        print(phase, "passed", flush=True)


if __name__ == "__main__":
    main()
