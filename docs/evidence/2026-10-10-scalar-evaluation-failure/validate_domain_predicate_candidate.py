"""Replay the retained runtime validation commands after domain-predicate proofs."""
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
    proof = "typed-api-prototype-v1761-domain-predicate-verus"
    expected = sha(E / proof / "source-hashes.json.gz")
    for phase in [proof, "typed-api-prototype-v1762-domain-predicate-acceptance",
                  "typed-api-prototype-v1763-domain-predicate-rejections"]:
        result = read(E / phase / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(E / phase / "source-hashes.json.gz") == expected
    previous = json.loads(gzip.decompress((E / "typed-api-prototype-v1696-domain-bound-release-build/source-hashes.json.gz").read_bytes()))
    current = json.loads(gzip.decompress((E / proof / "source-hashes.json.gz").read_bytes()))
    allowed = read(E / "domain-predicate-baseline-v1759/baseline.json")["baseline_inputs_sha256"]
    assert previous.keys() == current.keys()
    assert {name for name in previous if previous[name] != current[name]} == set(allowed)
    retained = read(E / "domain-bound-candidate-validation-v1702.json")["phases"]
    assert len(retained) == 9
    out = E / "domain-predicate-candidate-validation-v1772.json"
    assert not out.exists()
    rows = {}
    for version, (prior, item) in zip(range(1764, 1773), retained.items()):
        suffix = prior.split("-domain-bound-", 1)[1]
        phase = f"typed-api-prototype-v{version}-domain-predicate-{suffix}"
        assert not (E / phase).exists()
        command = item["command"]
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase,
                        "--exec", *command], cwd=ROOT, check=True)
        result = read(E / phase / "result.json")
        assert sha(E / phase / "source-hashes.json.gz") == expected
        rows[phase] = {"baseline_phase": prior, "command": command, "result": result}
        out.write_text(json.dumps({
            "source_record_sha256": expected, "runner_sha256": sha(Path(__file__)),
            "complete": len(rows) == len(retained) and result["exit_code"] == 0,
            "phases": rows,
            "scope": "Unchanged nine runtime commands from the v1754 retained candidate; proof extraction remains unretained pending cost and replay gates.",
        }, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"], (phase, result)
        print(phase, "passed", flush=True)


if __name__ == "__main__":
    main()
