"""Run unchanged integer/root tests and refresh the unchanged conditional target."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "integer-coarse-zero-focused-validation-v2083.json"
    assert not out.exists()
    baseline = read(E / "integer-coarse-zero-baseline-v2079/baseline.json")
    before = json.loads(gzip.decompress((E / baseline["current_source_record"]).read_bytes()))
    changed = {name: sha(SOURCE / name) for name, digest in before.items() if sha(SOURCE / name) != digest}
    assert set(changed) == set(baseline["planned_changed_files"])
    assert len(changed) == 1
    for name, digest in baseline["proof_and_guard_inputs_sha256"].items():
        assert sha(SOURCE / name) == digest
    tests = [
        ("typed-api-prototype-v2080-integer-coarse-zero-focused", "integer_scale_"),
        ("typed-api-prototype-v2081-integer-coarse-zero-root-controls", "direct_nth_root_"),
    ]
    commands = [(phase, ["cargo", "test", "--release", "-p", "hyperreal", "--lib",
                         "--all-features", pattern, "--", "--nocapture"])
                for phase, pattern in tests]
    commands += [
        ("typed-api-prototype-v2082-integer-coarse-zero-verus",
         [sys.executable, str(E / "run_candidate_verus.py"), "typed-api-prototype-v2082-integer-coarse-zero-verus"]),
        ("typed-api-prototype-v2083-integer-coarse-zero-acceptance",
         [sys.executable, str(E / "run_candidate_proof_checks.py"), "typed-api-prototype-v2083-integer-coarse-zero-acceptance", "test_acceptance.py"]),
    ]
    expected = None
    rows = {}
    for phase, command in commands:
        assert not (E / phase).exists()
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase, "--exec", *command],
                       cwd=ROOT, check=True)
        result = read(E / phase / "result.json")
        digest = sha(E / phase / "source-hashes.json.gz")
        expected = expected or digest
        assert digest == expected
        rows[phase] = {"command": command, "result": result}
        out.write_text(json.dumps({
            "source_record_sha256": expected, "changed_since_rejected_source_sha256": changed,
            "runner_sha256": sha(Path(__file__)), "phases": rows,
            "complete": len(rows) == 4 and result["exit_code"] == 0,
            "scope": "Existing five integer tests and three direct-root controls, refreshed conditional verification and acceptance tests. The changed integer branch remains outside the Verus target. Mutation guards are not rerun by this helper; reuse must be audited separately. Broad/cost/replay admission is still pending.",
        }, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        print(phase, "passed", flush=True)


if __name__ == "__main__":
    main()
