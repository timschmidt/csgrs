"""Run acceptance and the cache-specific mutation guards for the eager decoder."""
import ast
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
    baseline = read(E / "eager-bound-decoder-baseline-v2257/protocol.json")
    expected = baseline["source_record_sha256"]
    early = read(E / "eager-bound-decoder-early-validation-v2261.json")
    assert early["complete"] and early["source_record_sha256"] == expected
    parent = E / "run_candidate_proof_checks.py"
    wrapper = E / "run_candidate_cache_proof_checks_v1.py"
    protocol = E / "eager-bound-decoder-guard-protocol-v2265.json"
    output = E / "eager-bound-decoder-focused-guards-v2265.json"
    assert all(not p.exists() for p in [wrapper, protocol, output])
    text = parent.read_text()
    before = '    assert entrypoint in {"test_rejections.py", "test_acceptance.py"}'
    assert text.count(before) == 1
    text = text.replace(before, '    assert entrypoint == "test_cache_rejections.py"')
    ast.parse(text)
    wrapper.write_text(text)
    phases = [
        {"phase": "typed-api-prototype-v2264-eager-bound-decoder-acceptance", "helper": parent.name,
         "entrypoint": "test_acceptance.py"},
        {"phase": "typed-api-prototype-v2265-eager-bound-decoder-cache-rejections", "helper": wrapper.name,
         "entrypoint": "test_cache_rejections.py"},
    ]
    protocol.write_text(json.dumps({"status": "Frozen before focused guard execution",
        "source_record_sha256": expected, "runner_sha256": sha(Path(__file__)), "phases": phases,
        "wrapper_predecessor": parent.name, "wrapper_predecessor_sha256": sha(parent),
        "wrapper": wrapper.name, "wrapper_sha256": sha(wrapper),
        "only_wrapper_change": {"before": before, "after": '    assert entrypoint == "test_cache_rejections.py"'},
        "scope": "Ten acceptance tests and only the current cache mutation suite: 23 production mutations, 18 model mutations and one passing baseline. The other mutation suites are not rerun or credited as fresh/reused here. The execution/input/tool capture and rejection criteria remain unchanged. No runtime retention or cost pass follows."}, indent=2) + "\n")
    results = []
    for row in phases:
        phase = E / row["phase"]
        assert not phase.exists()
        command = [sys.executable, str(E / row["helper"]), phase.name, row["entrypoint"]]
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase.name,
                        "--exec", *command], cwd=ROOT, check=True)
        result = read(phase / "result.json")
        results.append(row | {"result": result})
        output.write_text(json.dumps({"source_record_sha256": expected, "protocol_sha256": sha(protocol),
            "runner_sha256": sha(Path(__file__)), "complete": len(results) == 2 and result["exit_code"] == 0,
            "phases": results, "scope": "Acceptance and cache-specific guards only; no other mutation-suite execution"}, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
        print(phase.name, "passed", flush=True)


if __name__ == "__main__":
    main()
