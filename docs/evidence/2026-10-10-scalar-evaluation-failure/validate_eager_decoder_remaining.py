"""Complete the remaining mutation groups and runtime checks without duplicate gates."""
import ast
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    milestone = read(E / "eager-bound-decoder-early-milestone-v2265.json")
    expected = milestone["source_record_sha256"]
    cached_phase = "typed-api-prototype-v2265-eager-bound-decoder-cache-rejections"
    cache = read(E / cached_phase / "proof-checks/tooling.json")
    assert cache["exit_code"] == 0 and cache["rejected_invocations"] == 41 and cache["baseline_successes"] == 1
    assert sha(E / cached_phase / "source-hashes.json.gz") == expected
    parent = E / "run_candidate_proof_checks.py"
    wrapper = E / "run_candidate_non_cache_proof_checks_v1.py"
    protocol_path = E / "eager-bound-decoder-remaining-protocol-v2276.json"
    guard_path = E / "eager-bound-decoder-remaining-guards-v2267.json"
    controller_path = E / "eager-bound-decoder-candidate-validation-v2276.json"
    assert all(not p.exists() for p in [wrapper, protocol_path, guard_path, controller_path])
    original = parent.read_text()
    before = '    assert entrypoint in {"test_rejections.py", "test_acceptance.py"}'
    assert original.count(before) == 1
    text = original.replace(before, '    assert entrypoint == "test_rejections.py"')
    before = '    script = CRATE / "verification" / entrypoint\n'
    after = '''    script = CRATE / "verification" / entrypoint
    cache_phase = EVIDENCE / "typed-api-prototype-v2265-eager-bound-decoder-cache-rejections"
    cache = json.loads((cache_phase / "proof-checks/tooling.json").read_text())
    assert cache["tool_sha256"] == tool_hashes and cache["tool_unchanged"] and cache["exit_code"] == 0
    assert cache["rejected_invocations"] == 41 and cache["baseline_successes"] == 1
    assert digest(cache_phase / "source-hashes.json.gz") == "cc51e3938c1cb6d772778a4683b7412812e234e69cfb1550d6b6e14120d264ad"
    original_entrypoint = script.read_text()
    omitted = "    reject_cache_mutations()\\n"
    assert original_entrypoint.count(omitted) == 1
    selected_entrypoint = original_entrypoint.replace(omitted, "")
    (out / "selected-entrypoint.py").write_text(selected_entrypoint)
    (out / "selection.json").write_text(json.dumps({
        "original_entrypoint": str(script), "original_entrypoint_sha256": digest(script),
        "selected_entrypoint_sha256": digest(out / "selected-entrypoint.py"),
        "only_removed_call": omitted, "separately_completed_cache_phase": cache_phase.name,
        "cache_tooling_sha256": digest(cache_phase / "proof-checks/tooling.json"),
        "scope": "Run every other mutation and positive baseline unchanged. The cache group already ran on these exact source/verifier identities; it is combined explicitly, not rerun or omitted from total coverage.",
    }, indent=2) + "\\n")
'''
    assert text.count(before) == 1
    text = text.replace(before, after)
    before = '        runpy.run_path(str(script), run_name="__main__")'
    after = '        exec(compile(selected_entrypoint, str(script), "exec"), {"__name__": "__main__", "__file__": str(script)})'
    assert text.count(before) == 1
    text = text.replace(before, after)
    ast.parse(text)
    wrapper.write_text(text)
    retained = read(E / "domain-predicate-candidate-validation-v1772.json")["phases"]
    early = read(E / "eager-bound-decoder-early-validation-v2261.json")
    reuse = [
        ("typed-api-prototype-v1766-domain-predicate-release-build", "typed-api-prototype-v2261-eager-bound-decoder-release-build"),
        ("typed-api-prototype-v1767-domain-predicate-workspace-fmt", "typed-api-prototype-v2258-eager-bound-decoder-workspace-fmt"),
        ("typed-api-prototype-v1768-domain-predicate-included-fmt", "typed-api-prototype-v2259-eager-bound-decoder-included-fmt"),
    ]
    phases = {}
    for previous, current in reuse:
        row = early["phases"][current]
        assert row["command"] == retained[previous]["command"]
        assert row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert sha(E / current / "source-hashes.json.gz") == expected
        phases[current] = {"baseline_phase": previous, **row, "execution": "Already completed on this exact candidate"}
    runs = []
    for version, old_version, suffix in [(2268, 1764, "scalar-tests"), (2269, 1765, "clippy"),
        (2273, 1769, "stack-tests"), (2274, 1770, "default-tests"), (2275, 1771, "debug-tests"),
        (2276, 1772, "minimal-clippy")]:
        previous = f"typed-api-prototype-v{old_version}-domain-predicate-{suffix}"
        runs.append({"phase": f"typed-api-prototype-v{version}-eager-bound-decoder-{suffix}",
            "baseline_phase": previous, "command": retained[previous]["command"]})
    protocol_path.write_text(json.dumps({"status": "Frozen before remaining guard and runtime execution",
        "source_record_sha256": expected, "runner_sha256": sha(Path(__file__)),
        "wrapper": wrapper.name, "wrapper_sha256": sha(wrapper), "wrapper_predecessor": parent.name,
        "wrapper_predecessor_sha256": sha(parent), "separately_completed_cache_phase": cached_phase,
        "cache_tooling_sha256": sha(E / cached_phase / "proof-checks/tooling.json"),
        "guard_phase": "typed-api-prototype-v2267-eager-bound-decoder-non-cache-rejections",
        "remaining_expected_mutations": 204, "remaining_expected_passing_baselines": 3,
        "already_completed_runtime_checks": phases, "remaining_runtime_commands": runs,
        "scope": "Every original mutation group is covered on the same source. Cache guards, release build and both formatting checks already passed; run the other groups and six remaining original runtime commands once. No source edits, overlapping builds/tests or fixture timing comparisons; costs and runtime retention remain separate and unresolved."}, indent=2) + "\n")
    guard_phase = "typed-api-prototype-v2267-eager-bound-decoder-non-cache-rejections"
    assert not (E / guard_phase).exists()
    subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), guard_phase, "--exec",
                    sys.executable, str(wrapper), guard_phase, "test_rejections.py"], cwd=ROOT, check=True)
    result = read(E / guard_phase / "result.json")
    guard_path.write_text(json.dumps({"source_record_sha256": expected, "protocol_sha256": sha(protocol_path),
        "phase": guard_phase, "result": result, "complete": result["exit_code"] == 0,
        "separately_completed_cache_phase": cached_phase}, indent=2) + "\n")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert sha(E / guard_phase / "source-hashes.json.gz") == expected
    tooling = read(E / guard_phase / "proof-checks/tooling.json")
    assert tooling["tool_sha256"] == cache["tool_sha256"] and tooling["tool_unchanged"]
    assert tooling["verus_invocations"] == 207 and tooling["rejected_invocations"] == 204 and tooling["baseline_successes"] == 3
    print("Remaining 204 mutations and three positive baselines passed; total 245 and four on this source.", flush=True)
    for row in runs:
        phase = E / row["phase"]
        assert not phase.exists()
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase.name,
                        "--exec", *row["command"]], cwd=ROOT, check=True)
        result = read(phase / "result.json")
        phases[phase.name] = {"baseline_phase": row["baseline_phase"], "command": row["command"],
            "result": result, "execution": "New execution after the remaining guard groups"}
        controller_path.write_text(json.dumps({"source_record_sha256": expected,
            "protocol_sha256": sha(protocol_path), "runner_sha256": sha(Path(__file__)),
            "complete": len(phases) == 9 and result["exit_code"] == 0, "phases": phases,
            "new_runtime_commands": len(phases) - 3, "already_completed_runtime_commands": 3,
            "scope": "Nine original runtime commands on this source, with three completed early checks included without rerunning; no cost/retention claim"}, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
        print(phase.name, "passed", flush=True)


if __name__ == "__main__":
    main()
