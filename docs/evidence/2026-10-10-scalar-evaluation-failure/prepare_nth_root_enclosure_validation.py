"""Fork the unchanged broad commands and account for the two added endpoint tests."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "nth-root-enclosure-validation-preparation-v2013.json"
    assert not out.exists()
    proof_path = E / "nth-root-enclosure-proof-validation-v2004.json"
    proof = json.loads(proof_path.read_text())
    assert proof["focused_tests"] == 3 and proof["verified_units"] == 651
    assert proof["fresh_acceptance_tests"] == 10 and proof["reused_rejected_mutants"] == 247
    versions = {1924: 2003, 1926: 2004, **dict(zip(range(1927, 1936), range(2005, 2014)))}
    forks = {}
    for parent_name, name in [
        ("validate_cache_state_merge_candidate.py", "validate_nth_root_enclosure_candidate.py"),
        ("record_cache_state_merge_validation.py", "record_nth_root_enclosure_validation.py"),
    ]:
        parent, target = E / parent_name, E / name
        assert not target.exists()
        text = parent.read_text().replace("cache-state-merge", "nth-root-enclosure")
        for old, new in versions.items():
            text = text.replace(f"v{old}", f"v{new}")
        text = text.replace("555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59", proof["source_record_sha256"])
        if name.startswith("validate_"):
            old = '["fresh_verification_phase", "fresh_acceptance_phase", "fresh_rejection_phase"]'
            assert text.count(old) == 1
            text = text.replace(old, '["fresh_verification_phase", "fresh_acceptance_phase"]')
            anchor = '    checkpoint_path = E / proof["retained_runtime_checkpoint"]\n'
            assert text.count(anchor) == 1
            text = text.replace(anchor,
                '    for name, digest in proof["proof_and_guard_inputs_sha256"].items():\n'
                '        assert sha(ROOT / "target/scalar-evaluation-candidate/source" / name) == digest\n'
                '    rejection = E / proof["reused_rejection_phase"]\n'
                '    assert sha(rejection / "proof-checks/tooling.json") == proof["reused_rejection_tooling_sha256"]\n'
                '    result = read(rejection / "result.json")\n'
                '    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]\n'
                + anchor)
            assert "range(1927, 1936)" in text
            text = text.replace("range(1927, 1936)", "range(2005, 2014)")
            text = text.replace("the merge extraction remains unretained", "the combined kernel/selector proposal remains unretained")
        else:
            for old, new in [(1074, 1076), (5858, 5860), (1059, 1061)]:
                assert str(old) in text
                text = text.replace(str(old), str(new))
        ast.parse(text)
        assert "cache-state-merge" not in text and "v1935" not in text
        target.write_text(text)
        forks[name] = {"predecessor": parent_name, "predecessor_sha256": sha(parent), "fork_sha256": sha(target)}
    out.write_text(json.dumps({
        "status": "Prepared before running the nine broad commands on the combined proposal",
        "source_record_sha256": proof["source_record_sha256"], "proof_record": proof_path.name,
        "proof_record_sha256": sha(proof_path), "runner_forks": forks,
        "generator_sha256": sha(Path(__file__)),
        "baseline_commands": "domain-predicate-candidate-validation-v1772.json",
        "phases": list(range(2005, 2014)),
        "expected_tests": {"scalar": [1076, 1, 0], "stack": [5860, 3, 33],
            "default": [1061, 3, 0], "debug": [1076, 1, 0]},
        "limits": "The commands, 108 included formatting paths, two strict Clippy configurations and inherited warning/ignore comparisons are unchanged. Only the two new endpoint tests change expected pass counts. This does not replace original-cap costs, replays or full-workspace/CI admission.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared": list(forks), "commands_changed": False, "added_tests": 2}))


if __name__ == "__main__":
    main()
