"""Make the exponent-link mutation unique without changing the proof proposal."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    old = E / "prepare_trailing_metadata_candidate_v2350.py"
    runner = E / "prepare_trailing_metadata_candidate_v2351.py"
    old_guard = E / "trailing-metadata-guard-proposal-v2350.py"
    guard = E / "trailing-metadata-guard-proposal-v2351.py"
    failure = E / "trailing-metadata-preparation-failure-v2350.json"
    assert not any(p.exists() for p in [runner, guard, failure])
    partial = E / "trailing-metadata-baseline-v2350"
    assert partial.exists() and not (partial / "protocol.json").exists()
    assert not (partial / "application.json").exists()
    text = old_guard.read_text()
    for expression in ["pow2(exponent)", "pow2(exponent + 1)"]:
        before = '"odd * pow2(shift) == ' + expression + ',"'
        after = '"odd * pow2(shift) == ' + expression + ',\\n    ensures odd == 1"'
        assert text.count(before) == 1
        text = text.replace(before, after)
    tree = ast.parse(text)
    mutations = ast.literal_eval(next(n.value for n in tree.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "MUTATIONS" for t in n.targets)))
    model_path = partial / "candidate/crates/hyperreal/verification/bit_length_model.rs"
    model = model_path.read_text()
    for name, before, after in mutations:
        start = model.index("proof fn " + name + "(")
        end = model.index("\n}\n", start) + 3
        assert model[start:end].count(before) == 1, name
        assert before != after
    guard.write_text(text)
    code = old.read_text().replace('OUT = E / "trailing-metadata-baseline-v2350"',
                                  'OUT = E / "trailing-metadata-baseline-v2351"')
    code = code.replace(old_guard.name, guard.name)
    assert code != old.read_text()
    ast.parse(code)
    runner.write_text(code)
    failure.write_text(json.dumps({"status": "Preserve the failed preparation; no SOURCE edit or proof run occurred",
        "failed_runner": old.name, "failed_runner_sha256": sha(old), "observed_exit_code": 1,
        "observed_failure": "AssertionError at line 129: the exponent-link mutation also matched nonlinear-arithmetic premises inside the same function.",
        "partial_directory": partial.name, "partial_candidate_model_sha256": sha(model_path),
        "old_guard_sha256": sha(old_guard), "new_guard": guard.name, "new_guard_sha256": sha(guard),
        "corrected_runner": runner.name, "corrected_runner_sha256": sha(runner),
        "repair_runner_sha256": sha(Path(__file__)), "negative_controls": len(mutations),
        "correction": "Include the following ensures clause in the mutation anchor, selecting exactly the declared precondition. Keep all twelve intended defects and all theorem bodies unchanged.",
        "source_changed": False, "scope": "This was an anchor-uniqueness guard in preparation, not a Verus failure or runtime result. The partial candidate and failed helper remain unchanged."}, indent=2) + "\n")
    print(json.dumps({"unique_negative_controls": len(mutations), "proof_proposal_changed": False}))


if __name__ == "__main__":
    main()
