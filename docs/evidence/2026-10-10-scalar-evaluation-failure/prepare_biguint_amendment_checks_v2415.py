"""Fork fresh proof controls and runtime preparation for the Clippy amendment."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    proof = E / "biguint-metadata-proof-v2414"
    result = json.loads((proof / "result.json").read_text())
    assert result["exit_code"] == 0 and result["verification_results"]["verified"] == 462
    record = E / "biguint-amendment-check-preparation-v2415.json"
    assert not record.exists()
    rows = [
        ("check_biguint_metadata_guards_v2408.py", "check_biguint_metadata_guards_v2415.py", [
            ("biguint-metadata-candidate-v2406", "biguint-metadata-candidate-v2413"),
            ("biguint-metadata-guards-v2408", "biguint-metadata-guards-v2415"),
            ("biguint-metadata-proof-v2407", "biguint-metadata-proof-v2414"),
        ]),
        ("prepare_biguint_runtime_preview_v2411.py", "prepare_biguint_runtime_preview_v2416.py", [
            ("biguint-metadata-candidate-v2406", "biguint-metadata-candidate-v2413"),
            ("biguint-runtime-preparation-v2411", "biguint-runtime-preparation-v2416"),
            ("biguint-metadata-runtime-preview-v2411", "biguint-metadata-runtime-preview-v2416"),
            ("biguint-metadata-runtime-build-v2411", "biguint-metadata-runtime-build-v2416"),
            ("biguint-metadata-proof-v2407", "biguint-metadata-proof-v2414"),
            ("biguint-metadata-guards-v2408", "biguint-metadata-guards-v2415"),
        ]),
        ("validate_biguint_runtime_preview_v2412.py", "validate_biguint_runtime_preview_v2417.py", [
            ("biguint-runtime-preparation-v2411", "biguint-runtime-preparation-v2416"),
        ]),
    ]
    prepared = []
    for parent_name, child_name, replacements in rows:
        parent, child = E / parent_name, E / child_name
        assert not child.exists()
        code = parent.read_text()
        for before, after in replacements:
            assert code.count(before) == 1, (child_name, before)
            code = code.replace(before, after)
        ast.parse(code)
        child.write_text(code)
        prepared.append({"parent": parent_name, "parent_sha256": sha(parent), "successor": child_name,
                         "successor_sha256": sha(child), "replacements": replacements})
    record.write_text(json.dumps({
        "status": "Frozen before fresh controls and ordinary-Rust preparation for the amended preview",
        "runner_sha256": sha(Path(__file__)), "proof_result_sha256": sha(proof / "result.json"),
        "prepared": prepared, "negative_controls": 20, "positive_baselines": 1,
        "scope": "Only input/phase bindings change in the existing runners. Mutation definitions, proof error criteria, dependency pin checks and runtime corpus remain unchanged. All controls execute afresh after the helper's empty-slice change. No earlier failed phase is overwritten or credited as successful.",
    }, indent=2) + "\n")
    print(json.dumps({"fresh_runners": len(prepared), "negative_controls": 20, "positive_baselines": 1}))


if __name__ == "__main__":
    main()
