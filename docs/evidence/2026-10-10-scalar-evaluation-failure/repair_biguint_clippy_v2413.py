"""Fork the shared dependency preview to address the observed strict Clippy findings."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "prepare_biguint_metadata_candidate_v2406.py"
    preparation = E / "prepare_biguint_metadata_candidate_v2413.py"
    old_runner = E / "probe_biguint_metadata_v2407.py"
    runner = E / "probe_biguint_metadata_v2414.py"
    record = E / "biguint-clippy-correction-v2413.json"
    assert all(not p.exists() for p in [preparation, runner, record, E / "biguint-metadata-candidate-v2413", E / "biguint-metadata-proof-v2414"])
    failed = E / "biguint-runtime-v2412-vendor-clippy"
    result = json.loads((failed / "result.json").read_text())
    assert result["exit_code"] == 101 and result["work_unchanged"] and result["source_unchanged"]
    diagnostic = (failed / "stderr.log").read_text()
    assert diagnostic.count("error: unnecessary `unsafe` block") == 2
    assert diagnostic.count("error: truncating to zero length") == 2
    assert diagnostic.count("error: length comparison to zero") == 1
    code = parent.read_text()
    assert code.count("if words.len() == 0 {") == 1
    code = code.replace("if words.len() == 0 {", "if words.is_empty() {")
    anchor = '    (vendor / "src/biguint.rs").write_text(biguint)\n'
    assert code.count(anchor) == 1
    added = '''    for name, function in [("addition.rs", "adc"), ("subtraction.rs", "sbb")]:
        path = vendor / "src/biguint" / name
        text = path.read_text()
        before = "    #[inline]\\n    fn " + function + "("
        assert text.count(before) == 2
        after = "    // Older supported compilers require the intrinsic's unsafe block.\\n    #[allow(unused_unsafe)]\\n" + before
        path.write_text(text.replace(before, after))
    path = vendor / "src/biguint/multiplication.rs"
    text = path.read_text()
    assert text.count("p.data.truncate(0);") == 2
    path.write_text(text.replace("p.data.truncate(0);", "p.data.clear();"))
'''
    code = code.replace(anchor, anchor + added)
    before = '    assert modified == ["Cargo.toml", "src/biguint.rs"]'
    assert code.count(before) == 1
    code = code.replace(before, '    assert modified == ["Cargo.toml", "src/biguint/addition.rs", "src/biguint/multiplication.rs", "src/biguint/subtraction.rs", "src/biguint.rs"]')
    before = '        "preserved_behavior": "The normalize guard'
    assert code.count(before) == 1
    code = code.replace(before, '        "compatibility_fixes": "Use is_empty in the new helper. Preserve the existing four native intrinsic unsafe blocks for older supported compilers with function-local unused_unsafe allowances. Replace the two unchanged upstream multiplication truncate(0) calls with capacity-preserving clear(). These addition/subtraction/multiplication bodies remain outside the metadata proof and require runtime qualification.",\n' + before)
    assert code.count("biguint-metadata-candidate-v2406") == 1
    code = code.replace("biguint-metadata-candidate-v2406", "biguint-metadata-candidate-v2413")
    ast.parse(code)
    preparation.write_text(code)
    code = old_runner.read_text()
    assert code.count("biguint-metadata-candidate-v2406") == 1 and code.count("biguint-metadata-proof-v2407") == 1
    code = code.replace("biguint-metadata-candidate-v2406", "biguint-metadata-candidate-v2413").replace("biguint-metadata-proof-v2407", "biguint-metadata-proof-v2414")
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({
        "status": "Forked before rebuilding or verifying the amended preview; failed Clippy phase retained",
        "repair_runner_sha256": sha(Path(__file__)), "parent_preparation_sha256": sha(parent),
        "failed_result_sha256": sha(failed / "result.json"), "failed_stderr_sha256": sha(failed / "stderr.log"),
        "preparation_sha256": sha(preparation), "parent_proof_runner_sha256": sha(old_runner), "proof_runner_sha256": sha(runner),
        "changes": ["Use the equivalent is_empty check in both shared bit-count helpers.",
            "Narrowly permit the original intrinsic unsafe blocks on modern compilers; retain those blocks for older supported compiler signatures.",
            "Use clear() for the two original multiplication scratch-vector truncate(0) calls; preserve capacity and all subsequent resize/normalization operations."],
        "scope": "The original Clippy failure remains failed. New proof, controls and ordinary-Rust checks must run on this amendment. Additional upstream maintenance is disclosed and is not covered by the conditional metadata proof. No SOURCE change, fixed-baseline scalar admission or MSRV claim.",
        "source_changed": False, "runtime_admitted": False,
    }, indent=2) + "\n")
    print(json.dumps({"preparation": preparation.name, "proof_runner": runner.name, "new_helper_findings": 1, "upstream_host_findings": 4}))


if __name__ == "__main__":
    main()
