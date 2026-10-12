"""Connect the executable word-width constants to explicit ghost width functions."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "prepare_biguint_metadata_candidate_v2404.py"
    preparation = E / "prepare_biguint_metadata_candidate_v2406.py"
    old_runner = E / "probe_biguint_metadata_v2405.py"
    runner = E / "probe_biguint_metadata_v2407.py"
    record = E / "biguint-width-mode-correction-v2406.json"
    assert all(not p.exists() for p in [preparation, runner, record, E / "biguint-metadata-candidate-v2406", E / "biguint-metadata-proof-v2407"])
    failed = E / "biguint-metadata-proof-v2405"
    result = json.loads((failed / "result.json").read_text())
    assert result["exit_code"] == 1 and result["preview_unchanged"] and result["source_unchanged"]
    assert (failed / "stderr.log").read_text().count("cannot read const with mode exec") == 7
    code = parent.read_text()
    anchor = "#[cfg_attr(verus_keep_ghost, verus_spec(ensures WIDTH == BITS,))]"
    assert code.count(anchor) == 1
    assert code.count("WIDTH * words.len()") == 5
    assert code.count("metadata::WIDTH as nat") == 1
    replacement = '''#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn word_width() -> u64 { BITS }
}

#[cfg_attr(verus_keep_ghost, verus_spec(ensures WIDTH == word_width(),))]'''
    code = code.replace(anchor, replacement)
    code = code.replace("WIDTH * words.len()", "word_width() * words.len()")
    code = code.replace("metadata::WIDTH as nat", "metadata::word_width() as nat")
    assert code.count("biguint-metadata-candidate-v2404") == 1
    code = code.replace("biguint-metadata-candidate-v2404", "biguint-metadata-candidate-v2406")
    ast.parse(code)
    preparation.write_text(code)
    code = old_runner.read_text()
    assert code.count("biguint-metadata-candidate-v2404") == 1 and code.count("biguint-metadata-proof-v2405") == 1
    code = code.replace("biguint-metadata-candidate-v2404", "biguint-metadata-candidate-v2406").replace("biguint-metadata-proof-v2405", "biguint-metadata-proof-v2407")
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({
        "status": "Forked before rerunning preparation or proof; failed v2405 retained",
        "repair_runner_sha256": sha(Path(__file__)), "parent_preparation_sha256": sha(parent),
        "failed_result_sha256": sha(failed / "result.json"), "failed_stderr_sha256": sha(failed / "stderr.log"),
        "preparation_sha256": sha(preparation), "parent_proof_runner_sha256": sha(old_runner), "proof_runner_sha256": sha(runner),
        "change": "The pinned verifier distinguishes executable constants from specification expressions. Add one ghost literal width per native helper and prove the existing executable constant equals it. Contracts use the ghost width; executable constant values and operation bodies are unchanged. This equality is verified, not assumed.",
        "source_changed": False, "retained_coverage_changed": False,
    }, indent=2) + "\n")
    print(json.dumps({"preparation": preparation.name, "proof_runner": runner.name, "verified_width_bindings_attempted": 2}))


if __name__ == "__main__":
    main()
