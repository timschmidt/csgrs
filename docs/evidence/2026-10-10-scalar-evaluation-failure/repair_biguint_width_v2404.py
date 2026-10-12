"""Fork the preview to verify its unchanged native-width constants."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "prepare_biguint_metadata_candidate_v2402.py"
    preparation = E / "prepare_biguint_metadata_candidate_v2404.py"
    old_runner = E / "probe_biguint_metadata_v2403.py"
    runner = E / "probe_biguint_metadata_v2405.py"
    record = E / "biguint-width-correction-v2404.json"
    assert all(not p.exists() for p in [preparation, runner, record, E / "biguint-metadata-candidate-v2404", E / "biguint-metadata-proof-v2405"])
    failed = E / "biguint-metadata-proof-v2403"
    result = json.loads((failed / "result.json").read_text())
    assert result["exit_code"] == 1 and result["preview_unchanged"] and result["source_unchanged"]
    assert (failed / "stderr.log").read_text().count("which is ignored because") == 2
    code = parent.read_text()
    anchor = "pub(crate) const WIDTH: u64 = BITS;"
    assert code.count(anchor) == 1 and code.count("biguint-metadata-candidate-v2402") == 1
    code = code.replace(anchor, "#[cfg_attr(verus_keep_ghost, verus_spec(ensures WIDTH == BITS,))]\n" + anchor)
    code = code.replace("biguint-metadata-candidate-v2402", "biguint-metadata-candidate-v2404")
    ast.parse(code)
    preparation.write_text(code)
    code = old_runner.read_text()
    assert code.count("biguint-metadata-candidate-v2402") == 1 and code.count("biguint-metadata-proof-v2403") == 1
    code = code.replace("biguint-metadata-candidate-v2402", "biguint-metadata-candidate-v2404").replace("biguint-metadata-proof-v2403", "biguint-metadata-proof-v2405")
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({
        "status": "Forked before rerunning preparation or proof; failed v2403 retained",
        "repair_runner_sha256": sha(Path(__file__)), "parent_preparation_sha256": sha(parent),
        "failed_result_sha256": sha(failed / "result.json"), "failed_stderr_sha256": sha(failed / "stderr.log"),
        "preparation_sha256": sha(preparation), "parent_proof_runner_sha256": sha(old_runner), "proof_runner_sha256": sha(runner),
        "change": "Add an erased verus_spec on each unchanged WIDTH constant, proving its existing literal value. No executable body, precondition or postcondition of the metadata operations changes.",
        "source_changed": False, "retained_coverage_changed": False,
    }, indent=2) + "\n")
    print(json.dumps({"preparation": preparation.name, "proof_runner": runner.name, "width_constants_marked": 2}))


if __name__ == "__main__":
    main()
