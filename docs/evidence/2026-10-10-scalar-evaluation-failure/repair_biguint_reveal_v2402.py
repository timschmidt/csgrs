"""Fork the frozen preview to use explicit BigUint names in ghost reveals."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "prepare_biguint_metadata_candidate_v2400.py"
    preparation = E / "prepare_biguint_metadata_candidate_v2402.py"
    old_runner = E / "probe_biguint_metadata_v2401.py"
    runner = E / "probe_biguint_metadata_v2403.py"
    record = E / "biguint-reveal-correction-v2402.json"
    assert all(not p.exists() for p in [preparation, runner, record, E / "biguint-metadata-candidate-v2402", E / "biguint-metadata-proof-v2403"])
    failed = E / "biguint-metadata-proof-v2401"
    result = json.loads((failed / "result.json").read_text())
    assert result["exit_code"] == 1 and result["preview_unchanged"] and result["source_unchanged"]
    assert (failed / "stderr.log").read_text().count("Self is not supported in reveal/hide") == 7
    code = parent.read_text()
    assert code.count("reveal(Self::") == 7 and code.count("biguint-metadata-candidate-v2400") == 1
    code = code.replace("reveal(Self::", "reveal(BigUint::").replace("biguint-metadata-candidate-v2400", "biguint-metadata-candidate-v2402")
    ast.parse(code)
    preparation.write_text(code)
    code = old_runner.read_text()
    assert code.count("biguint-metadata-candidate-v2400") == 1 and code.count("biguint-metadata-proof-v2401") == 1
    code = code.replace("biguint-metadata-candidate-v2400", "biguint-metadata-candidate-v2402").replace("biguint-metadata-proof-v2401", "biguint-metadata-proof-v2403")
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({
        "status": "Forked before rerunning preparation or proof; failed v2401 retained",
        "repair_runner_sha256": sha(Path(__file__)), "parent_preparation_sha256": sha(parent),
        "failed_result_sha256": sha(failed / "result.json"), "failed_stderr_sha256": sha(failed / "stderr.log"),
        "preparation_sha256": sha(preparation), "parent_proof_runner_sha256": sha(old_runner), "proof_runner_sha256": sha(runner),
        "change": "Only seven erased reveal targets change from Self to the explicit BigUint type required by the pinned macro; runtime bodies and proof requirements are unchanged.",
        "source_changed": False, "retained_coverage_changed": False,
    }, indent=2) + "\n")
    print(json.dumps({"preparation": preparation.name, "proof_runner": runner.name, "ghost_reveal_corrections": 7}))


if __name__ == "__main__":
    main()
