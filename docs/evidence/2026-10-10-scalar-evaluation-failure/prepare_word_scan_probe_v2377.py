"""Exercise the typed-word model in the same module scope as the intended target."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "word-scan-proposal-v2376.rs"
    runner_parent = E / "probe_word_scan_v2376.py"
    proposal = E / "word-scan-proposal-v2377.rs"
    runner = E / "probe_word_scan_v2377.py"
    record = E / "word-scan-probe-preparation-v2377.json"
    assert all(not p.exists() for p in [proposal, runner, record])
    failed = E / "word-scan-probe-v2376"
    assert json.loads((failed / "result.json").read_text())["exit_code"] == 1
    assert "marked `open` but not marked `pub`" in (failed / "stderr.log").read_text()
    text = parent.read_text()
    anchor = "use crate::digit_scan_model::*;"
    assert text.count(anchor) == 1
    text = text.replace(anchor, "mod word_scan_model {\n" + anchor) + "}\n"
    proposal.write_text(text)
    code = runner_parent.read_text()
    for old, new in [(parent.name, proposal.name), ("word-scan-probe-v2376", "word-scan-probe-v2377")]:
        assert code.count(old) == 1
        code = code.replace(old, new)
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({"status": "Frozen before the intended-module-scope probe",
        "preparation_sha256": sha(Path(__file__)), "parent_proposal_sha256": sha(parent),
        "parent_runner_sha256": sha(runner_parent), "proposal_sha256": sha(proposal),
        "runner_sha256": sha(runner), "failed_result_sha256": sha(failed / "result.json"),
        "change": "Place the unchanged crate-visible spec and proof items inside word_scan_model, matching the intended proof target's module scope and the retained model modules. No contract, proof body or imported definition changes.",
        "source_changed": False, "prior_failures_preserved": True}, indent=2) + "\n")
    print(json.dumps({"frozen_module_scope_correction": True}))


if __name__ == "__main__":
    main()
