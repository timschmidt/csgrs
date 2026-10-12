"""Keep support lemma visibility within the crate that owns the retained models."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "word-scan-proposal-v2375.rs"
    runner_parent = E / "probe_word_scan_v2375.py"
    proposal = E / "word-scan-proposal-v2376.rs"
    runner = E / "probe_word_scan_v2376.py"
    record = E / "word-scan-probe-preparation-v2376.json"
    assert all(not p.exists() for p in [proposal, runner, record])
    failed = E / "word-scan-probe-v2375"
    assert json.loads((failed / "result.json").read_text())["exit_code"] == 1
    assert "cannot refer to private function" in (failed / "stderr.log").read_text()
    text = parent.read_text()
    assert text.count("pub proof fn") == 18 and text.count("pub open spec fn") == 12
    text = text.replace("pub proof fn", "pub(crate) proof fn")
    text = text.replace("pub open spec fn", "pub(crate) open spec fn")
    proposal.write_text(text)
    code = runner_parent.read_text()
    for old, new in [(parent.name, proposal.name), ("word-scan-probe-v2375", "word-scan-probe-v2376")]:
        assert code.count(old) == 1
        code = code.replace(old, new)
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({"status": "Frozen before the crate-visibility correction probe",
        "preparation_sha256": sha(Path(__file__)), "parent_proposal_sha256": sha(parent),
        "parent_runner_sha256": sha(runner_parent), "proposal_sha256": sha(proposal),
        "runner_sha256": sha(runner), "failed_result_sha256": sha(failed / "result.json"),
        "change": "Use pub(crate) for all proposed proof/spec items, matching the retained crate-private digit and limb models. Mathematical contracts and proof bodies are unchanged.",
        "source_changed": False, "prior_failures_preserved": True}, indent=2) + "\n")
    print(json.dumps({"frozen_visibility_correction": True}))


if __name__ == "__main__":
    main()
