"""Fork the typed-word probe with disambiguated cast/comparison syntax."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "word-scan-proposal-v2374.rs"
    runner_parent = E / "probe_word_scan_v2374.py"
    proposal = E / "word-scan-proposal-v2375.rs"
    runner = E / "probe_word_scan_v2375.py"
    record = E / "word-scan-probe-preparation-v2375.json"
    assert all(not p.exists() for p in [proposal, runner, record])
    failed = E / "word-scan-probe-v2374"
    assert json.loads((failed / "result.json").read_text())["exit_code"] == 1
    assert "expected `,`" in (failed / "stderr.log").read_text()
    text = parent.read_text()
    for width in [32, 64]:
        for kind in ["zeros", "ones"]:
            old = f"u{width}_trailing_{kind}(words[index]) as nat < {width}"
            new = f"(u{width}_trailing_{kind}(words[index]) as nat) < {width}"
            assert text.count(old) == 1
            text = text.replace(old, new)
    proposal.write_text(text)
    code = runner_parent.read_text()
    for old, new in [(parent.name, proposal.name), ("word-scan-probe-v2374", "word-scan-probe-v2375")]:
        assert code.count(old) == 1
        code = code.replace(old, new)
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({"status": "Frozen before the typed-word syntax correction probe",
        "preparation_sha256": sha(Path(__file__)), "parent_proposal_sha256": sha(parent),
        "parent_runner_sha256": sha(runner_parent), "proposal_sha256": sha(proposal),
        "runner_sha256": sha(runner), "failed_result_sha256": sha(failed / "result.json"),
        "change": "Parenthesize each cast immediately before a less-than operator; all mathematical premises and results are unchanged.",
        "source_changed": False, "prior_failure_preserved": True}, indent=2) + "\n")
    print(json.dumps({"frozen_syntax_correction": True}))


if __name__ == "__main__":
    main()
