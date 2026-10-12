"""Fork the failed connection probe with its explicit natural-index conversion."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "digit-scan-connected-proposal-v2360.rs"
    runner_parent = E / "probe_digit_scan_connections_v2360.py"
    proposal = E / "digit-scan-connected-proposal-v2361.rs"
    runner = E / "probe_digit_scan_connections_v2361.py"
    record = E / "digit-scan-connection-preparation-v2361.json"
    assert all(not p.exists() for p in [proposal, runner, record])
    failed = E / "digit-scan-connections-probe-v2360"
    result = json.loads((failed / "result.json").read_text())
    assert result["exit_code"] == 1
    assert 'expected `nat`, found `int`' in (failed / "stderr.log").read_text()
    text = parent.read_text()
    before = "width * (digits.len() - 1) + top_bits),"
    after = "width * ((digits.len() - 1) as nat) + top_bits),"
    assert text.count(before) == 1
    proposal.write_text(text.replace(before, after))
    code = runner_parent.read_text()
    for before, after in [(parent.name, proposal.name),
        ("digit-scan-connections-probe-v2360", "digit-scan-connections-probe-v2361")]:
        assert code.count(before) == 1
        code = code.replace(before, after)
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({
        "status": "Frozen before rerunning the connection probe",
        "preparation_sha256": sha(Path(__file__)),
        "parent_proposal_sha256": sha(parent), "parent_runner_sha256": sha(runner_parent),
        "failed_phase_result_sha256": sha(failed / "result.json"),
        "proposal_sha256": sha(proposal), "runner_sha256": sha(runner),
        "change": "Convert the normalized, nonempty last-digit index from mathematical int to nat in the bit-length expression. The existing nonempty premise establishes its range.",
        "source_changed": False, "prior_failure_preserved": True,
    }, indent=2) + "\n")
    print(json.dumps({"frozen_connection_correction": True}))


if __name__ == "__main__":
    main()
