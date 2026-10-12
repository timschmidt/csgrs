"""Fork the connection proof to use the pinned public u32 bit-position contract."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "digit-scan-connected-proposal-v2361.rs"
    runner_parent = E / "probe_digit_scan_connections_v2361.py"
    leading = E / "u32-digit-bit-length-proposal-v2362.rs"
    proposal = E / "digit-scan-connected-proposal-v2362.rs"
    runner = E / "probe_digit_scan_connections_v2362.py"
    record = E / "digit-scan-connection-preparation-v2362.json"
    assert all(not p.exists() for p in [proposal, runner, record])
    failed = E / "digit-scan-connections-probe-v2361"
    result = json.loads((failed / "result.json").read_text())
    assert result["exit_code"] == 1
    assert "marked 'closed'" in (failed / "stderr.log").read_text()
    text = parent.read_text()
    start = text.index("pub proof fn u32_digit_bit_length(")
    end = text.index("\n}\n", start) + 3
    proposal.write_text(text[:start] + leading.read_text() + text[end:])
    code = runner_parent.read_text()
    for before, after in [(parent.name, proposal.name),
        ("digit-scan-connections-probe-v2361", "digit-scan-connections-probe-v2362")]:
        assert code.count(before) == 1
        code = code.replace(before, after)
    ast.parse(code)
    runner.write_text(code)
    sources = {}
    for name, filename in [("std_specs/bits.rs", "/tmp/csgrs-pinned-verus-bits-671956e.rs"),
                           ("bits.rs", "/tmp/csgrs-pinned-verus-bit-lemmas-671956e.rs")]:
        sources[name] = {"url": "https://raw.githubusercontent.com/verus-lang/verus/671956ec527d3b7164779f767bdbfe769bedce6c/source/vstd/" + name,
                         "sha256": sha(Path(filename))}
    record.write_text(json.dumps({
        "status": "Frozen before rerunning the connection probe with public u32 contracts",
        "preparation_sha256": sha(Path(__file__)),
        "parent_proposal_sha256": sha(parent), "parent_runner_sha256": sha(runner_parent),
        "failed_phase_result_sha256": sha(failed / "result.json"),
        "replacement_proof_sha256": sha(leading),
        "proposal_sha256": sha(proposal), "runner_sha256": sha(runner),
        "pinned_library_source_inspected": sources,
        "change": "Keep the u32 statement and replace the unavailable recursive reveal with the public top-set-bit and higher-zero-bit guarantees and pinned shift/division lemmas. No new trusted contract or axiom is introduced.",
        "source_changed": False, "prior_failures_preserved": True,
    }, indent=2) + "\n")
    print(json.dumps({"frozen_u32_contract_correction": True}))


if __name__ == "__main__":
    main()
