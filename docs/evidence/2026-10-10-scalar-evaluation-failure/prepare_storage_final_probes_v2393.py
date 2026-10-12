"""Make the mutable post-state explicit without changing storage-operation bodies."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    record = E / "storage-final-probe-preparation-v2393.json"
    assert not record.exists()
    rows = []
    for kind, before_version, after_version in [("shrink", 2389, 2393), ("truncate", 2390, 2394)]:
        before = E / f"storage-{kind}-proposal-v{before_version}.rs"
        parent = E / f"probe_storage_{kind}_v{before_version}.py"
        proposal = E / f"storage-{kind}-proposal-v{after_version}.rs"
        runner = E / f"probe_storage_{kind}_v{after_version}.py"
        assert not proposal.exists() and not runner.exists()
        failure = E / f"storage-{kind}-probe-v{before_version}"
        assert json.loads((failure / "result.json").read_text())["exit_code"] == 1
        assert "disambiguate by wrapping it in either `old` or `final`" in (failure / "stderr.log").read_text()
        lines = before.read_text().splitlines(True)
        assert sum(line.startswith("    ensures ") for line in lines) == 2
        text = "".join(line.replace("words@", "final(words)@") if line.startswith("    ensures ") else line for line in lines)
        proposal.write_text(text)
        code = parent.read_text()
        for old, new in [(before.name, proposal.name),
            (f"storage-{kind}-probe-v{before_version}", f"storage-{kind}-probe-v{after_version}")]:
            assert code.count(old) == 1
            code = code.replace(old, new)
        ast.parse(code)
        runner.write_text(code)
        rows.append({"kind": kind, "prior_phase": failure.name, "prior_result_sha256": sha(failure / "result.json"),
            "prior_proposal_sha256": sha(before), "prior_runner_sha256": sha(parent),
            "proposal": proposal.name, "proposal_sha256": sha(proposal),
            "runner": runner.name, "runner_sha256": sha(runner), "phase": f"storage-{kind}-probe-v{after_version}"})
    record.write_text(json.dumps({"status": "Frozen before the explicit mutable post-state probes",
        "preparation_sha256": sha(Path(__file__)), "probes": rows, "source_changed": False,
        "change": "Wrap the mutable post-state word view in final(words), as required by the pinned toolchain. Preserve the old state and every statement in the operation body. No new assumption or library specification is introduced; the original failures remain archived."}, indent=2) + "\n")
    print(json.dumps({"frozen_mutable_poststate_probes": len(rows), "source_changed": False}))


if __name__ == "__main__":
    main()
