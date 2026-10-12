"""Separate unsupported closure patterns from the iterator call-contract check."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    record = E / "storage-closure-probe-preparation-v2391.json"
    assert not record.exists()
    rows = []
    for kind, before_version, after_version in [("forward", 2387, 2391), ("reverse", 2388, 2392)]:
        before = E / f"storage-{kind}-proposal-v{before_version}.rs"
        parent = E / f"probe_storage_{kind}_v{before_version}.py"
        proposal = E / f"storage-{kind}-proposal-v{after_version}.rs"
        runner = E / f"probe_storage_{kind}_v{after_version}.py"
        assert not proposal.exists() and not runner.exists()
        failure = E / f"storage-{kind}-probe-v{before_version}"
        assert json.loads((failure / "result.json").read_text())["exit_code"] == 1
        assert "only variables are supported here, not general patterns" in (failure / "stderr.log").read_text()
        text = before.read_text()
        for width in [32, 64]:
            start = text.index(f"fn {kind}_u{width}(")
            end = text.index("\n}\n", start) + 3
            function = text[start:end]
            old = "|&digit| digit != 0"
            new = f"|digit: &u{width}| *digit != 0"
            assert function.count(old) == 1
            text = text[:start] + function.replace(old, new) + text[end:]
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
    record.write_text(json.dumps({"status": "Frozen before checking iterator calls with an explicit dereference closure",
        "preparation_sha256": sha(Path(__file__)), "probes": rows, "source_changed": False,
        "change": "Expand the reference pattern on each primitive Copy word into a typed reference parameter and explicit dereference. Keep the iterator call and mathematical contract unchanged. This diagnostic probe is not credited as a proof of the dependency's original closure syntax or implementation; the original failures remain archived."}, indent=2) + "\n")
    print(json.dumps({"frozen_closure_diagnostic_probes": len(rows), "source_changed": False}))


if __name__ == "__main__":
    main()
