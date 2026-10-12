"""Freeze the retained cost binaries and fork only candidate/baseline identities."""
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())


def main():
    out = E / "integer-coarse-zero-cost-protocol-v2093.json"
    assert not out.exists()
    proof = read(E / "integer-coarse-zero-proof-validation-v2083.json")
    assert proof["source_record_sha256"] == "e093cd7ed95ae15ecb66d04cd76beb89b04aa205971330f4b2794db4190ded62"
    forks, pending = {}, {}

    def fork(old, new, replacements):
        original = (E / old).read_text()
        assert not (E / new).exists()
        for before in replacements:
            assert before in original, (old, before)
        pattern = "|".join(re.escape(value) for value in sorted(replacements, key=len, reverse=True))
        updated = re.sub(pattern, lambda match: replacements[match[0]], original)
        compile(updated, str(E / new), "exec")
        pending[new] = updated
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(E / old),
                      "fork_sha256": hashlib.sha256(updated.encode()).hexdigest(),
                      "literal_replacements": replacements}

    names = ["atan", "integer", "inverse", "normal-form", "default", "algebraic", "acos"]
    mapping = {f"typed-api-prototype-v{old}-domain-predicate-{name}-cost":
               f"typed-api-prototype-v{new}-integer-coarse-zero-{name}-cost"
               for old, new, name in zip(range(1773, 1780), range(2093, 2100), names)}
    common = {
        "typed-api-prototype-v1766-domain-predicate-release-build": "typed-api-prototype-v2086-integer-coarse-zero-release-build",
        "typed-api-prototype-v1761-domain-predicate-verus": "typed-api-prototype-v2082-integer-coarse-zero-verus",
        **mapping,
    }
    fork("run_domain_predicate_costs.py", "run_integer_coarse_zero_costs.py", {
        **common,
        "typed-api-prototype-v1764-domain-predicate-scalar-tests": "typed-api-prototype-v2084-integer-coarse-zero-scalar-tests",
        "domain-predicate-cost-orchestration-v1779.json": "integer-coarse-zero-cost-orchestration-v2099.json",
        "conditional domain-predicate extraction": "combined integer shortcut, root kernel and outlined-cache proposal",
    })
    earlier = {f"typed-api-prototype-v{old}-domain-bound-{name}-cost":
               f"typed-api-prototype-v{new}-domain-predicate-{name}-cost"
               for old, new, name in zip(range(1703, 1710), range(1773, 1780), names)}
    fork("replay_domain_predicate_cost.py", "replay_integer_coarse_zero_cost.py", {
        **common, **earlier,
        "domain-predicate-preconditioned-cost-v1780": "integer-coarse-zero-preconditioned-cost-v2100",
        "domain-predicate-cost-cpu-prelude-v1780": "integer-coarse-zero-cost-cpu-prelude-v2100",
        "typed-api-prototype-v1754-state.json": "typed-api-prototype-v1836-state.json",
        "domain-bound-cost-review-v1710.json": "domain-predicate-cost-review-v1780.json",
        "Baseline is the retained v1754 magnitude-domain candidate":
            "Baseline is the retained v1836 domain-predicate candidate",
    })
    checkpoint = E / proof["retained_runtime_checkpoint"]
    assert sha(checkpoint) == proof["retained_runtime_checkpoint_sha256"]
    retained = read(checkpoint)["artifacts_sha256"]
    cost_path = E / "domain-predicate-cost-review-v1780.json"
    assert sha(cost_path) == retained[cost_path.name]
    costs = read(cost_path)
    binaries = {}
    for family, item in costs["families"].items():
        phase = item["phases"]["candidate"]
        assert sha(E / phase / "summary.json") == retained[phase + "/summary.json"]
        commands = [row for row in read(E / phase / "commands.json") if row["name"] not in {"format", "build"}]
        binary = Path(commands[0]["command"][0])
        assert sha(binary) == item["binary_sha256"]["candidate"]
        binaries[family] = {"baseline_phase": phase, "candidate_phase": mapping[phase],
            "binary": str(binary), "binary_sha256": sha(binary), "original_processes": len(commands),
            "probe_sha256": sha(E / phase / "probe.rs"), "runner_sha256": sha(E / phase / "reproduce.py")}
    assert sum(row["original_processes"] for row in binaries.values()) == 468
    previous = read(E / "domain-predicate-cost-protocol-v1773.json")
    record = {
        "status": "Frozen before building or measuring cost binaries for this combined proposal; all prior failed comparisons and diagnostics remain separate",
        "source_record": proof["source_record"], "source_record_sha256": proof["source_record_sha256"],
        "retained_runtime_checkpoint": checkpoint.name, "retained_runtime_checkpoint_sha256": sha(checkpoint),
        "retained_cost_record": cost_path.name, "retained_cost_record_sha256": sha(cost_path),
        "runner_forks": forks, "families": binaries, "protocol": previous["protocol"],
        "protocol_predecessor": "domain-predicate-cost-protocol-v1773.json",
        "protocol_predecessor_sha256": sha(E / "domain-predicate-cost-protocol-v1773.json"),
    }
    for name, source in pending.items():
        (E / name).write_text(source)
        assert sha(E / name) == forks[name]["fork_sha256"]
    out.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"forked_runners": list(forks), "families": len(binaries),
                      "standalone_processes": 468, "paired_processes": 936}))


if __name__ == "__main__":
    main()
