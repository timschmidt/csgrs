"""Freeze the retained cost binaries and fork only candidate/baseline identities."""
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())


def main():
    out = E / "bound-decoder-cost-protocol-v2240.json"
    assert not out.exists()
    proof = read(E / "bound-decoder-proof-validation-v2171.json")
    assert proof["source_record_sha256"] == "80bb17679a0891ff93eba8ed10b9f63c205f3ccb39e35dbb3e187e23bb0b7162"
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
               f"typed-api-prototype-v{new}-bound-decoder-{name}-cost"
               for old, new, name in zip(range(1773, 1780), range(2240, 2247), names)}
    common = {
        "typed-api-prototype-v1766-domain-predicate-release-build": "typed-api-prototype-v2174-bound-decoder-release-build",
        "typed-api-prototype-v1761-domain-predicate-verus": "typed-api-prototype-v2169-bound-decoder-verus",
        **mapping,
    }
    fork("run_domain_predicate_costs.py", "run_bound_decoder_costs.py", {
        **common,
        "typed-api-prototype-v1764-domain-predicate-scalar-tests": "typed-api-prototype-v2172-bound-decoder-scalar-tests",
        "domain-predicate-cost-orchestration-v1779.json": "bound-decoder-cost-orchestration-v2246.json",
        "conditional domain-predicate extraction": "proved bound-decoder proposal",
    })
    earlier = {f"typed-api-prototype-v{old}-domain-bound-{name}-cost":
               f"typed-api-prototype-v{new}-domain-predicate-{name}-cost"
               for old, new, name in zip(range(1703, 1710), range(1773, 1780), names)}
    fork("replay_domain_predicate_cost.py", "replay_bound_decoder_cost.py", {
        **common, **earlier,
        "domain-predicate-preconditioned-cost-v1780": "bound-decoder-preconditioned-cost-v2247",
        "domain-predicate-cost-cpu-prelude-v1780": "bound-decoder-cost-cpu-prelude-v2247",
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
        "status": "Frozen before running fresh standalone or paired cost measurements; the earlier seven-binary static comparison found no complete executable identity, so reuse is unavailable. All prior failed comparisons and diagnostics remain separate",
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
