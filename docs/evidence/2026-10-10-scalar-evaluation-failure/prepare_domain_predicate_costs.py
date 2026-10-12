"""Fork immutable cost runners and freeze unchanged workloads before runtime builds."""
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())


def main():
    out = E / "domain-predicate-cost-protocol-v1773.json"
    assert not out.exists()
    forks = {}

    def fork(old, new, replacements):
        original = (E / old).read_text()
        assert not (E / new).exists()
        for before in replacements:
            assert before in original, (old, before)
        pattern = "|".join(re.escape(value) for value in sorted(replacements, key=len, reverse=True))
        updated = re.sub(pattern, lambda match: replacements[match[0]], original)
        compile(updated, str(E / new), "exec")
        (E / new).write_text(updated)
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(E / old),
                      "fork_sha256": sha(E / new), "literal_replacements": replacements}

    old_names = ["atan", "integer", "inverse", "normal-form", "default", "algebraic", "acos"]
    mapping = {f"typed-api-prototype-v{old}-domain-bound-{name}-cost":
               f"typed-api-prototype-v{new}-domain-predicate-{name}-cost"
               for old, new, name in zip(range(1703, 1710), range(1773, 1780), old_names)}
    common = {
        "typed-api-prototype-v1696-domain-bound-release-build": "typed-api-prototype-v1766-domain-predicate-release-build",
        "read(E/'domain-bound-candidate-caps-v1693.json')['source_record_sha256']":
            "sha(E/'typed-api-prototype-v1761-domain-predicate-verus/source-hashes.json.gz')",
        **mapping,
    }
    fork("run_domain_bound_costs.py", "run_domain_predicate_costs.py", {
        **common,
        "typed-api-prototype-v1694-domain-bound-scalar-tests": "typed-api-prototype-v1764-domain-predicate-scalar-tests",
        "domain-bound-cost-orchestration-v1709.json": "domain-predicate-cost-orchestration-v1779.json",
        "magnitude-domain extension": "conditional domain-predicate extraction",
    })
    earlier = {f"typed-api-prototype-v{old}-domain-sign-{name}-cost":
               f"typed-api-prototype-v{new}-domain-bound-{name}-cost"
               for old, new, name in zip(range(1631, 1638), range(1703, 1710), old_names)}
    fork("replay_domain_bound_cost.py", "replay_domain_predicate_cost.py", {
        **common, **earlier,
        "domain-bound-preconditioned-cost-v1710": "domain-predicate-preconditioned-cost-v1780",
        "domain-bound-cost-cpu-prelude-v1710": "domain-predicate-cost-cpu-prelude-v1780",
        "typed-api-prototype-v1672-state.json": "typed-api-prototype-v1754-state.json",
        "domain-sign-cost-review-v1639.json": "domain-bound-cost-review-v1710.json",
        "Baseline is the retained v1672 domain-sign candidate, checked against its committed checkpoint and binary provenance. Its earlier narrow integer cost tradeoff and failed v1638 baseline guard remain separate retained evidence.":
            "Baseline is the retained v1754 magnitude-domain candidate, checked against its committed checkpoint and binary provenance. Earlier original-cap failures at v1638 and v1710 and the narrow integer cost tradeoff remain separate retained evidence. This proof-only extraction has no new completeness benefit to justify a runtime regression.",
    })
    checkpoint = E / "typed-api-prototype-v1754-state.json"
    retained = read(checkpoint)["artifacts_sha256"]
    costs = read(E / "domain-bound-cost-review-v1710.json")
    assert sha(E / "domain-bound-cost-review-v1710.json") == retained["domain-bound-cost-review-v1710.json"]
    binaries = {}
    for family, item in costs["families"].items():
        phase = item["phases"]["candidate"]
        assert sha(E / phase / "summary.json") == retained[phase + "/summary.json"]
        commands = [row for row in read(E / phase / "commands.json") if row["name"] not in {"format", "build"}]
        binary = Path(commands[0]["command"][0])
        assert sha(binary) == item["binary_sha256"]["candidate"]
        binaries[family] = {"baseline_phase": phase, "candidate_phase": mapping[phase],
                            "binary": str(binary), "binary_sha256": sha(binary),
                            "original_processes": len(commands),
                            "probe_sha256": sha(E / phase / "probe.rs"),
                            "runner_sha256": sha(E / phase / "reproduce.py")}
    assert sum(row["original_processes"] for row in binaries.values()) == 468
    out.write_text(json.dumps({
        "status": "Frozen before building or measuring the runtime extraction",
        "source_record": "typed-api-prototype-v1761-domain-predicate-verus/source-hashes.json.gz",
        "source_record_sha256": sha(E / "typed-api-prototype-v1761-domain-predicate-verus/source-hashes.json.gz"),
        "retained_runtime_checkpoint": checkpoint.name, "retained_runtime_checkpoint_sha256": sha(checkpoint),
        "retained_cost_record": "domain-bound-cost-review-v1710.json", "retained_cost_record_sha256": sha(E / "domain-bound-cost-review-v1710.json"),
        "runner_forks": forks, "families": binaries,
        "protocol": [
            "Keep all seven probes and their 52 full-size fixtures unchanged. Run all 468 standalone trials and all 936 paired processes, unless a real failure stops a phase; preserve every failed attempt.",
            "Use original 192 MiB address-space, CPU 2/3-second and four-second wall caps. Pair retained baseline and candidate binaries in alternating order after the identical 50-million-step CPU prelude; do not overlap builds, source edits or other measurement workloads.",
            "Preserve every non-timing output, allocation count, consumed-unit field, sample and checksum. Compare elapsed times separately and report executable sizes; no uniform speedup claim follows.",
            "Investigate any observed median timing regression above five percent with the same full-size fixture, original caps, both binaries, at least 27 alternating pairs and unchanged controls. Preserve the original observation regardless of the follow-up.",
            "An original-cap failure stays failed. Larger-cap diagnosis may be retained separately and cannot turn that failure into a pass or silently substitute for the frozen workload.",
            "Proof coverage alone provides no completeness or performance benefit that can excuse a runtime regression. The pre-Verus acceptance baseline and all older failures/tradeoffs remain unchanged; these local controls do not admit the full stack.",
        ],
    }, indent=2) + "\n")
    print(json.dumps({"forked_runners": list(forks), "families": len(binaries),
                      "standalone_processes": 468, "paired_processes": 936}))


if __name__ == "__main__":
    main()
