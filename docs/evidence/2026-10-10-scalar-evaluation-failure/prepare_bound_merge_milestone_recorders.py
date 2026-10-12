"""Freeze result recorders and the timing follow-up selection before measurements."""
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    provenance = E / "bound-merge-milestone-recorder-forks-v1857.json"
    strategy_path = E / "bound-merge-cost-followup-strategy-v1857.json"
    assert not provenance.exists() and not strategy_path.exists()
    pending, forks = {}, {}

    def fork(old, new, replacements):
        source = (E / old).read_text()
        assert not (E / new).exists()
        assert all(before in source for before in replacements), old
        pattern = "|".join(re.escape(value) for value in sorted(replacements, key=len, reverse=True))
        updated = re.sub(pattern, lambda match: replacements[match[0]], source)
        compile(updated, str(E / new), "exec")
        pending[new] = updated
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(E / old),
            "fork_sha256": hashlib.sha256(updated.encode()).hexdigest(), "literal_replacements": replacements}

    fork("record_domain_predicate_broad_milestone.py", "record_bound_merge_broad_milestone.py", {
        "domain-predicate-broad-validation-v1772.json": "bound-merge-broad-validation-v1849.json",
        "domain-predicate-candidate-validation-v1772.json": "bound-merge-candidate-validation-v1849.json",
        "domain-predicate-proof-validation-v1763.json": "bound-merge-proof-validation-v1840.json",
        "domain-predicate-broad-milestone-v1772.json": "bound-merge-broad-milestone-v1849.json",
        "domain-predicate-broad-ledger-v1772.json": "bound-merge-broad-ledger-v1849.json",
        "typed-api-prototype-v1766-domain-predicate-release-build": "typed-api-prototype-v1843-bound-merge-release-build",
        "v1754": "v1836", "645 verified units, 226 rejected mutations": "651 verified units, 246 rejected mutations",
        "domain-predicate-cost-protocol-v1773.json": "bound-merge-cost-protocol-v1850.json",
        "domain-predicate-replay-protocol-v1781.json": "bound-merge-replay-protocol-v1858.json",
        "compare_domain_predicate_queries.py": "compare_bound_merge_queries.py",
        "compare_domain_predicate_replays.py": "compare_bound_merge_replays.py",
        "audit_domain_predicate_fact_retention.py": "audit_bound_merge_fact_retention.py",
        "domain-predicate-fact-audit-preparation-v1835.json": "bound-merge-replay-protocol-v1858.json",
        "v1764-v1772": "v1841-v1849", "v1761 proof source": "v1838 proof source",
        'Next validate its runtime extraction against the unchanged tests, public/capped replays "\n           "and retained cost controls before deciding retention.':
            'Next validate its runtime extraction against unchanged scalar/consumer tests, cost "\n           "controls and capped/numerical/public replays before deciding retention.',
    })
    fork("record_domain_predicate_initial_costs.py", "record_bound_merge_initial_costs.py", {
        "domain-predicate-initial-cost-review-v1780.json": "bound-merge-initial-cost-review-v1857.json",
        "domain-predicate-cost-protocol-v1773.json": "bound-merge-cost-protocol-v1850.json",
        "domain-predicate-cost-orchestration-v1779.json": "bound-merge-cost-orchestration-v1856.json",
        "domain-predicate-preconditioned-cost-v1780": "bound-merge-preconditioned-cost-v1857",
        "replay_domain_predicate_cost.py": "replay_bound_merge_cost.py",
    })
    protocol_path = E / "bound-merge-cost-protocol-v1850.json"
    protocol = read(protocol_path)
    assert not (E / "bound-merge-cost-orchestration-v1856.json").exists()
    assert not (E / "bound-merge-preconditioned-cost-v1857").exists()
    fixtures = {}
    for family, item in protocol["families"].items():
        commands = read(E / item["baseline_phase"] / "commands.json")
        selected = []
        for row in commands:
            if row["name"] in {"format", "build"}:
                continue
            args = row["command"][1:]
            if args not in selected:
                selected.append(args)
        fixtures[family] = selected
    assert sum(map(len, fixtures.values())) == 52 and len(fixtures) == 7
    strategy = {
        "status": "Frozen before any candidate cost process or paired timing observation",
        "source_record_sha256": protocol["source_record_sha256"],
        "cost_protocol": protocol_path.name, "cost_protocol_sha256": sha(protocol_path),
        "trigger": "Any complete fixture whose original nine-pair median candidate increase is greater than five percent.",
        "selection": "For each affected family, repeat every original fixture in that family, in the existing runner's family/fixture order. This retains all larger and smaller unchanged controls and does not select controls using their timing outcomes.",
        "repetitions": 27, "address_space_mib": 192, "cpu_seconds": [2, 3], "wall_seconds": 4,
        "cpu_prelude_iterations": 50000000, "family_fixtures": fixtures,
        "original_failure_rule": "A failed original-cap process remains failed. Preserve all completed and partial observations; this timing-repeat rule cannot convert an original failure into success.",
        "interpretation": "Keep initial and follow-up observations. A persistent increase above five percent prevents declaring the local proof-only extraction cost gate passed. No uniform-speedup or causal claim follows; all older failures/tradeoffs and the fixed pre-Verus baseline remain separate.",
    }
    for name, source in pending.items():
        (E / name).write_text(source)
        assert sha(E / name) == forks[name]["fork_sha256"]
    strategy_path.write_text(json.dumps(strategy, indent=2) + "\n")
    provenance.write_text(json.dumps({"runner_forks": forks,
        "followup_strategy": strategy_path.name, "followup_strategy_sha256": sha(strategy_path),
        "scope": "Result recorders retain their prior assertions and parsers. Identity/proof-count literals and the current ledger transition change explicitly; the repeat selection is frozen before observing candidate timings."}, indent=2) + "\n")
    print(json.dumps({"recorders": list(forks), "followup_strategy": strategy_path.name,
                      "unchanged_family_fixtures": sum(map(len, fixtures.values()))}))


if __name__ == "__main__":
    main()
