"""Freeze the existing cost-repeat rule and fork its audits before any new timings."""
import copy
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    protocol_path = E / "cache-state-merge-cost-protocol-v1936.json"
    protocol = read(protocol_path)
    expected = "555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59"
    assert protocol["source_record_sha256"] == expected
    out = E / "cache-state-merge-cost-audit-preparation-v1943.json"
    strategy_path = E / "cache-state-merge-cost-followup-strategy-v1943.json"
    assert not out.exists() and not strategy_path.exists()
    assert not (E / "cache-state-merge-cost-orchestration-v1942.json").exists()
    assert not (E / "cache-state-merge-preconditioned-cost-v1943").exists()
    pairs = [
        ("record_bound_merge_initial_costs.py", "record_cache_state_merge_initial_costs.py"),
        ("prepare_bound_merge_cost_followup.py", "prepare_cache_state_merge_cost_followup.py"),
        ("record_bound_merge_costs.py", "record_cache_state_merge_costs.py"),
        ("run_bound_merge_cost_validation.py", "run_cache_state_merge_cost_validation.py"),
        ("record_bound_merge_cost_milestone.py", "record_cache_state_merge_cost_milestone.py"),
    ]
    forks = {}
    for old, new in pairs:
        predecessor, target = E / old, E / new
        patch = E / (new.removesuffix(".py") + "-v1943.patch")
        assert not target.exists() and not patch.exists()
        original = predecessor.read_text()
        source = re.sub(r"v(\d+)\b", lambda match:
            "v" + str(int(match[1]) + 86) if 1838 <= int(match[1]) <= 1858 else match[0], original)
        source = source.replace("bound-merge", "cache-state-merge").replace("bound_merge", "cache_state_merge")
        source = source.replace("run_cache_state_merge_replays.py", "run_cache_state_replays.py")
        source = source.replace("2cdd765b340ce6d0a331aeb25b2596639bc262bb18805cb7efeed27aa43143b9", expected)
        compile(source, str(target), "exec")
        target.write_text(source)
        patch.write_text("".join(difflib.unified_diff(original.splitlines(keepends=True), source.splitlines(keepends=True),
                        fromfile=old, tofile=new, n=0)))
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(predecessor),
            "runner_sha256": sha(target), "fork_patch": patch.name, "fork_patch_sha256": sha(patch)}
    predecessor_strategy = E / "bound-merge-cost-followup-strategy-v1857.json"
    original_strategy = read(predecessor_strategy)
    strategy = copy.deepcopy(original_strategy)
    strategy["source_record_sha256"] = expected
    strategy["cost_protocol"] = protocol_path.name
    strategy["cost_protocol_sha256"] = sha(protocol_path)
    assert len(strategy["family_fixtures"]) == 7 and sum(map(len, strategy["family_fixtures"].values())) == 52
    assert strategy["repetitions"] == 27 and strategy["cpu_prelude_iterations"] == 50000000
    assert strategy["address_space_mib"] == 192 and strategy["cpu_seconds"] == [2, 3] and strategy["wall_seconds"] == 4
    for key in original_strategy.keys() - {"source_record_sha256", "cost_protocol", "cost_protocol_sha256"}:
        assert strategy[key] == original_strategy[key]
    strategy_path.write_text(json.dumps(strategy, indent=2) + "\n")
    out.write_text(json.dumps({"source_record_sha256": expected, "runner_forks": forks,
        "strategy": strategy_path.name, "strategy_sha256": sha(strategy_path),
        "strategy_predecessor": predecessor_strategy.name, "strategy_predecessor_sha256": sha(predecessor_strategy),
        "scope": "Only source/candidate identities change. Preserve every original receipt audit, all 52 full-size fixtures, original caps, 9 initial pairs, 27 family-wide repeat pairs, all non-timing fields and original failures. The failed v1857 comparison remains a failed experiment for its own source; it is not overwritten or qualified by these new measurements."}, indent=2) + "\n")
    print(json.dumps({"prepared_audits": list(forks), "repeat_strategy_frozen": True,
        "fixtures": 52, "family_repetitions": 27, "timings_started": False}))


if __name__ == "__main__":
    main()
