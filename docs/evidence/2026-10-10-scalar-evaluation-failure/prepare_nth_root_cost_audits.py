"""Freeze unchanged cost gates and describe only fields the probes actually emit."""
import ast
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
    expected = "f5fa38edebe8eb5ecdf831098567fab0cf0d8d6f588b56f1c93c4d5f56a5cac3"
    protocol_path = E / "nth-root-enclosure-cost-protocol-v2014.json"
    assert read(protocol_path)["source_record_sha256"] == expected
    out = E / "nth-root-enclosure-cost-audit-preparation-v2021.json"
    strategy_path = E / "nth-root-enclosure-cost-followup-strategy-v2021.json"
    assert not out.exists() and not strategy_path.exists()
    assert not (E / "nth-root-enclosure-cost-orchestration-v2020.json").exists()
    assert not (E / "nth-root-enclosure-preconditioned-cost-v2021").exists()
    corrections = {
        "including allocations, samples, work accounting and checksums":
            "including certification counts, checksums and output fields",
        "including allocations, samples, consumed work and checksums":
            "including certification counts, checksums and output fields",
        "checksums, allocation/work receipts": "checksums and all non-timing output fields",
        "Samples, allocations, work counts and checksums are unchanged.":
            "Certification counts, checksums and all non-timing output fields are unchanged. These probes do not measure allocations or consumed evaluation work.",
        "limits. Samples, allocations, consumed work and checksums are unchanged. All timing ":
            "limits. Certification counts, checksums and non-timing output fields are unchanged. "
            "These probes do not measure allocations or consumed evaluation work. All timing ",
        "The proof-only extraction provides no new completeness gain to justify a runtime regression.":
            "The combined kernel/selector proposal provides no new completeness gain to justify a runtime regression.",
        "unchanged proved nth-root-enclosure source": "unchanged combined kernel/selector source",
        "the extraction": "the combined proposal",
        "The extraction": "The combined proposal",
    }
    forks = {}
    names = ["record_cache_state_merge_initial_costs.py", "prepare_cache_state_merge_cost_followup.py",
             "record_cache_state_merge_costs.py", "run_cache_state_merge_cost_validation.py",
             "record_cache_state_merge_cost_milestone.py"]
    for old in names:
        new = old.replace("cache_state_merge", "nth_root_enclosure")
        predecessor, target = E / old, E / new
        patch = E / (new.removesuffix(".py") + "-v2021.patch")
        assert not target.exists() and not patch.exists()
        original = predecessor.read_text()
        source = re.sub(r"v(\d+)\b", lambda match:
            "v" + str(int(match[1]) + 78) if 1927 <= int(match[1]) <= 1998 else match[0], original)
        source = source.replace("cache-state-merge", "nth-root-enclosure")
        source = source.replace("cache_state_merge", "nth_root_enclosure").replace("cache_state", "nth_root")
        source = source.replace("555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59", expected)
        applied = {}
        for before, after in corrections.items():
            if before in source:
                applied[before] = after
                source = source.replace(before, after)
        ast.parse(source)
        target.write_text(source)
        patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                                   fromfile=old, tofile=new, n=0)))
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(predecessor),
            "runner_sha256": sha(target), "fork_patch": patch.name, "fork_patch_sha256": sha(patch),
            "description_corrections": applied}
    predecessor_strategy = E / "cache-state-merge-cost-followup-strategy-v1943.json"
    original_strategy = read(predecessor_strategy)
    strategy = copy.deepcopy(original_strategy)
    strategy.update(source_record_sha256=expected, cost_protocol=protocol_path.name,
                    cost_protocol_sha256=sha(protocol_path))
    strategy["interpretation"] = strategy["interpretation"].replace("local proof-only extraction", "local combined proposal")
    unchanged = original_strategy.keys() - {"source_record_sha256", "cost_protocol", "cost_protocol_sha256", "interpretation"}
    assert all(strategy[key] == original_strategy[key] for key in unchanged)
    assert len(strategy["family_fixtures"]) == 7 and sum(map(len, strategy["family_fixtures"].values())) == 52
    assert strategy["repetitions"] == 27 and strategy["cpu_prelude_iterations"] == 50000000
    assert strategy["address_space_mib"] == 192 and strategy["cpu_seconds"] == [2, 3] and strategy["wall_seconds"] == 4
    strategy_path.write_text(json.dumps(strategy, indent=2) + "\n")
    out.write_text(json.dumps({
        "source_record_sha256": expected, "runner_forks": forks,
        "strategy": strategy_path.name, "strategy_sha256": sha(strategy_path),
        "strategy_predecessor": predecessor_strategy.name, "strategy_predecessor_sha256": sha(predecessor_strategy),
        "preparer_sha256": sha(Path(__file__)),
        "scope": "Keep all 52 full-size fixtures, original process caps, nine initial pairs and 27 pairs of every fixture in each affected family. Every non-timing receipt field remains an exact equality check. Correct the inherited prose: these cost probes emit certification counts, checksums and output fields, not allocation or consumed-work counters. No acceptance rule changes. Prior timing failures and the v1943 SIGXCPU remain failed and immutable.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared_audits": list(forks), "repeat_strategy_frozen": True,
                      "fixtures": 52, "family_repetitions": 27, "measurements_started": False}))


if __name__ == "__main__":
    main()
