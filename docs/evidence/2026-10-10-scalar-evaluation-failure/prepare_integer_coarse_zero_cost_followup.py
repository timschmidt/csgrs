"""Apply the predeclared family-repeat rule to the completed original-cap timings."""
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    original_path = E / "integer-coarse-zero-initial-cost-review-v2100.json"
    original = read(original_path)
    assert original["paired_complete"] and original["successful_paired_processes"] == 936
    assert original["complete_fixtures"] == 52 and not original["partial_fixtures"]
    assert original["failed_attempt"] is None
    strategy_path = E / "integer-coarse-zero-cost-followup-strategy-v2100.json"
    strategy = read(strategy_path)
    assert original["protocol_sha256"] == strategy["cost_protocol_sha256"]
    assert original["source_record_sha256"] == strategy["source_record_sha256"]
    increases = original["above_five_percent_increases"]
    assert increases == [row for row in original["cases"] if row["median_percent_change"] > 5]
    affected = {row["family"] for row in increases}
    selection = {family: fixtures for family, fixtures in strategy["family_fixtures"].items() if family in affected}
    assert set(selection) == affected
    protocol = E / "integer-coarse-zero-cost-followup-protocol-v2100.json"
    runner = E / "replay_integer_coarse_zero_followup.py"
    provenance = E / "integer-coarse-zero-cost-followup-preparation-v2100.json"
    assert not any(path.exists() for path in [protocol, runner, provenance])
    config = {
        "output": "integer-coarse-zero-focused-cost-v2100",
        "prelude": "integer-coarse-zero-focused-cpu-prelude-v2100",
        "repetitions": strategy["repetitions"], "cpu_seconds": strategy["cpu_seconds"],
        "wall_seconds": strategy["wall_seconds"], "selection": selection,
        "scope": "Twenty-seven alternating-order pairs for every unchanged original fixture in each family with an initial increase above five percent. The family selection follows the strategy frozen before any candidate timings. Preserve all full-size inputs, binaries, checksums and all non-timing output fields, original 192 MiB/CPU 2/3-second/four-second wall limits and the identical 50-million-step CPU prelude. Keep initial and follow-up observations without causal attribution or a uniform-speedup claim.",
    }
    data = {
        "status": "Frozen after the complete original-cap comparison and before follow-up execution",
        "source_record_sha256": original["source_record_sha256"],
        "original_record": original_path.name, "original_record_sha256": sha(original_path),
        "original_run": original["paired_phase"], "original_run_complete": True,
        "initial_above_five_percent_increases": increases,
        "retained_runtime_checkpoint": original["retained_runtime_checkpoint"],
        "followup_strategy": strategy_path.name, "followup_strategy_sha256": sha(strategy_path),
        "unchanged_inputs": "The complete original run remains immutable. Every fixture in each affected family is retained as an unchanged control; none is selected according to a favorable timing outcome.",
        "jobs": [config] if selection else [],
    }
    predecessor = E / "replay_domain_predicate_followup.py"
    source = predecessor.read_text()
    prepared = read(E / "integer-coarse-zero-cost-protocol-v2093.json")["runner_forks"]["replay_integer_coarse_zero_cost.py"]
    replacements = {old: new for old, new in prepared["literal_replacements"].items() if old in source}
    before = "domain-predicate-cost-followup-protocol-v1780.json"
    assert source.count(before) == 1
    replacements[before] = protocol.name
    pattern = "|".join(re.escape(value) for value in sorted(replacements, key=len, reverse=True))
    updated = re.sub(pattern, lambda match: replacements[match[0]], source)
    compile(updated, str(runner), "exec")
    protocol.write_text(json.dumps(data, indent=2) + "\n")
    runner.write_text(updated)
    provenance.write_text(json.dumps({
        "predecessor": predecessor.name, "predecessor_sha256": sha(predecessor),
        "runner": runner.name, "runner_sha256": sha(runner),
        "protocol": protocol.name, "protocol_sha256": sha(protocol),
        "literal_replacements": replacements,
        "scope": "Only source, baseline/candidate and protocol identities change in the already validated runner; selection and original limits follow the frozen strategy.",
    }, indent=2) + "\n")
    print(json.dumps({"followup_jobs": len(data["jobs"]), "affected_families": sorted(affected),
        "selected_fixtures": sum(map(len, selection.values())),
        "paired_processes": 2 * strategy["repetitions"] * sum(map(len, selection.values())),
        "protocol_sha256": sha(protocol), "runner_sha256": sha(runner)}))


if __name__ == "__main__":
    main()
