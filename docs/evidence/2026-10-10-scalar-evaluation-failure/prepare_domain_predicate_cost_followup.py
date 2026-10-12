"""Freeze the prescribed follow-up for the observed pi timing increase."""
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    original_path = E / "domain-predicate-initial-cost-review-v1780.json"
    original = read(original_path)
    assert original["paired_complete"] and original["successful_paired_processes"] == 936
    assert original["complete_fixtures"] == 52 and not original["partial_fixtures"]
    assert original["failed_attempt"] is None
    increases = original["above_five_percent_increases"]
    assert len(increases) == 1 and increases[0]["family"] == "inverse_trig"
    assert increases[0]["arguments"] == ["cost", "pi", "1024", "64"]
    protocol = E / "domain-predicate-cost-followup-protocol-v1780.json"
    runner = E / "replay_domain_predicate_followup.py"
    provenance = E / "domain-predicate-cost-followup-preparation-v1780.json"
    assert not any(path.exists() for path in [protocol, runner, provenance])
    config = {
        "output": "domain-predicate-focused-cost-v1780",
        "prelude": "domain-predicate-focused-cpu-prelude-v1780",
        "repetitions": 27, "cpu_seconds": [2, 3], "wall_seconds": 4,
        "selection": {"inverse_trig": [
            ["cost", "pi", "1024", "64"],
            ["cost", "atan", "1024", "64"],
            ["cost", "acos", "1024", "64"],
        ]},
        "scope": "Twenty-seven alternating-order pairs of the above-five-percent pi control and unchanged same-width atan/acos controls. Preserve all full-size inputs, query counts, binaries, checksums and allocation/work receipts; use the original 192 MiB, CPU 2/3-second and four-second wall caps and the same 50-million-step CPU prelude. Retain both the initial and follow-up observations. This proof-only change has no completeness gain that could justify a persistent runtime regression.",
    }
    protocol.write_text(json.dumps({
        "status": "Frozen after the complete original-cap comparison and before follow-up execution",
        "source_record_sha256": original["source_record_sha256"],
        "original_record": original_path.name, "original_record_sha256": sha(original_path),
        "original_run": original["paired_phase"], "original_run_complete": True,
        "initial_above_five_percent_increases": increases,
        "retained_runtime_checkpoint": original["retained_runtime_checkpoint"],
        "unchanged_inputs": "The original run and every observation remain immutable. Both binaries use identical inputs and limits; no reduced fixture or substituted baseline is used.",
        "jobs": [config],
    }, indent=2) + "\n")
    predecessor = E / "replay_domain_bound_followup.py"
    source = predecessor.read_text()
    prepared = read(E / "domain-predicate-cost-protocol-v1773.json")["runner_forks"]["replay_domain_predicate_cost.py"]
    replacements = {old: new for old, new in prepared["literal_replacements"].items() if old in source}
    before = "domain-bound-cost-followup-protocol-v1710.json"
    assert source.count(before) == 1
    replacements[before] = protocol.name
    pattern = "|".join(re.escape(value) for value in sorted(replacements, key=len, reverse=True))
    changed = re.sub(pattern, lambda match: replacements[match[0]], source)
    compile(changed, str(runner), "exec")
    runner.write_text(changed)
    provenance.write_text(json.dumps({
        "predecessor": predecessor.name, "predecessor_sha256": sha(predecessor),
        "runner": runner.name, "runner_sha256": sha(runner),
        "protocol": protocol.name, "protocol_sha256": sha(protocol),
        "literal_replacements": replacements,
        "scope": "Only frozen source, baseline/candidate phase and protocol references change in the already validated follow-up runner. Workloads, measurements, guards and comparison criteria remain unchanged.",
    }, indent=2) + "\n")
    print(json.dumps({"followup_jobs": 1, "paired_processes": 162,
                      "protocol_sha256": sha(protocol), "runner_sha256": sha(runner)}))


if __name__ == "__main__":
    main()
