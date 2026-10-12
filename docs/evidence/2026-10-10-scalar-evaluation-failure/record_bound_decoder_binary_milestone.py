"""Record unsuccessful binary-identity qualification and the frozen cost fallback."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    output = E / "bound-decoder-binary-milestone-v2247.json"
    audit = E / "bound-decoder-binary-ledger-v2247.json"
    assert not output.exists() and not audit.exists()
    previous = read(E / "bound-decoder-broad-milestone-v2180.json")
    source_record = E / previous["source_record"]
    expected = previous["source_record_sha256"]
    assert sha(source_record) == expected
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    assert all(sha(SOURCE / name) == digest for name, digest in sources.items())
    main = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in main.items())
    inputs = ["bound-decoder-binary-comparison-v2181/summary.json", "bound-decoder-cost-qualification-v2181.json",
              "bound-decoder-cost-protocol-preparation-v2247.json", "bound-decoder-measured-replay-protocol-v2247.json"]
    comparison, qualification, preparation, replays = [read(E / name) for name in inputs]
    assert all(item["source_record_sha256"] == expected for item in [comparison, qualification, preparation, replays])
    assert comparison["builds_passed"] == 7 and comparison["new_timing_processes"] == 0
    assert not comparison["all_probe_binaries_identical"]
    assert len(comparison["changed_probe_families"]) == 7
    for row in comparison["results"]:
        assert sha(Path(row["binary"])) == row["binary_sha256"]
    assert not qualification["qualified_for_unchanged_probe_fixtures"]
    assert not (E / "bound-decoder-cost-validation-v2247.json").exists()
    assert len(replays["caps"]) == 39
    assert sum(row["expected_cases"] for row in replays["numerical_replays"]) == 4129
    assert sum(row["expected_cases"] for row in replays["public_replays"]) == 648
    next_priority = (
        "Run the frozen v2240-v2247 original-cap decoder cost comparison and all prescribed "
        "family-wide repeats; complete the unchanged capped/numerical/public replays and fact "
        "audits only if those costs pass. All seven compiled probes differ, so the identity "
        "reuse route remains unqualified. Preserve all earlier failures, the fixed pre-Verus "
        "baseline and open producer/existence/atomic and main/native/full-board requirements."
    )
    record = {"status": "All seven probe binaries differ; fresh cost fallback frozen before measurements",
        "source_record": previous["source_record"], "source_record_sha256": expected,
        "inputs_sha256": {name: sha(E / name) for name in inputs},
        "identical_binaries": 0, "changed_families": comparison["changed_probe_families"],
        "new_timing_processes": 0, "reused_cost_receipts": 0,
        "retained_runtime_checkpoint": previous["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": previous["retained_runtime_checkpoint_sha256"],
        "runtime_retained": False, "main_runtime_admitted": False,
        "source_files_verified": len(sources), "main_files_verified": len(main),
        "next_priority": next_priority,
        "limits": "Different executable bytes establish neither a timing regression nor its cause. The conditional decoder proof and nine broad passes stand, but runtime retention requires the frozen original-cap costs and subsequent replays. No property/target or whole-crate obligation is closed; the previous failed experiments and resource limits remain explicit."}
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before, after = json.loads(before_bytes), json.loads(before_bytes)
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in [inputs[0], inputs[1], output.name]:
        p4["evidence"].append(str(E.relative_to(ROOT) / name))
    update = (
        "The v2181 compiled comparison passes all seven builds but finds no byte-identical "
        "probe. No cost receipts are reused and no fresh timing result is inferred. The "
        "v2240-v2247 fallback freezes all 52 original fixtures, 468 standalone processes, "
        "nine initial pairs and 27-pair repeats for every fixture in each initially "
        "regressing family, with unchanged caps. The separate measured-cost replay plan "
        "retains all 39 caps, 4129 numerical cases and 648 public schedules. All execution "
        "and retention gates remain pending; v1836 is still retained."
    )
    p4["remaining"] += " " + update
    assert after["next_actions"][0].count(previous["next_priority"]) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(previous["next_priority"], next_priority, 1)
    after["next_actions"][0] += " " + update
    same = copy.deepcopy(after)
    old = next(row for row in before["properties"] if row["id"] == "P4")
    compare = next(row for row in same["properties"] if row["id"] == "P4")
    compare["evidence"], compare["remaining"] = old["evidence"], old["remaining"]
    same["next_actions"][0] = before["next_actions"][0]
    assert same == before
    output.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    audit.write_text(json.dumps({"before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": sha(ledger), "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "milestone_record_sha256": sha(output), "runtime_retention_pending": True}, indent=2) + "\n")
    print(json.dumps({"identical_binaries": 0, "cost_receipts_reused": 0,
                      "fresh_cost_protocol_frozen": True, "runtime_retention_pending": True}))


if __name__ == "__main__":
    main()
