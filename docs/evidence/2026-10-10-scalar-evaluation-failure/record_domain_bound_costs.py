"""Retain the original failure and the separately predeclared timing follow-ups."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
source = read(E / "domain-bound-candidate-caps-v1693.json")["source_record_sha256"]
checkpoint = read(E / "typed-api-prototype-v1672-state.json")["artifacts_sha256"]
prior_path = E / "domain-sign-cost-review-v1639.json"
assert sha(prior_path) == checkpoint[prior_path.name]
prior = read(prior_path)
standalone = read(E / "domain-bound-cost-orchestration-v1709.json")
assert standalone["completed"] and standalone["source_record_sha256"] == source
families = {}
total = 0


def nontiming(row):
    return {k: v for k, v in row.items() if k != "elapsed_ns"}


for row in standalone["results"]:
    family = row["family"]
    before = E / prior["families"][family]["phases"]["candidate"]
    after = E / row["phase"]
    assert sha(before / "summary.json") == checkpoint[before.name + "/summary.json"]
    assert all(sha(before / f) == sha(after / f) for f in ["probe.rs", "reproduce.py"])
    commands = lambda p: [r for r in read(p / "commands.json") if r["name"] not in ["format", "build"]]
    a, b = commands(before), commands(after)
    assert len(a) == len(b) == row["processes"]
    total += len(b)
    expected = {}
    for x, y in zip(a, b):
        assert x["name"] == y["name"] and x["command"][1:] == y["command"][1:]
        for command in [x, y]:
            assert command["address_space_cap_mib"] == 192
            assert command["cpu_seconds"] == [2, 3] and command["timeout_seconds"] == 4
        va, vb = read(before / (x["name"] + ".stdout")), read(after / (y["name"] + ".stdout"))
        assert nontiming(va) == nontiming(vb)
        key = tuple(x["command"][1:])
        assert key not in expected or expected[key] == nontiming(va)
        expected[key] = nontiming(va)
    binaries = {"baseline": Path(a[0]["command"][0]), "candidate": Path(b[0]["command"][0])}
    hashes = {k: sha(p) for k, p in binaries.items()}
    assert hashes["baseline"] == prior["families"][family]["binary_sha256"]["candidate"]
    families[family] = {
        "phases": {"baseline": before.name, "candidate": after.name}, "processes": len(b),
        "non_timing_outputs_identical": True, "binary_sha256": hashes,
        "binary_bytes": {k: p.stat().st_size for k, p in binaries.items()},
        "_expected": expected,
    }
assert total == 468


def receipts(path, count, cpu, wall):
    rows = read(path / "receipts.json")
    assert len(rows) == count
    for row in rows:
        family, index, which = row["name"].rsplit("-", 2)
        assert row["exit_code"] == 0
        assert row["binary_sha256"] == families[family]["binary_sha256"][which]
        assert row["address_space_bytes"] == 192 * 1024 * 1024
        assert row["cpu_seconds_soft_hard"] == cpu and row["wall_seconds_limit"] == wall
        assert nontiming(row["receipt"]) == families[family]["_expected"][tuple(row["command"][1:])]
    preparation = read(path / "prelude-receipts.json")
    assert all(r["exit_code"] == 0 and r["stderr"] == "" for r in preparation)
    assert len({r["stdout"] for r in preparation}) == 1
    return rows, preparation


partial = E / "domain-bound-preconditioned-cost-v1710"
assert not (partial / "summary.json").exists()
partial_rows, partial_preludes = receipts(partial, 864, [2, 3], 4)
assert len(partial_preludes) == 865
failed = read(partial / "failed-attempt.json")
assert failed["name"] == "algebraic-18-baseline" and failed["exit_code"] == -24
assert not failed["wall_timeout"]
assert failed["cpu_seconds_soft_hard"] == [2, 3] and failed["wall_seconds_limit"] == 4
assert failed["binary_sha256"] == families["algebraic"]["binary_sha256"]["baseline"]
partial_record = read(E / "domain-bound-partial-cost-v1710.json")
assert partial_record["complete_fixtures"] == 48
assert all(sha(partial / n) == v for n, v in partial_record["original_run_artifacts_sha256"].items())

followups = {}
protocol_path = E / "domain-bound-cost-followup-protocol-v1710.json"
for config, count in zip(read(protocol_path)["jobs"], [162, 108]):
    path = E / config["output"]
    data = read(path / "summary.json")
    assert data["source_record_sha256"] == source and data["guards_unchanged"]
    assert data["configuration"] == config and data["protocol_sha256"] == sha(protocol_path)
    assert data["processes"] == count and data["cpu_preparation"]["prelude_processes"] == count
    assert all(sha(path / n) == v for n, v in data["artifacts_sha256"].items())
    for family, provenance in data["provenance"].items():
        assert provenance["phases"] == families[family]["phases"]
        assert provenance["binary_sha256_at_paired_run"] == families[family]["binary_sha256"]
        assert all(sha(Path(p)) == provenance["binary_sha256_at_paired_run"][k]
                   for k, p in provenance["binaries"].items())
    rows, preludes = receipts(path, count, config["cpu_seconds"], config["wall_seconds"])
    assert len(preludes) == count
    followups[path.name] = data

focused = followups["domain-bound-focused-cost-v1710"]
algebraic = followups["domain-bound-algebraic-cost-v1710"]
acos = next(r for r in focused["cases"]["inverse_trig"] if r["arguments"][1] == "acos")
assert 0 < acos["median_percent_change"] < 5
initial = next(r for r in partial_record["cases"]
               if r["family"] == "inverse_trig" and r["arguments"][1] == "acos")
assert initial["median_percent_change"] > 5
for family in families.values():
    del family["_expected"]
out = E / "domain-bound-cost-review-v1710.json"
assert not out.exists()
record = {
    "status": "Scoped cost evidence for an isolated completeness extension; the original fixed-CPU run remains incomplete",
    "source_record_sha256": source, "baseline_checkpoint": "typed-api-prototype-v1672-state.json",
    "original_candidate_processes": total, "families": families,
    "original_fixed_cpu_run": {"phase": partial.name, "completed_processes": 864,
        "prelude_processes": 865, "failed_attempt": failed, "complete": False,
        "case_statistics": partial_record["cases"]},
    "focused_original_cap_followup": {"phase": "domain-bound-focused-cost-v1710", "processes": 162,
        "cases": focused["cases"], "initial_above_five_percent_case": initial},
    "separate_larger_cap_diagnostic": {"phase": "domain-bound-algebraic-cost-v1710", "processes": 108,
        "cases": algebraic["cases"], "cpu_seconds": [4, 5], "wall_seconds": 8,
        "passes_original_two_second_guard": False},
    "initial_and_followup_acos_percent": [initial["median_percent_change"], acos["median_percent_change"]],
    "all_successful_cost_processes_including_standalone": 1602,
    "separate_cpu_preparation_processes": 1135,
    "interpretation": [
        "The original complete trials contain one increase above five percent: the inverse-trig acos cancellation control. Its 8.37 percent increase becomes 2.45 percent in the predeclared 27-pair repeat. Both observations remain retained. No causal attribution to code placement or uniform speedup is established.",
        "The stopped original-cap process is the retained v1672 baseline. Its final receipt was absent, so the interrupted stage is unknown. The separate equal larger-cap six-fixture diagnostic does not pass or replace that failure.",
        "All successful cost processes preserve the exact non-timing outputs of the frozen probes. Timing and executable size remain local controls, not whole-board admission evidence.",
        "The unchanged v1672 baseline already retains a measured narrow integer-control slowdown versus its older baseline. Current relative comparisons do not erase that earlier tradeoff.",
        "This extension is retained for stronger proof reuse and 38 additional valid public answers at unchanged allowances. Public-query work and allocations can increase when additional evaluation becomes reachable; no universal allocation reduction is claimed.",
        "EvaluationBudget does not bound bytes, time or unmetered algebraic helpers. Original process-cap failures remain separate empirical limitations.",
    ],
}
out.write_text(json.dumps(record, indent=2) + "\n")
print(json.dumps({"record": out.name, "successful_cost_processes": 1602,
                  "separate_cpu_preparations": 1135,
                  "acos_original_followup_percent": record["initial_and_followup_acos_percent"]}))
