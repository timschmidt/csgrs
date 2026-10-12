"""Freeze the original replay corpora under the reviewed computation evidence."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    output = E / "eager-bound-decoder-replay-protocol-v2280.json"
    assert not output.exists()
    previous_path = E / "bound-decoder-replay-protocol-v2182.json"
    previous = read(previous_path)
    qualification_path = E / "eager-bound-decoder-computation-review-v2279.json"
    qualification = read(qualification_path)
    assert qualification["qualified_for_fresh_replays"]
    assert not qualification["original_full_file_identity_rule_satisfied"]
    assert qualification["fresh_timing_processes"] == 0
    source_record = "eager-bound-decoder-baseline-v2257/candidate-source-hashes.json.gz"
    expected = qualification["source_record_sha256"]
    assert sha(E / source_record) == expected
    build = "typed-api-prototype-v2261-eager-bound-decoder-release-build"
    unit = "typed-api-prototype-v2268-eager-bound-decoder-scalar-tests"
    mapping = {previous["build_phase"]: build, previous["unit_phase"]: unit}
    caps = []
    for version, row in zip(range(2280, 2319), previous["caps"]):
        caps.append({**row, "phase": f"typed-api-prototype-v{version}-eager-bound-decoder-retained-cap"})
        assert sha(E / "run_limited_unit_test.py") == row["runner_sha256"]
        assert read(E / row["baseline_phase"] / "command.json")["command"][4:] == row["arguments"]
    for version, row in zip(range(2319, 2332), previous["numerical_replays"]):
        mapping[row["phase"]] = f"typed-api-prototype-v{version}-eager-bound-decoder-" + row["phase"].split("-bound-decoder-", 1)[1]
    numerical = []
    for row in previous["numerical_replays"]:
        new = {**row, "phase": mapping[row["phase"]], "command": [mapping.get(p, p) for p in row["command"]]}
        assert sha(Path(new["command"][1])) == row["runner_sha256"]
        assert all(sha(E / row["baseline_phase"] / name) == h for name, h in row["baseline_rust_inputs_sha256"].items())
        numerical.append(new)
    forks, pending = {}, {}

    def fork(old, new, replacements):
        path = E / old
        assert not (E / new).exists()
        original = path.read_text()
        assert all(old_text in original for old_text in replacements), old
        pattern = "|".join(re.escape(s) for s in sorted(replacements, key=len, reverse=True))
        text = re.sub(pattern, lambda match: replacements[match[0]], original)
        ast.parse(text)
        pending[new] = text
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(path),
            "fork_sha256": hashlib.sha256(text.encode()).hexdigest(), "literal_replacements": replacements}

    public, public_protocols = [], {}
    for version, row, runner in zip([2332, 2333], previous["public_replays"],
        ["reproduce_eager_bound_decoder_public_reuse.py", "reproduce_eager_bound_decoder_composed_reuse.py"]):
        assert sha(E / row["runner"]) == row["runner_sha256"]
        fork(row["runner"], runner, {"bound-decoder-binary-comparison-v2181/summary.json": "eager-bound-decoder-binary-comparison-v2262/summary.json"})
        old_protocol = read(E / row["protocol"])
        assert sha(E / row["protocol"]) == row["protocol_sha256"]
        protocol = copy.deepcopy(old_protocol)
        protocol.update(status="Frozen unchanged corpus replay for the eager-payload decoder",
            source_record_sha256=expected, runner_sha256=forks[runner]["fork_sha256"],
            predecessor_protocol=row["protocol"], predecessor_protocol_sha256=sha(E / row["protocol"]),
            candidate_proof_source=source_record)
        family = row["phase"].split("-bound-decoder-", 1)[1]
        name = f"eager-bound-decoder-{family}-protocol-v{version}.json"
        assert not (E / name).exists()
        encoded = json.dumps(protocol, indent=2) + "\n"
        public_protocols[name] = encoded
        phase = f"typed-api-prototype-v{version}-eager-bound-decoder-{family}"
        public.append({**row, "phase": phase, "protocol": name,
            "protocol_sha256": hashlib.sha256(encoded.encode()).hexdigest(),
            "command": ["/usr/bin/python3", str(E / runner), build, phase, unit, name],
            "runner": runner, "runner_sha256": forks[runner]["fork_sha256"],
            "predecessor_runner": row["runner"], "predecessor_runner_sha256": sha(E / row["runner"]),
            "runner_change": "Only the source-bound candidate library-digest record changes."})
        assert protocol["cases"] == old_protocol["cases"] and protocol["process_caps"] == old_protocol["process_caps"]
    parent = (E / "run_bound_decoder_replays.py").read_text()
    start = parent.index('    qualification = read(E / "bound-decoder-cost-qualification-v2181.json")\n')
    end = parent.index('    build, unit = ', start)
    old_gate = parent[start:end]
    new_gate = '''    qualification_path = E / "eager-bound-decoder-computation-review-v2279.json"
    assert sha(qualification_path) == protocol["cost_qualification_sha256"]
    qualification = read(qualification_path)
    assert qualification["source_record_sha256"] == expected
    assert qualification["qualified_for_fresh_replays"]
    assert qualification["method"] == "unchanged-computation-with-reviewed-diagnostic-metadata"
    assert not qualification["original_full_file_identity_rule_satisfied"]
    assert qualification["fresh_timing_processes"] == 0
    assert not qualification["all_historical_original_caps_pass"]
    assert all(sha(E / name) == value for name, value in qualification["static_inputs_sha256"].items())
    comparison = read(E / "eager-bound-decoder-binary-comparison-v2262/summary.json")
    assert comparison["source_record_sha256"] == expected and len(comparison["results"]) == 7
    for row in comparison["results"]:
        assert sha(Path(row["binary"])) == row["binary_sha256"]
    mutation = read(E / "eager-bound-decoder-remaining-guards-v2267.json")
    assert mutation["complete"] and mutation["source_record_sha256"] == expected
'''
    common = {previous_path.name: output.name,
        "bound-decoder-replay-validation-v2235.json": "eager-bound-decoder-replay-validation-v2333.json"}
    fork("run_bound_decoder_replays.py", "run_eager_bound_decoder_replays.py", {**common,
        "bound-decoder-candidate-validation-v2180.json": "eager-bound-decoder-candidate-validation-v2276.json",
        old_gate: new_gate})
    fork("compare_bound_decoder_queries.py", "compare_eager_bound_decoder_queries.py", {
        previous_path.name: output.name,
        "bound-decoder-domain-query-comparison-v2220.json": "eager-bound-decoder-domain-query-comparison-v2318.json"})
    fork("compare_bound_decoder_replays.py", "compare_eager_bound_decoder_replays.py", {**common,
        "bound-decoder-deterministic-replay-comparison-v2236.json": "eager-bound-decoder-deterministic-replay-comparison-v2334.json"})
    fork("audit_bound_decoder_fact_retention.py", "audit_eager_bound_decoder_fact_retention.py", {**common,
        "bound-decoder-public-fact-audit-v2236.json": "eager-bound-decoder-public-fact-audit-v2334.json"})
    assert len(caps) == 39 and len(numerical) == 13 and len(public) == 2
    assert sum(r["expected_cases"] for r in numerical) == 4129
    assert sum(r["expected_cases"] for r in public) == 648
    record = {"status": "Frozen before replays; broad runtime completion remains required",
        "source_record": source_record, "source_record_sha256": expected,
        "build_phase": build, "unit_phase": unit, "caps": caps,
        "numerical_replays": numerical, "public_replays": public,
        "requirements": previous["requirements"], "runner_forks": forks,
        "protocol_predecessor": previous_path.name, "protocol_predecessor_sha256": sha(previous_path),
        "cost_qualification": qualification_path.name, "cost_qualification_sha256": sha(qualification_path),
        "audit_scope": "Use the separately frozen and reviewed unchanged-computation method. Whole-file identity remains false; there is no fresh timing pass and both known retained-baseline cap failures remain failed. All 39 capped commands, 4129 numerical cases and 648 public schedules, caps, fixtures, assertions and fact/work comparisons are unchanged. Only candidate identities and library bindings differ. No overlapping workloads or source edits; runtime retention is a later decision."}
    for name, text in pending.items():
        (E / name).write_text(text)
        assert sha(E / name) == forks[name]["fork_sha256"]
    for name, text in public_protocols.items():
        (E / name).write_text(text)
    output.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"protocol": output.name, "capped_phases": 39,
        "numerical_cases": 4129, "public_cases": 648, "execution_started": False}))


if __name__ == "__main__":
    main()
