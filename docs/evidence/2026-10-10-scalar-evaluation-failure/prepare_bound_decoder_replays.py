"""Freeze unchanged decoder replays, with an explicit executable-identity cost gate."""
import copy
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "bound-decoder-replay-protocol-v2182.json"
    assert not out.exists()
    proof = read(E / "bound-decoder-proof-validation-v2171.json")
    source_record, source_hash = proof["source_record"], proof["source_record_sha256"]
    assert sha(E / source_record) == source_hash
    broad = read(E / "bound-decoder-candidate-validation-v2180.json")
    assert broad["complete"] and broad["source_record_sha256"] == source_hash
    prior_path = E / "domain-predicate-replay-protocol-v1781.json"
    prior = read(prior_path)
    checkpoint = read(E / proof["retained_runtime_checkpoint"])
    assert sha(prior_path) == checkpoint["artifacts_sha256"][prior_path.name]
    completed = read(E / "domain-predicate-replay-validation-v1834.json")
    assert completed["complete"] and len(completed["results"]) == 54
    build = "typed-api-prototype-v2174-bound-decoder-release-build"
    unit = "typed-api-prototype-v2172-bound-decoder-scalar-tests"
    mapping = {prior["build_phase"]: build, prior["unit_phase"]: unit}
    caps = []
    for version, row in zip(range(2182, 2221), prior["caps"]):
        command = read(E / row["phase"] / "command.json")["command"]
        assert command[4:] == row["arguments"] and sha(Path(command[1])) == row["runner_sha256"]
        caps.append({"phase": f"typed-api-prototype-v{version}-bound-decoder-retained-cap",
            "baseline_phase": row["phase"], "arguments": row["arguments"],
            "runner_sha256": row["runner_sha256"]})
    assert len(caps) == 39
    for version, row in zip(range(2221, 2234), prior["numerical_replays"]):
        mapping[row["phase"]] = f"typed-api-prototype-v{version}-bound-decoder-" + row["phase"].split("-domain-predicate-", 1)[1]
    numerical = []
    for row in prior["numerical_replays"]:
        command = [mapping.get(part, part) for part in row["command"]]
        inputs = {path.name: sha(path) for path in sorted((E / row["phase"]).glob("*.rs"))}
        assert inputs == row["baseline_rust_inputs_sha256"]
        assert sha(Path(command[1])) == row["runner_sha256"]
        numerical.append({"phase": mapping[row["phase"]], "baseline_phase": row["phase"],
            "command": command, "runner_sha256": row["runner_sha256"],
            "expected_cases": row["expected_cases"], "baseline_rust_inputs_sha256": inputs})
    assert len(numerical) == 13 and sum(row["expected_cases"] for row in numerical) == 4129
    forks, pending = {}, {}

    def fork(old, new, replacements):
        original = (E / old).read_text()
        assert not (E / new).exists()
        assert all(before in original for before in replacements), old
        pattern = "|".join(re.escape(value) for value in sorted(replacements, key=len, reverse=True))
        updated = re.sub(pattern, lambda match: replacements[match[0]], original)
        compile(updated, str(E / new), "exec")
        pending[new] = updated
        forks[new] = {"predecessor": old, "predecessor_sha256": sha(E / old),
            "fork_sha256": hashlib.sha256(updated.encode()).hexdigest(), "literal_replacements": replacements}

    public, public_protocols = [], {}
    before = 'prior_libraries = read(E / "typed-api-prototype-v1773-domain-predicate-atan-cost/summary.json")["libraries_sha256"]'
    after = 'prior_libraries = {name: row["candidate_sha256"] for name, row in read(E / "bound-decoder-binary-comparison-v2181/summary.json")["libraries"].items()}'
    for version, row, runner in zip([2234, 2235], prior["public_replays"],
        ["reproduce_bound_decoder_public_reuse.py", "reproduce_bound_decoder_composed_reuse.py"]):
        previous_protocol = read(E / row["protocol"])
        assert sha(E / row["protocol"]) == row["protocol_sha256"]
        assert sha(E / row["runner"]) == row["runner_sha256"]
        assert (E / row["runner"]).read_text().count(before) == 1
        fork(row["runner"], runner, {before: after})
        protocol = copy.deepcopy(previous_protocol)
        protocol.update(status="Frozen unchanged corpus replay for the proved bound decoder",
            source_record_sha256=source_hash, runner_sha256=forks[runner]["fork_sha256"],
            predecessor_protocol=row["protocol"], predecessor_protocol_sha256=sha(E / row["protocol"]),
            candidate_proof_source=source_record)
        family = row["phase"].split("-domain-predicate-", 1)[1]
        name = f"bound-decoder-{family}-protocol-v{version}.json"
        assert not (E / name).exists()
        encoded = json.dumps(protocol, indent=2) + "\n"
        public_protocols[name] = encoded
        phase = f"typed-api-prototype-v{version}-bound-decoder-{family}"
        public.append({"phase": phase, "baseline_phase": row["phase"], "protocol": name,
            "protocol_sha256": hashlib.sha256(encoded.encode()).hexdigest(),
            "command": ["/usr/bin/python3", str(E / runner), build, phase, unit, name],
            "expected_cases": len(protocol["cases"]), "runner": runner,
            "runner_sha256": forks[runner]["fork_sha256"], "predecessor_runner": row["runner"],
            "predecessor_runner_sha256": sha(E / row["runner"]),
            "runner_change": {"before": before, "after": after}, "probe_sha256": protocol["probe_sha256"]})
        assert protocol["cases"] == previous_protocol["cases"] and protocol["process_caps"] == previous_protocol["process_caps"]
    assert sum(row["expected_cases"] for row in public) == 648
    common = {"domain-predicate-replay-protocol-v1781.json": out.name,
        "domain-predicate-replay-validation-v1834.json": "bound-decoder-replay-validation-v2235.json"}
    old_gate = ('    costs = read(E / "domain-predicate-cost-orchestration-v1779.json")\n'
                '    assert costs["completed"] and costs["source_record_sha256"] == expected\n')
    new_gate = ('    qualification = read(E / "bound-decoder-cost-qualification-v2181.json")\n'
                '    assert qualification["source_record_sha256"] == expected\n'
                '    assert qualification["qualified_for_unchanged_probe_fixtures"]\n'
                '    assert qualification["method"] == "complete-executable-identity"\n'
                '    comparison_path = E / qualification["binary_comparison"]\n'
                '    assert sha(comparison_path) == qualification["binary_comparison_sha256"]\n'
                '    comparison = read(comparison_path)\n'
                '    assert comparison["all_probe_binaries_identical"] and comparison["source_record_sha256"] == expected\n'
                '    assert len(comparison["results"]) == 7\n'
                '    for row in comparison["results"]:\n'
                '        assert sha(Path(row["binary"])) == row["binary_sha256"] == row["baseline_binary_sha256"]\n')
    fork("run_domain_predicate_replays.py", "run_bound_decoder_replays.py", {
        **common,
        "domain-predicate-candidate-validation-v1772.json": "bound-decoder-candidate-validation-v2180.json",
        old_gate: new_gate,
    })
    fork("compare_domain_predicate_queries.py", "compare_bound_decoder_queries.py", {
        "domain-predicate-replay-protocol-v1781.json": out.name,
        "domain-predicate-domain-query-comparison-v1819.json": "bound-decoder-domain-query-comparison-v2220.json",
    })
    fork("compare_domain_predicate_replays.py", "compare_bound_decoder_replays.py", {
        **common,
        "domain-predicate-deterministic-replay-comparison-v1835.json": "bound-decoder-deterministic-replay-comparison-v2236.json",
        "typed-api-prototype-v1755-public-proof-reuse": "typed-api-prototype-v1833-domain-predicate-public-proof-reuse",
        "typed-api-prototype-v1757-composed-proof-reuse": "typed-api-prototype-v1834-domain-predicate-composed-proof-reuse",
    })
    fork("audit_domain_predicate_fact_retention.py", "audit_bound_decoder_fact_retention.py", {
        **common,
        "domain-predicate-public-fact-audit-v1835.json": "bound-decoder-public-fact-audit-v2236.json",
    })
    record = {
        "status": "Frozen before capped, numerical and public replays; executable-identity qualification remains a separate prerequisite",
        "source_record": source_record, "source_record_sha256": source_hash,
        "build_phase": build, "unit_phase": unit,
        "caps": caps, "numerical_replays": numerical, "public_replays": public,
        "requirements": prior["requirements"], "runner_forks": forks,
        "protocol_predecessor": prior_path.name, "protocol_predecessor_sha256": sha(prior_path),
        "cost_qualification": "bound-decoder-cost-qualification-v2181.json",
        "audit_scope": "The candidate identities and library binding change; every corpus, cap, oracle and fact/work comparison remains unchanged. This runner requires the separately audited full-executable identity of all seven cost probes. It makes no fresh timing claim. If identity fails, this runner cannot start; fork a measured-cost gate after completing the unchanged original-cap controls. Historical resource failures stay failed.",
    }
    for name, text in pending.items():
        (E / name).write_text(text)
        assert sha(E / name) == forks[name]["fork_sha256"]
    for name, text in public_protocols.items():
        (E / name).write_text(text)
    out.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"capped_phases": len(caps), "numerical_cases": 4129,
        "public_reuse_cases": 648, "forked_runners": list(forks)}))


if __name__ == "__main__":
    main()
