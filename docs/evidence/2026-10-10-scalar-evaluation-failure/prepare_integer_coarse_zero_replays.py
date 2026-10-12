"""Freeze the retained replay corpora and fork their identity-bound audit runners."""
import copy
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "integer-coarse-zero-replay-protocol-v2101.json"
    assert not out.exists()
    proof = read(E / "integer-coarse-zero-proof-validation-v2083.json")
    source_record, source_hash = proof["source_record"], proof["source_record_sha256"]
    assert sha(E / source_record) == source_hash
    prior_path = E / "domain-predicate-replay-protocol-v1781.json"
    prior = read(prior_path)
    checkpoint = read(E / proof["retained_runtime_checkpoint"])
    assert sha(prior_path) == checkpoint["artifacts_sha256"][prior_path.name]
    completed = read(E / "domain-predicate-replay-validation-v1834.json")
    assert completed["complete"] and len(completed["results"]) == 54
    build = "typed-api-prototype-v2086-integer-coarse-zero-release-build"
    unit = "typed-api-prototype-v2084-integer-coarse-zero-scalar-tests"
    mapping = {prior["build_phase"]: build, prior["unit_phase"]: unit}
    caps = []
    for version, row in zip(range(2101, 2140), prior["caps"]):
        command = read(E / row["phase"] / "command.json")["command"]
        assert command[4:] == row["arguments"] and sha(Path(command[1])) == row["runner_sha256"]
        caps.append({"phase": f"typed-api-prototype-v{version}-integer-coarse-zero-retained-cap",
            "baseline_phase": row["phase"], "arguments": row["arguments"],
            "runner_sha256": row["runner_sha256"]})
    assert len(caps) == 39
    for version, row in zip(range(2140, 2153), prior["numerical_replays"]):
        mapping[row["phase"]] = f"typed-api-prototype-v{version}-integer-coarse-zero-" + row["phase"].split("-domain-predicate-", 1)[1]
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
    for version, row, runner in zip([2153, 2154], prior["public_replays"],
        ["reproduce_integer_zero_public_reuse.py", "reproduce_integer_zero_composed_reuse.py"]):
        previous_protocol = read(E / row["protocol"])
        assert sha(E / row["protocol"]) == row["protocol_sha256"]
        assert sha(E / row["runner"]) == row["runner_sha256"]
        before = "typed-api-prototype-v1773-domain-predicate-atan-cost/summary.json"
        after = "typed-api-prototype-v2093-integer-coarse-zero-atan-cost/summary.json"
        assert (E / row["runner"]).read_text().count(before) == 1
        fork(row["runner"], runner, {before: after})
        protocol = copy.deepcopy(previous_protocol)
        protocol.update(status="Frozen unchanged corpus replay for the combined integer shortcut, root kernel and outlined-cache proposal",
            source_record_sha256=source_hash, runner_sha256=forks[runner]["fork_sha256"],
            predecessor_protocol=row["protocol"], predecessor_protocol_sha256=sha(E / row["protocol"]),
            candidate_proof_source=source_record)
        family = row["phase"].split("-domain-predicate-", 1)[1]
        name = f"integer-coarse-zero-{family}-protocol-v{version}.json"
        assert not (E / name).exists()
        encoded = json.dumps(protocol, indent=2) + "\n"
        public_protocols[name] = encoded
        phase = f"typed-api-prototype-v{version}-integer-coarse-zero-{family}"
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
        "domain-predicate-replay-validation-v1834.json": "integer-coarse-zero-replay-validation-v2154.json"}
    fork("run_domain_predicate_replays.py", "run_integer_zero_replays.py", {
        **common,
        "domain-predicate-candidate-validation-v1772.json": "integer-coarse-zero-candidate-validation-v2092.json",
        "domain-predicate-cost-orchestration-v1779.json": "integer-coarse-zero-cost-orchestration-v2099.json",
        '    assert costs["completed"] and costs["source_record_sha256"] == expected\n': '    assert costs["completed"] and costs["source_record_sha256"] == expected\n    cost_review = read(E / "integer-coarse-zero-cost-review-v2100.json")\n    controller = read(E / "integer-coarse-zero-cost-validation-v2100.json")\n    assert controller["complete"] and cost_review["local_timing_and_process_gates_passed"]\n    assert cost_review["source_record_sha256"] == controller["source_record_sha256"] == expected\n',
    })
    fork("compare_domain_predicate_queries.py", "compare_integer_zero_queries.py", {
        "domain-predicate-replay-protocol-v1781.json": out.name,
        "domain-predicate-domain-query-comparison-v1819.json": "integer-coarse-zero-domain-query-comparison-v2139.json",
    })
    fork("compare_domain_predicate_replays.py", "compare_integer_zero_replays.py", {
        **common,
        "domain-predicate-deterministic-replay-comparison-v1835.json": "integer-coarse-zero-deterministic-replay-comparison-v2155.json",
        "typed-api-prototype-v1755-public-proof-reuse": "typed-api-prototype-v1833-domain-predicate-public-proof-reuse",
        "typed-api-prototype-v1757-composed-proof-reuse": "typed-api-prototype-v1834-domain-predicate-composed-proof-reuse",
    })
    fork("audit_domain_predicate_fact_retention.py", "audit_integer_zero_fact_retention.py", {
        **common,
        "domain-predicate-public-fact-audit-v1835.json": "integer-coarse-zero-public-fact-audit-v2155.json",
    })
    record = {
        "status": "Frozen before capped, numerical and public replays; broad runtime validation is in progress",
        "source_record": source_record, "source_record_sha256": source_hash,
        "build_phase": build, "unit_phase": unit,
        "caps": caps, "numerical_replays": numerical, "public_replays": public,
        "requirements": prior["requirements"], "runner_forks": forks,
        "protocol_predecessor": prior_path.name, "protocol_predecessor_sha256": sha(prior_path),
        "audit_scope": "Candidate identities change, and replay startup additionally requires the final cost audit and controller to pass. The public/composed fact-retention audit loops, numerical oracles, work comparisons and full-size cases are unchanged.",
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
