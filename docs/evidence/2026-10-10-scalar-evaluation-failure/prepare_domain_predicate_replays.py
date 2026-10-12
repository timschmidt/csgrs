"""Freeze unchanged capped, numerical and public reuse corpora for the extraction."""
import copy
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "domain-predicate-replay-protocol-v1781.json"
    assert not out.exists()
    source_record = "typed-api-prototype-v1761-domain-predicate-verus/source-hashes.json.gz"
    source_hash = sha(E / source_record)
    build = "typed-api-prototype-v1766-domain-predicate-release-build"
    unit = "typed-api-prototype-v1764-domain-predicate-scalar-tests"
    caps = read(E / "domain-bound-candidate-caps-v1693.json")["tests"]
    remaining = read(E / "domain-bound-remaining-validation-v1753.json")["results"]
    caps += [row for row in remaining if "cap" in row]
    assert len(caps) == 39
    cap_cases = []
    for version, row in zip(range(1781, 1820), caps):
        original = read(E / row["phase"] / "command.json")["command"]
        assert Path(original[1]).name == "run_limited_unit_test.py"
        assert original[4].startswith("computable::node::")
        cap_cases.append({"phase": f"typed-api-prototype-v{version}-domain-predicate-retained-cap",
                          "baseline_phase": row["phase"], "arguments": original[4:],
                          "runner_sha256": sha(Path(original[1]))})
    replays = read(E / "domain-bound-replay-orchestration-v1745.json")["results"]
    replays += [row for row in remaining if "expected_cases" in row]
    assert len(replays) == 13 and sum(row["expected_cases"] for row in replays) == 4129
    replacements = {"typed-api-prototype-v1696-domain-bound-release-build": build,
                    "typed-api-prototype-v1694-domain-bound-scalar-tests": unit}
    for version, row in zip(range(1820, 1833), replays):
        replacements[row["phase"]] = f"typed-api-prototype-v{version}-domain-predicate-" + row["phase"].split("-domain-bound-", 1)[1]
    replay_cases = []
    for row in replays:
        command = [replacements.get(part, part) for part in row["command"]]
        replay_cases.append({"phase": replacements[row["phase"]], "baseline_phase": row["phase"],
                             "command": command, "runner_sha256": sha(Path(command[1])),
                             "expected_cases": row["expected_cases"],
                             "baseline_probe_sha256": sha(E / row["phase"] / "probe.rs")})
    public = []
    for family, version, prior_protocol, original_runner, new_runner in [
        ("public-proof-reuse", 1833, "public-proof-reuse-protocol-v1755.json",
         "reproduce_public_proof_reuse.py", "reproduce_public_proof_reuse_v2.py"),
        ("composed-proof-reuse", 1834, "composed-proof-reuse-protocol-v1757.json",
         "reproduce_composed_proof_reuse.py", "reproduce_composed_proof_reuse_v2.py"),
    ]:
        predecessor = (E / original_runner).read_text()
        before = "typed-api-prototype-v1703-domain-bound-atan-cost/summary.json"
        after = "typed-api-prototype-v1773-domain-predicate-atan-cost/summary.json"
        assert predecessor.count(before) == 1 and not (E / new_runner).exists()
        candidate = predecessor.replace(before, after)
        compile(candidate, str(E / new_runner), "exec")
        (E / new_runner).write_text(candidate)
        prior = read(E / prior_protocol)
        protocol = copy.deepcopy(prior)
        protocol.update(status="Frozen unchanged corpus replay for the conditional domain-predicate extraction",
                        source_record_sha256=source_hash, runner_sha256=sha(E / new_runner),
                        predecessor_protocol=prior_protocol, predecessor_protocol_sha256=sha(E / prior_protocol),
                        candidate_proof_source=source_record)
        if family == "public-proof-reuse":
            old = "This adds public coverage to an unchanged retained runtime. Existing private deterministic publication tests and the original capped cost failures remain separate evidence."
            assert old in protocol["limits"]
            protocol["limits"][protocol["limits"].index(old)] = (
                "This replays the unchanged public corpus against an unretained runtime extraction. "
                "Existing private deterministic publication tests and the original capped cost failures remain separate evidence.")
        else:
            old = "No test or runtime source in the retained candidate is changed."
            assert protocol["scope"].count(old) == 1
            protocol["scope"] = protocol["scope"].replace(old,
                "The unchanged corpus is replayed against the unretained runtime extraction.")
        name = f"domain-predicate-{family}-protocol-v{version}.json"
        assert not (E / name).exists()
        assert protocol["cases"] == prior["cases"] and protocol["process_caps"] == prior["process_caps"]
        (E / name).write_text(json.dumps(protocol, indent=2) + "\n")
        phase = f"typed-api-prototype-v{version}-domain-predicate-{family}"
        public.append({"phase": phase, "protocol": name, "protocol_sha256": sha(E / name),
                       "command": ["/usr/bin/python3", str(E / new_runner), build, phase, unit, name],
                       "expected_cases": len(protocol["cases"]),
                       "runner": new_runner, "runner_sha256": sha(E / new_runner),
                       "predecessor_runner": original_runner, "predecessor_runner_sha256": sha(E / original_runner),
                       "runner_change": {"before": before, "after": after},
                       "probe_sha256": protocol["probe_sha256"]})
    assert sum(row["expected_cases"] for row in public) == 648
    out.write_text(json.dumps({
        "status": "Frozen before runtime builds and replay execution",
        "source_record": source_record, "source_record_sha256": source_hash,
        "build_phase": build, "unit_phase": unit,
        "caps": cap_cases, "numerical_replays": replay_cases, "public_replays": public,
        "requirements": [
            "Preserve all 39 capped test commands, 4129 numerical cases and 648 public reuse schedules; no runtime test body, probe or oracle is changed.",
            "Keep the original process limits and full-size inputs. Validate each terminal result and hash; preserve failures under their original phase names.",
            "Compare all deterministic non-timing observations with the retained runtime. Public thread schedules may change Unknown/Exhausted counts; require each observed sample/fact to satisfy its oracle and every published fact to persist.",
            "Check all successful numerical enclosures, typed errors, work accounting and serialized bytes. Public constructors and structural inspections remain outside query allowances.",
            "No builds, source edits or other validation/measurement workloads overlap a capped phase. Main admission and whole-crate proof remain separate gates.",
        ],
    }, indent=2) + "\n")
    print(json.dumps({"capped_phases": len(cap_cases), "numerical_cases": 4129,
                      "public_reuse_cases": 648}))


if __name__ == "__main__":
    main()
