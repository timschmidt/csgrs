"""Reuse the retained receipt audits without freezing race-dependent counts."""
import ast
import hashlib
import json
from pathlib import Path
import textwrap

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    output = E / "audit_domain_predicate_fact_retention.py"
    provenance = E / "domain-predicate-fact-audit-preparation-v1835.json"
    assert not output.exists() and not provenance.exists()
    bodies = {}
    retains = []
    for name, source_name in [("public", "record_public_proof_reuse.py"),
                              ("composed", "record_composed_proof_reuse.py")]:
        source = (E / source_name).read_text()
        tree = ast.parse(source)
        retained = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "retains")
        retains.append(retained)
        start = next(index for index, node in enumerate(tree.body) if isinstance(node, ast.Assign)
                     and any(isinstance(target, ast.Name) and target.id == "counts" for target in node.targets))
        stop = next(index for index in range(start, len(tree.body)) if isinstance(tree.body[index], ast.For))
        assert isinstance(tree.body[stop].target, ast.Tuple)
        assert tree.body[stop].target.elts[0].id == "row"
        bodies[name] = "\n".join(ast.get_source_segment(source, node) for node in tree.body[start:stop + 1])
    assert ast.dump(retains[0], include_attributes=False) == ast.dump(retains[1], include_attributes=False)
    source = (E / "record_public_proof_reuse.py").read_text()
    common = ast.get_source_segment(source, retains[0])
    header = '''"""Audit every retained public fact sequence after the runtime extraction."""
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import sys

sys.set_int_max_str_digits(20000)
E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()

'''
    checks = {
        "public": '''assert counts["sequential_cold_unknown"] == counts["sequential_cold_exhausted"] == 24
assert counts["sequential_finer_exhausted"] == 24
assert counts["warm_worker_samples"] == 1536
assert counts["race_worker_samples"] + counts["race_worker_exhausted"] == 4608
assert 0 <= counts["race_unknown_signs"] <= 4608
assert 1536 <= counts["known_sign_query_retention_checks"] <= 6144
assert counts["joined_final_fact_comparisons"] == 6144
assert counts["invalid_budget_queries"] == 96 and counts["invalid_unlimited_retries"] == 24
return {"observed_counts": counts}
''',
        "composed": '''assert counts["composed_cases"] == 468 and counts["invalid_cases"] == 54
assert counts["bounded:Ok"] + counts["bounded:Exhausted"] == 3276
assert counts["finer:Ok"] + counts["finer:Exhausted"] == 3276
assert counts["events:unlimited_retry"] == counts["events:finer_retry"] == 3276
assert counts["events:cached"] == 13104
assert counts["root_fact_retention_comparisons"] == 85176
assert counts["child_fact_retention_comparisons"] == 9360
assert sum(value for key, value in counts.items() if key.startswith("invalid_retry:")) == 378
return {"observed_counts": dict(counts), "bounded_outcomes_by_root": by_root, "bounded_outcomes_by_allowance": by_units}
''',
    }
    parts = [header, common, "\n\n"]
    for name in ["public", "composed"]:
        parts += [f"def audit_{name}(PHASE, summary, protocol, rows):\n",
                  textwrap.indent(bodies[name] + "\n" + checks[name], "    "), "\n\n"]
    parts.append('''def main():
    plan_path = E / "domain-predicate-replay-protocol-v1781.json"
    plan = read(plan_path)
    validation = read(E / "domain-predicate-replay-validation-v1834.json")
    assert validation["complete"] and validation["protocol_sha256"] == sha(plan_path)
    assert validation["source_record_sha256"] == plan["source_record_sha256"]
    out = E / "domain-predicate-public-fact-audit-v1835.json"
    assert not out.exists()
    reports = {}
    for label, item, audit in zip(["public", "composed"], plan["public_replays"], [audit_public, audit_composed]):
        phase = E / item["phase"]
        summary = read(phase / "summary.json")
        protocol = read(phase / "protocol.json")
        rows = read(phase / "receipts.json")
        assert summary["cases"] == len(rows) == len(protocol["cases"]) == item["expected_cases"]
        assert summary["source_hashes_sha256"] == protocol["source_record_sha256"] == plan["source_record_sha256"]
        assert summary["guards_unchanged"] and summary["incomplete_cases"] == summary["invalid_samples"] == 0
        assert sha(phase / "protocol.json") == sha(E / item["protocol"]) == item["protocol_sha256"]
        assert sha(phase / "probe.rs") == sha(E / protocol["probe"]) == item["probe_sha256"]
        assert sha(phase / "reproduce.py") == sha(E / item["runner"]) == item["runner_sha256"]
        assert all(sha(phase / name) == digest for name, digest in summary["artifacts_sha256"].items())
        assert all(row["exit_code"] == 0 and not row["wall_timeout"] for row in read(phase / "commands.json"))
        reports[label] = {"phase": phase.name, "summary_sha256": sha(phase / "summary.json"),
                          "numeric_samples_checked": summary["numeric_samples_checked"],
                          "definite_sign_observations_checked": summary["definite_sign_observations_checked"],
                          **audit(phase, summary, protocol, rows)}
    out.write_text(json.dumps({
        "status": "Every audited public and composed fact sequence retains available certificates",
        "source_record_sha256": plan["source_record_sha256"], "protocol_sha256": sha(plan_path),
        "reports": reports,
        "scope": "Reuse the exact retained receipt-audit loops, with current observation counts. Check each worker/root fact sequence, joined final snapshots and subsequent sign queries after available proofs, including exhausted allowances and finer retries. Frozen probes and Python Fraction oracle checks establish the numerical observations separately.",
        "limits": [
            "Race-dependent sample, Exhausted and Unknown counts describe this run; they are not required to match an earlier schedule.",
            "Composed child snapshots start with the first serialized observation. The frozen Rust probe also checks its initial child state, which is not serialized in this corpus.",
            "Exact-rational flags are checked for retention, not general mathematical soundness. Constructors and structural inspections lie outside query allowances.",
            "Finite thread stress and composition schedules do not prove all interleavings, general graph semantics, aggregate byte/time bounds or whole-crate correctness. Runtime retention still depends on the other gates.",
        ],
    }, indent=2) + "\\n")
    print(json.dumps({key: value["observed_counts"] for key, value in reports.items()}))


if __name__ == "__main__":
    main()
''')
    generated = "".join(parts)
    compile(generated, str(output), "exec")
    output.write_text(generated)
    provenance.write_text(json.dumps({
        "predecessors_sha256": {name: sha(E / name) for name in ["record_public_proof_reuse.py", "record_composed_proof_reuse.py"]},
        "generated_audit": output.name, "generated_audit_sha256": sha(output),
        "common_retention_function_ast_identical": True,
        "change": "Extract the retained fact comparison and receipt loops unchanged. Preserve fixed corpus totals while reporting, rather than fixing, race-dependent observations. Remove ledger/admission mutations and require all new replay artifacts and frozen probes to match their hashes.",
        "status": "Prepared only; new replay and audit executions remain pending",
    }, indent=2) + "\n")
    print(json.dumps({"audit": output.name, "source_sha256": sha(output)}))


if __name__ == "__main__":
    main()
