"""Prepare, but do not execute, the next isolated retention audit."""
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    predecessor = E / "record_domain_predicate_retention.py"
    runner = E / "record_bound_merge_retention.py"
    patch = E / "bound-merge-retention-audit-fork-v1913.patch"
    provenance = E / "bound-merge-retention-audit-preparation-v1913.json"
    assert not any(path.exists() for path in [runner, patch, provenance])
    original = predecessor.read_text()
    text = original.replace("domain-predicate", "bound-merge").replace("domain_predicate", "bound_merge")
    text = re.sub(r"v(1754|17[6-9][0-9]|18[0-2][0-9]|183[0-6])\b",
                  lambda match: "v" + str(1836 if int(match[1]) == 1754 else int(match[1]) + 77), text)
    replacements = {
        "7a91b5d47685ba42e68609869b2712ecb56098061a1f8fb8c03a9813190ef8c6":
            "2cdd765b340ce6d0a331aeb25b2596639bc262bb18805cb7efeed27aa43143b9",
        "Retain the proved production predicate only after all frozen runtime gates.":
            "Retain compatible cached-bound merging only after all frozen runtime gates.",
        'proof["verified_units"] == 645': 'proof["verified_units"] == 651',
        'proof["listed_production_contracts"] == 97': 'proof["listed_production_contracts"] == 98',
        'proof["listed_model_theorems"] == 130': 'proof["listed_model_theorems"] == 135',
        'expected_tests = {1764: (1074, 1, 0), 1769: (5858, 3, 33),\n                      1770: (1059, 3, 0), 1771: (1074, 1, 0)}':
            'expected_tests = {1841: (1074, 1, 0), 1846: (5858, 3, 33),\n                      1847: (1059, 3, 0), 1848: (1074, 1, 0)}',
        'and len(changed) == 7': 'and len(changed) == 6',
        "The proof includes the actual production predicate body and establishes a real-domain consequence conditional on a valid incoming bound. Independent real existence, producer soundness, caller ordering, graph semantics and atomic publication remain separate obligations; all thirteen whole-crate groups remain open.":
            "The proof includes the actual production BoundInfo::merge body. Two valid bounds for the same real value are compatible; merging them preserves denotation, all available certificates and accepted domain predicates. Producer soundness and independent real existence remain premises. The atomic load/CAS loop, caller ordering and graph semantics remain separate obligations; all thirteen whole-crate groups remain open.",
        "Ten acceptance/dependency tests and 226 rejected mutations plus four passing baselines do not satisfy the still-pending fixed pre-Verus acceptance report. The failed v1837 proof attempt and all earlier failed experiments remain immutable.":
            "Ten acceptance/dependency tests and 246 rejected mutations plus four passing baselines do not satisfy the still-pending fixed pre-Verus acceptance report. This extraction has no failed proof attempt; all earlier failed experiments remain immutable.",
        "All 1566 fresh cost processes and 1098 separate CPU preludes pass their original caps. The initial 5.0501 percent pi increase and prescribed 1.0191 percent repeat both remain recorded. Seven local probe executables shrink 48-152 bytes. These measurements establish neither causal attribution, a uniform speedup nor whole-board performance/size parity.":
            "All fresh standalone, original paired and prescribed family-repeat processes pass their original caps. The cost record preserves all initial/repeated timing observations and separate executable sizes. These measurements establish neither causal attribution, a uniform speedup nor whole-board performance/size parity.",
        "Conditional predicate soundness does not discharge those prerequisites.":
            "Conditional merge and domain soundness do not discharge those prerequisites.",
        "Retain the isolated proved cached magnitude-domain predicate extraction; main admission remains open":
            "Retain the isolated proved compatible cached-bound merge; main admission remains open",
        '"cost_processes": 1566, "separate_cpu_preludes": 1098':
            '"cost_processes": processes, "separate_cpu_preludes": preludes',
        '"initial_and_followup_pi_percent": costs["initial_and_followup_pi_percent"]':
            '"initial_and_followup_percent": costs["initial_and_followup_percent"]',
        "Keep the exact runtime domain decisions and route them through the same pure BoundInfo method included by Verus. The extraction adds conditional proof coverage without losing any observed successful query, fact, work allowance or allocation behavior in the frozen deterministic corpora. The original-cap cost comparison and predeclared repeat pass. This is the new isolated baseline, not main runtime admission.":
            "Keep the exact runtime bound selection and route it through the same pure BoundInfo::merge body included by Verus. The extraction adds conditional denotation, compatibility, certificate-retention and domain-retention proofs without losing any observed successful query, fact, work allowance or allocation behavior in the frozen deterministic corpora. Original-cap costs and the predeclared repeat rule pass. This is the new isolated baseline; main runtime admission remains open.",
        "Historical review of v1836. Its public/concurrent/composed coverage gaps now have finite replay evidence and its cached-bound predicate has a conditional production proof; producer/existence/atomic/caller and resource obligations remain open.":
            "Historical review of v1754. Its public/concurrent/composed coverage gaps now have finite replay evidence, and cached-bound domain decisions and compatible merging have conditional production proofs. Producer/existence/atomic/caller and resource obligations remain open.",
        '"The v1913 isolated checkpoint retains the proved cached-domain predicate extraction "\n        "after all frozen gates: 645 verified units, 226 rejected mutations, four passing guard "':
            '"The v1913 isolated checkpoint retains the proved compatible cached-bound merge "\n        "after all frozen gates: 651 verified units, 246 rejected mutations, four passing guard "',
        '"The 1566 original-cap cost processes and 1098 CPU preludes pass, including the "\n        "predeclared pi follow-up; older cost failures/tradeoff and the fixed pre-Verus "':
            'f"The {processes} original-cap cost processes and {preludes} CPU preludes pass, "\n        "including all prescribed family repeats; older cost failures/tradeoff and the fixed pre-Verus "',
        '"evidence_commits": ["520dcad721e1630e232c168e67e40894bdf4374e", "96b45f3c20b935ad00a2354e78c3515f3aabd484", "29dc377ac4f6ed7fe31a8bc3849f96f79f6a96ef"]':
            '"evidence_commits": ["21a45c3ed2b7aee9e5f98d8beaba184b4a8f37f6", "0e865eca8bb98f83137c66fe78d2b9793b1f6557", subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()]',
    }
    for old, new in replacements.items():
        assert text.count(old) == 1, old
        text = text.replace(old, new, 1)
    text = text.replace('proof["fresh_rejected_mutants"] == 226', 'proof["fresh_rejected_mutants"] == 246')
    start = text.index('    assert costs["successful_cost_processes_including_standalone"] == 1566')
    end = text.index('\n    source_record = ', start)
    text = text[:start] + '''    cost_controller_path = E / "bound-merge-cost-validation-v1857.json"
    cost_controller = read(cost_controller_path)
    cost_milestone = read(E / "bound-merge-cost-milestone-v1857.json")
    assert cost_controller["complete"] and costs["local_timing_and_process_gates_passed"]
    assert not costs["persistent_above_five_percent_increases"]
    assert cost_milestone["cost_controller_sha256"] == sha(cost_controller_path)
    assert cost_milestone["cost_record_sha256"] == sha(E / "bound-merge-cost-review-v1857.json")
    assert cost_milestone["source_record_sha256"] == cost_controller["source_record_sha256"] == SOURCE_HASH
    assert subprocess.check_output(["git", "show", "HEAD:" + str((E / "bound-merge-cost-review-v1857.json").relative_to(ROOT))], cwd=ROOT) == (E / "bound-merge-cost-review-v1857.json").read_bytes()
    repeats = sum(row["successful_processes"] for row in costs["followups"])
    processes, preludes = 1404 + repeats, 936 + repeats
    assert costs["successful_cost_processes_including_standalone"] == processes
    assert costs["separate_cpu_preparation_processes"] == preludes and costs["original_process_caps_preserved"]
    assert costs["original_run"]["complete_fixtures"] == 52
    assert costs["original_run"]["successful_processes"] == 936 and costs["original_run"]["failed_attempts"] == 0
    assert all(row["failed_attempts"] == 0 for row in costs["followups"])
    assert all(row["non_timing_outputs_identical"] for row in costs["families"].values())
    assert all(case["median_percent_change"] <= 5 for row in costs["followups"]
               for cases in row["cases"].values() for case in cases)
''' + text[end:]
    compile(text, str(runner), "exec")
    assert "1566" not in text and "1098" not in text and "226" not in text
    assert "pi_percent" not in text and 'costs["followup"]' not in text
    runner.write_text(text)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(keepends=True), text.splitlines(keepends=True),
                    fromfile=predecessor.name, tofile=runner.name)))
    provenance.write_text(json.dumps({
        "predecessor": predecessor.name, "predecessor_sha256": sha(predecessor),
        "runner": runner.name, "runner_sha256": sha(runner),
        "fork_patch": patch.name, "fork_patch_sha256": sha(patch),
        "status": "Prepared only; all cost and replay gates must finish before execution",
        "scope": "Preserve the prior retention audit, update source/phase/proof identities, and audit the predeclared family-repeat cost protocol without a pi-specific or shrinking-executable assumption. No source or ledger changes occur during preparation.",
    }, indent=2) + "\n")
    print(json.dumps({"runner": runner.name, "runner_sha256": sha(runner), "prepared_only": True}))


if __name__ == "__main__":
    main()
