#!/usr/bin/env python3
"""Record the retained cache-contract experiments without rerunning workloads."""
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
from parse_test_results_v2 import parse_phase

EVIDENCE = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def phase(version):
    matches = [p for p in EVIDENCE.glob(f"typed-api-prototype-v{version}-*") if p.is_dir()]
    assert len(matches) == 1, (version, matches)
    return matches[0]


def main():
    out = EVIDENCE / sys.argv[1]
    assert not out.exists()
    runs = {}
    for version in [167, 168, 169, 170, 171, 172, 173, 174, 175, 176]:
        path = phase(version)
        result = json.loads((path / "result.json").read_text())
        assert result["source_unchanged"] and result["main_unchanged"]
        log = (path / "stdout.log").read_text()
        runs[path.name] = {
            "result": result,
            "source_record_sha256": digest(path / "source-hashes.json.gz"),
            "tests": parse_phase(path),
            "failed_test_names": re.findall(r"^test (.+) \.\.\. FAILED$", log, re.M),
        }
    weakened = re.findall(r"^case=.*$", (phase(173) / "stdout.log").read_text(), re.M)
    assert len(weakened) == 16
    assert all("exact_msd: true" in line and "exact_msd: false" in line for line in weakened)
    false_facts = re.findall(r"^operand=.*$", (phase(171) / "stdout.log").read_text(), re.M)
    assert len(false_facts) == 18
    assert runs[phase(170).name]["tests"]["top_level_failed"] == 10
    assert runs[phase(175).name]["tests"]["top_level_failed"] == 1
    assert runs[phase(176).name]["tests"]["top_level_passed"] == 916
    assert runs[phase(176).name]["tests"]["top_level_failed"] == 0
    interruption = json.loads((phase(165) / "interruption.json").read_text())
    assert interruption["authoritative_session_exit_code"] == 130
    cap_log = (phase(166) / "stdout.log").read_text()
    cap = json.loads(cap_log[cap_log.index('{"command":'):])
    assert cap["exit_code"] == -24 and not cap["wall_timeout"]
    decision_path = EVIDENCE / "p4-contract-decision-v173.json"
    decision = json.loads(decision_path.read_text())
    data = {
        "status": "Reproduced failures and rejected strict-equality experiment retained; repairs remain an isolated candidate",
        "helper_sha256": digest(Path(__file__)),
        "contract_decision": decision_path.name,
        "contract_decision_sha256": digest(decision_path),
        "user_answer": decision["user_answer"],
        "classification": {
            "allowed": "Unknown-to-Certified strengthening and reuse of previously established sign/domain/algebraic proofs, including later queries with lower refinement limits.",
            "forbidden": "Contradictions, weakened or lost available certificates, and numeric results outside the existing value/error contract.",
            "serialization": "Expression structure is retained; skipped accelerator proofs may need to be established again. A fresh decoded graph need not contain its source graph's transient proofs.",
        },
        "strict_experiment": {
            "review": "cache-contract-review-v172.json",
            "decision": "Rejected because it discards valid proof reuse; all experiment sources and failures remain archived.",
            "partial_run": interruption,
            "original_constructor_cap": cap,
        },
        "exact_magnitude_regression": {
            "observations": 54, "weakened_observations": len(weakened),
            "fixtures": 9, "precisions": [0, -16, -128],
            "orders": ["approximation before facts", "facts before approximation"],
            "failure_lines": weakened,
            "repair": "Discover structural bounds before an approximation may fill an unresolved bound slot.",
        },
        "exponential_domain_regression": {
            "original_false_fact_responses": len(false_facts),
            "failure_lines": false_facts,
            "final_cases": 36, "undefined_operands": 6,
            "sine_wrapper_depths": [0, 128], "deltas": [-1, 0, 1],
            "repair": "Prove each exponential argument real before relation cancellation.",
            "deep_valid_control": "All original 27 deep independent-atom proof queries remain, across 7/999/2048-bit deltas and three repeated queries.",
            "domain_budget_control": "A 300-level valid sine chain is declined cold and accepted after an independent existence proof; no node gets a numeric cache.",
        },
        "constructor_regression": {
            "workload": "public exp() on 48 raw nested exponentials over one",
            "original_cap_phase": phase(166).name,
            "retained_test": "cache_magnitude_tests::unknown_magnitude_exp_construction_does_not_evaluate_descendants",
            "valid_control": "exp(ln(2)) encloses exact two at precisions 0 and -64",
        },
        "runs": runs,
        "limits": [
            "Capped final-source runs and full downstream validation are recorded separately in the checkpoint.",
            "No complete cache concurrency, serialization, private-kernel accuracy, aggregate resource or production-admission claim.",
            "All 39 properties, 14 targets and crate future work remain in scope.",
        ],
    }
    out.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"runs": len(runs), "weakened_magnitude_responses": len(weakened),
                      "false_exp_facts": len(false_facts), "final_unit_passed": 916}))


if __name__ == "__main__":
    main()
