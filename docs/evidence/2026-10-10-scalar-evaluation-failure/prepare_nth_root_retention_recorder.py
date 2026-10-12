"""Prepare conditional retention with reused guards and separate kernel evidence."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parent = E / "record_cache_state_merge_retention.py"
    target = E / "record_nth_root_enclosure_retention.py"
    out = E / "nth-root-enclosure-retention-preparation-v2077.json"
    patch = E / "nth-root-enclosure-retention-preparation-v2077.patch"
    assert not any(path.exists() for path in [target, out, patch])
    original = parent.read_text()
    source = re.sub(r"v(\d+)\b", lambda match:
        "v" + str(int(match[1]) + 78) if 1927 <= int(match[1]) <= 1999 else match[0], original)
    source = source.replace("cache-state-merge", "nth-root-enclosure")
    source = source.replace("run_cache_state_replays.py", "run_nth_root_replays.py")
    source = source.replace("nth-root-enclosure-proof-validation-v1926.json", "nth-root-enclosure-proof-validation-v2004.json")
    source = source.replace("555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59",
                            "f5fa38edebe8eb5ecdf831098567fab0cf0d8d6f588b56f1c93c4d5f56a5cac3")

    def replace(before, after, count=1):
        nonlocal source
        assert source.count(before) == count, (before, source.count(before))
        source = source.replace(before, after)

    before = '''    assert proof["fresh_rejected_mutants"] == 247 and proof["fresh_passing_guard_baselines"] == 4
    assert proof["reused_rejected_mutants"] == 0
    for name, digest in proof["prior_experiments_sha256"].items():
        assert sha(E / name) == digest
    rejected = read(E / "bound-merge-cost-rejection-v1857.json")
    assert not rejected["runtime_retained"]
    for name, diagnostic in proof["compile_only_diagnostics"].items():
        assert sha(E / name / "summary.json") == diagnostic["summary_sha256"]
        assert diagnostic["timed_processes"] == 0
    for key in ["fresh_verification_phase", "fresh_acceptance_phase", "fresh_rejection_phase"]:
        checked_phase(proof[key])
'''
    after = '''    assert proof["focused_tests"] == 3
    assert proof["fresh_rejected_mutants"] == proof["fresh_passing_guard_baselines"] == 0
    assert proof["reused_rejected_mutants"] == 247 and proof["reused_passing_guard_baselines"] == 4
    reused_path = E / proof["reused_proof_record"]
    assert sha(reused_path) == proof["reused_proof_record_sha256"]
    reused = read(reused_path)
    assert reused["proof_and_guard_inputs_sha256"] == proof["proof_and_guard_inputs_sha256"]
    for name, digest in reused["prior_experiments_sha256"].items():
        assert sha(E / name) == digest
    for name, diagnostic in reused["compile_only_diagnostics"].items():
        assert sha(E / name / "summary.json") == diagnostic["summary_sha256"]
        assert diagnostic["timed_processes"] == 0
    prior_failure_path = E / proof["prior_failure"]
    assert sha(prior_failure_path) == proof["prior_failure_sha256"]
    assert not read(prior_failure_path)["runtime_retained"]
    assert not read(E / "bound-merge-cost-rejection-v1857.json")["runtime_retained"]
    rejection = E / proof["reused_rejection_phase"]
    rejection_result = read(rejection / "result.json")
    assert rejection_result["exit_code"] == 0 and rejection_result["source_unchanged"] and rejection_result["main_unchanged"]
    assert sha(rejection / "proof-checks/tooling.json") == proof["reused_rejection_tooling_sha256"]
    for key in ["focused_phase", "fresh_verification_phase", "fresh_acceptance_phase"]:
        checked_phase(proof[key])
    assert sha(E / proof["fresh_verification_phase"] / "proof-reports/dependencies.json") == proof["proof_dependency_identity_sha256"]
    assert sha(E / reused["fresh_verification_phase"] / "proof-reports/dependencies.json") == proof["proof_dependency_identity_sha256"]
'''
    replace(before, after)
    replace('expected_tests = {1927: (1074, 1, 0), 1932: (5858, 3, 33),\n                      1933: (1059, 3, 0), 1934: (1074, 1, 0)}',
            'expected_tests = {2005: (1076, 1, 0), 2010: (5860, 3, 33),\n                      2011: (1061, 3, 0), 2012: (1076, 1, 0)}')
    replace('and len(changed) == 6', 'and len(changed) == 8')
    anchor = '    limits = [\n'
    allocation = '''    allocation_path = E / "nth-root-enclosure-allocation-study-v2021/summary.json"
    allocation = read(allocation_path)
    assert allocation["source_record_sha256"] == SOURCE_HASH and allocation["guards_unchanged"]
    assert allocation["exact_endpoints_equal"] and allocation["counts_stable"]
    assert allocation["fixtures"] == 84 and allocation["kernel_calls"] == 504
    assert not allocation["increases"], "Review allocation increases before retention"
    for name, digest in allocation["artifacts_sha256"].items():
        assert sha(allocation_path.parent / name) == digest

'''
    replace(anchor, allocation + anchor)
    replace('Ten acceptance/dependency tests and 247 rejected mutations plus four passing baselines do not satisfy the still-pending fixed pre-Verus acceptance report. The outlined extraction has no failed proof attempt. The earlier BoundInfo extraction failed its timing gate; its rejection, the unsupported Option::or probe and the compile-only variants remain immutable.',
            'Ten fresh acceptance tests and 651 freshly verified units cover the unchanged conditional proof target. The 247 rejected mutations and four passing baselines are reused, with identical proof/guard inputs and dependency identities; zero new mutations were run. The bigint root kernel is outside Verus coverage. Its exact endpoints have focused inequality/differential tests and unchanged public MPFR cases. The fixed pre-Verus acceptance report remains pending. All earlier failed experiments remain immutable.')
    replace('The local cost baseline remains v1836. Older original-cap failures at v1638/v1710, the rejected BoundInfo extraction\'s v1857 timing regressions, the earlier retained integer-control tradeoff and the fixed pre-Verus acceptance baseline are not erased or reset by this successful comparison.',
            'The local cost baseline remains v1836. The outlined selector alone failed its original CPU cap at v1943. That failure, v1857 timing regressions, original-cap failures at v1638/v1710, the retained integer-control tradeoff and the fixed pre-Verus acceptance baseline remain unchanged; this combined-source run does not pass any earlier failed run.')
    replace('        "EvaluationBudget excludes aggregate bytes/time and unmetered algebraic helpers.',
            '        "The 84-case copied-kernel allocation study records stable counts and identical endpoints across 504 helper calls. It measures allocation requests only, not production call-site behavior, live/peak memory, query work or time. All cases and its limitations remain recorded separately.",\n'
            '        "EvaluationBudget excludes aggregate bytes/time and unmetered algebraic helpers.')
    replace('"status": "Retain the isolated proved outlined cache-state merge; main admission remains open"',
            '"status": "Retain the isolated one-root enclosure and conditionally proved outlined cache merge; main admission remains open"')
    replace('        "prior_experiments_sha256": proof["prior_experiments_sha256"],\n'
            '        "compile_only_diagnostics": proof["compile_only_diagnostics"],\n',
            '        "prior_experiments_sha256": reused["prior_experiments_sha256"],\n'
            '        "compile_only_diagnostics": reused["compile_only_diagnostics"],\n'
            '        "prior_capped_failure": proof["prior_failure"], "prior_capped_failure_sha256": proof["prior_failure_sha256"],\n'
            '        "allocation_study": str(allocation_path.relative_to(E)), "allocation_study_sha256": sha(allocation_path),\n')
    replace('"fresh_passing_guard_baselines", "reused_rejected_mutants"]',
            '"fresh_passing_guard_baselines", "reused_rejected_mutants", "reused_passing_guard_baselines", "focused_tests"]')
    replace('Keep the exact runtime bound selection and route it through the same pure BoundCache::merge body included by Verus. The extraction adds conditional denotation, compatibility, certificate-retention and domain-retention proofs without losing any observed successful query, fact, work allowance or allocation behavior in the frozen deterministic corpora. Original-cap costs and the predeclared repeat rule pass. This is the new isolated baseline; main runtime admission remains open.',
            'Keep the one-root integer enclosure with identical tested endpoints, together with the same pure BoundCache::merge body included by Verus. The kernel change reduces duplicate root computation; it has finite numerical and allocation evidence, not a formal kernel proof. The outlined selector has conditional denotation, compatibility, certificate-retention and domain-retention proofs. All frozen deterministic observations, original-cap costs and prescribed repeats pass. This becomes the isolated retained baseline; main runtime admission remains open.')
    replace('record["deterministic_replay_comparison"], record["public_fact_audit"], outputs[0].name,',
            'record["deterministic_replay_comparison"], record["public_fact_audit"], record["allocation_study"], outputs[0].name,')
    replace('"The v2077 isolated checkpoint retains the proved compatible cached-bound merge "\n'
            '        "after all frozen gates: 651 verified units, 247 rejected mutations, four passing guard "\n'
            '        "baselines, ten acceptance tests, nine broad commands, 39 capped phases (156 top-level "',
            '"The v2077 isolated checkpoint retains the one-root enclosure and outlined cached-bound merge "\n'
            '        "after all frozen gates: 651 fresh verified units, 247 reused rejected mutations and four reused "\n'
            '        "passing guard baselines, ten fresh acceptance tests, nine broad commands, 39 capped phases (156 top-level "')
    replace('"baseline remain explicit. This changes only the isolated retained candidate. "',
            '"baseline remain explicit. The bigint kernel is outside Verus; its focused endpoint checks and "\n'
            '        "84-case copied-kernel allocation study remain separate finite evidence. This changes only the isolated retained candidate. "')
    replace('the extraction is retained in v2077', 'the combined proposal is retained in v2077')
    replace('"dec6ce44ffc109de6f8f52d2ce253cec1cae5dbf",', '"dec6ce44ffc109de6f8f52d2ce253cec1cae5dbf", "85a24c007460551c9dee5746b68a192cb569b045",')
    replace('"top_level_passed": 5858', '"top_level_passed": 5860')
    replace('"top_level_passed": 1074', '"top_level_passed": 1076')
    replace('        "cost_record": record["cost_record"], "original_cost_guard_failures_preserved": True,',
            '        "cost_record": record["cost_record"], "original_cost_guard_failures_preserved": True,\n'
            '        "allocation_study": record["allocation_study"], "allocation_study_sha256": record["allocation_study_sha256"],')
    ast.parse(source)
    target.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                               fromfile=parent.name, tofile=target.name, n=0)))
    out.write_text(json.dumps({
        "status": "Prepared only; all runtime, cost, allocation and replay gates must pass before execution",
        "source_record_sha256": "f5fa38edebe8eb5ecdf831098567fab0cf0d8d6f588b56f1c93c4d5f56a5cac3",
        "predecessor": parent.name, "predecessor_sha256": sha(parent),
        "recorder": target.name, "recorder_sha256": sha(target),
        "patch": patch.name, "patch_sha256": sha(patch), "preparer_sha256": sha(Path(__file__)),
        "conditional_retention_checkpoint": "typed-api-prototype-v2077-state.json",
        "retained_runtime_checkpoint": "typed-api-prototype-v1836-state.json",
        "scope": "Preserve every original cost, cap, replay and fact-retention audit, accounting for two new scalar tests and eight changed files. Require the supplementary copied-kernel allocation study and report its limited scope. Distinguish fresh conditional verification from reused mutation guards and the unverified bigint kernel. Preserve all earlier failures and every property/target status; preparation is not retention.",
    }, indent=2) + "\n")
    print(json.dumps({"prepared": target.name, "executed": False}))


if __name__ == "__main__":
    main()
