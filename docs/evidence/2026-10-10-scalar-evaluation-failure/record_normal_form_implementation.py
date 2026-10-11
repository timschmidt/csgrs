#!/usr/bin/env python3
"""Check the normal-form repair against preserved regressions, queries and costs."""
from collections import Counter, defaultdict
import gzip
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
import record_algebraic_budget_implementation as evidence

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
EXPECTED = '041b19f608a46e52465bcbc331f4a5d569269809ae72bdd59e5b36bab5e1447c'
SOURCE_RECORD = 'typed-api-prototype-v648-normal-form-scalar-tests/source-hashes.json.gz'
read, sha, write = evidence.read, evidence.sha, evidence.write


def answers(phase):
    rows = defaultdict(Counter)
    for row in read(E / phase / 'receipts.json'):
        r = row['records'][0]
        if r['perturbation'] == 0:
            rows[f"warm={r['warm']}, units={r['allowance']}"][r['answer'] or 'Unknown'] += 1
    return {k: dict(v) for k, v in rows.items()}


def main():
    assert sha(E / SOURCE_RECORD) == EXPECTED
    previous = read(E / 'typed-api-prototype-v578-state.json')
    prior = json.loads(gzip.decompress((E / previous['source_record']).read_bytes()))
    source = json.loads(gzip.decompress((E / SOURCE_RECORD).read_bytes()))
    assert set(prior) <= set(source)
    changed = {n: {'before': prior.get(n), 'after': v} for n, v in source.items() if prior.get(n) != v}
    assert set(changed) == {'crates/hyperreal/src/computable/' + n for n in
                           ['budget.rs', 'node.rs', 'node/algebra.rs', 'node/structural_analysis.rs',
                            'node/symbolic_work.rs', 'node/normal_form_budget_tests.rs']}
    assert all(prior[n] == source[n] for n in prior if 'test' in n)
    assert all(sha(SOURCE / n) == v for n, v in source.items())
    expected_status = read(E.parent / '2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
    status = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).splitlines()
    assert [r for r in status if not r[3:].startswith(str(E.relative_to(ROOT))) and r[3:] != 'docs/properties-status.json'] == expected_status
    assert len(expected_status) == 49
    passes = {}
    for phase in ['typed-api-prototype-v647-normal-form-fast-cap', 'typed-api-prototype-v648-normal-form-scalar-tests',
                  'typed-api-prototype-v649-normal-form-clippy', 'typed-api-prototype-v650-normal-form-replay-build']:
        r = read(E / phase / 'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged'], phase
        assert sha(E / phase / 'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert '978 passed; 0 failed;' in (E / 'typed-api-prototype-v648-normal-form-scalar-tests/stdout.log').read_text()
    assert not any(r.startswith('warning:') for r in (E / 'typed-api-prototype-v649-normal-form-clippy/stderr.log').read_text().splitlines())
    capped = E / 'typed-api-prototype-v647-normal-form-fast-cap'
    log = (capped / 'stdout.log').read_text()
    assert '8 passed; 0 failed;' in log
    cap = json.loads(log.splitlines()[-1])
    assert cap['exit_code'] == 0 and not cap['wall_timeout']
    assert cap['address_space_bytes'] == 192 * 1024 * 1024 and cap['cpu_seconds_soft_hard'] == [2, 3] and cap['wall_seconds_limit'] == 4
    assert read(E / 'typed-api-prototype-v581-normal-form-regression-build/result.json')['exit_code'] == 101
    assert 'expected `i64`, found `i32`' in (E / 'typed-api-prototype-v581-normal-form-regression-build/stderr.log').read_text()
    for phase, counts in [('typed-api-prototype-v583-normal-form-regression', '1 passed; 5 failed;'),
                          ('typed-api-prototype-v598-normal-form-retry-regression-cap', '0 passed; 1 failed;')]:
        assert read(E / phase / 'result.json')['exit_code'] == 101
        assert counts in (E / phase / 'stdout.log').read_text()
    failed = E / 'typed-api-prototype-v590-normal-form-replay'
    assert read(failed / 'commands.json')[-1]['exit_code'] == 101
    assert read(failed / 'commands.json')[-1]['command'][-6:] == ['case', 'pi', '0', '0', '8', 'true']
    assert 'left: None' in (failed / 'case-17.stderr').read_text()
    before, after = 'typed-api-prototype-v579-normal-form-budget-baseline', 'typed-api-prototype-v651-normal-form-replay'
    for phase in [before, after]:
        r = evidence.check_artifacts(phase)
        assert r['cases'] == r['zero_budget_retained_proofs'] == 384 and r['numeric_observations'] == 960
        assert r['incomplete_cases'] == r['invalid_samples'] == 0
    assert r['source_hashes_sha256'] == EXPECTED
    for name in ['probe.rs', 'reproduce.py']:
        assert sha(E / before / name) == sha(E / after / name)
    assert [evidence.contract(r) for r in read(E / before / 'receipts.json')] == [evidence.contract(r) for r in read(E / after / 'receipts.json')]
    preserved = {}
    for phase, old, count in [
        ('typed-api-prototype-v655-normal-form-budget-replay', 'typed-api-prototype-v577-evaluation-budget-replay', 384),
        ('typed-api-prototype-v656-normal-form-algebraic-replay', 'typed-api-prototype-v523-algebraic-budget-replay', 288),
        ('typed-api-prototype-v657-normal-form-algebraic-wide-replay-v2', 'typed-api-prototype-v549-algebraic-wide-budget-replay', 144),
    ]:
        r = evidence.check_artifacts(phase)
        assert r['source_hashes_sha256'] == EXPECTED and r['cases'] == count and r['incomplete_cases'] == 0
        for name in ['probe.rs', 'reproduce.py']:
            assert sha(E / old / name) == sha(E / phase / name)
        old_commands = [evidence.contract({'command': r}) for r in read(E / old / 'commands.json') if r['name'] not in ['format', 'build']]
        new_commands = [evidence.contract({'command': r}) for r in read(E / phase / 'commands.json') if r['name'] not in ['format', 'build']]
        assert old_commands == new_commands, phase
        preserved[phase] = {'cases': count, 'summary_sha256': sha(E / phase / 'summary.json')}
    evidence.EXPECTED = EXPECTED
    costs = {}
    records = []
    for category, baseline, candidate, checksum in [
        ('normal-form', 'typed-api-prototype-v580-normal-form-cost-baseline', 'typed-api-prototype-v652-normal-form-cost', 'certified'),
        ('default', 'typed-api-prototype-v525-default-evaluation-cost', 'typed-api-prototype-v653-normal-form-default-cost', 'checksums'),
        ('algebraic', 'typed-api-prototype-v524-algebraic-cost', 'typed-api-prototype-v654-normal-form-algebraic-cost', 'certified'),
    ]:
        record = 'normal-form-' + category + '-cost-comparison-v657.json'
        costs[category] = evidence.cost(baseline, candidate, record, checksum)
        records.append(record)
    write('normal-form-budget-implementation-v657.json', {
        'status': 'Isolated normal-form work accounting and retry repair pass focused correctness validation; paired cost follow-up and full checkpoint pending; no runtime admission',
        'previous_checkpoint': 'typed-api-prototype-v578-state.json', 'baseline_commit': '4786a7f82',
        'source_record': SOURCE_RECORD, 'source_record_sha256': EXPECTED, 'changed_sources': changed,
        'contract_decision': 'p4-contract-decision-v173.json', 'prior_named_tests_unchanged': True,
        'changes': [
            'Debit active shared work before Laurent visits, normal-form memo copies, coefficient cloning, shifts, rational arithmetic, radical reduction and quadratic-surd sign comparisons.',
            'Make opaque atom equality fallible; discard interrupted temporary forms before fallback, memoization or root sign publication.',
            'Meter every bounded rational visit and selected coefficient operations while preserving all original depth, width, term and domain restrictions.',
            'Do not retain tentative Unknown signs during budgeted searches: a completed cheap walk can precede an interrupted stronger symbolic proof. Completed positive, negative and zero certificates remain reusable.',
            'Keep the preexisting shift-size rejection before allocation charges; read the Laurent accounting mode once per request and pass it through temporary-form operations.',
            'Skip coefficient accounting and fallible structural comparison for ordinary calls without an active allowance. No additional public API or persistent budget state.'
        ],
        'retained_failures': {
            'initial_test_build': 'typed-api-prototype-v581-normal-form-regression-build',
            'test_build_correction': 'Change integer fixture parameter from i32 to Rational::new\'s required i64; no assertion changed.',
            'initial_accounting_regression': 'typed-api-prototype-v583-normal-form-regression', 'failed_accounting_tests': 5, 'already_passing_undefined_test': 1,
            'first_repair_public_replay': 'typed-api-prototype-v590-normal-form-replay', 'public_case': ['pi', 0, 0, 8, True],
            'promoted_retry_regression': 'typed-api-prototype-v598-normal-form-retry-regression-cap',
            'retry_failure': 'A tentative Unknown survived budget exhaustion and suppressed a later unlimited quadratic/Laurent proof.',
            'shift_failure': 'typed-api-prototype-v610-normal-form-budget-replay',
            'shift_unit_failure': 'typed-api-prototype-v643-normal-form-shift-regression-cap',
            'shift_repair': 'Reject shifts beyond the existing coefficient limit before charging for allocation that would never occur.'
        },
        'unit_coverage': {'functions': 8, 'cap': cap, 'laurent_success_work': 140, 'rational_success_work': 94,
                          'allowance_sweeps': 'Every allowance from zero through measured successful work plus two for Laurent and bounded rational recognition.',
                          'coefficient_bits': [16, 256, 1024], 'coefficient_work': [36, 128, 788],
                          'coefficient_allowances': ['0', '1', 'needed/4', 'needed-1', 'needed', 'needed+1'],
                          'opaque_comparison_depth': 512, 'nested_square_depth': 42, 'shift_bits': 1000000,
                          'unsupported_shift_regressions': ['i32::MIN', 'i32::MAX', 'MAX_APPROXIMATION_BITS+1'],
                          'public_retry_schedules': 22, 'public_retry_allowances': [0, 1, 4, 8, 16, 32, 64, 128, 256, 512, 100000]},
        'replay': {'phase': after, 'cases': 384, 'numeric_observations': 960, 'zero_budget_retained_proofs': 384,
                   'probe_harness_arguments_order_caps_preserved': True, 'zero_answers_before': answers(before), 'zero_answers_after': answers(after)},
        'preserved_budget_replays': preserved, 'cost_records': records, 'observed_median_percent_changes': costs,
        'passing_final_source_validation': passes, 'main_runtime_unchanged': True, 'original_worktree_entries_preserved': 49,
        'paired_cost': 'normal-form-paired-cost-v657/summary.json',
        'cost_followup': 'Paired timing still shows roughly 17 percent on the wide-pi fixture and 14 percent on reciprocals. Specialize the ordinary path so dynamic accounting checks are removed before retention.',
        'orchestration_correction': 'The first v657 wide replay invocation omitted the required replay mode. Its IndexError and partial phase are retained; the v657-v2 invocation passes all 144 unchanged cases.',
        'intermediate_trials': 'normal-form-initial-trials-v643.json', 'intermediate_commit': 'e69370b45',
        'limits': [
            'Cooperative size-weighted work units are neither CPU-instruction, elapsed-time nor allocation bounds; a single exact arithmetic operation can still do additional work.',
            'The private helper is request-local and synchronous; no allowance or metering flag is retained on a node. Existing nested/atomic budget tests remain required.',
            'Eight private regressions, 22 public retry schedules and finite external matrices do not establish every cutpoint, concurrency schedule or whole-application resource bound.',
            'Inverse-trigonometric linear recognition, algebraic leaf cloning and radical/quadratic-tower helper accounting still need separate review.',
            'This runtime remains outside the conditional Verus target; thirteen whole-crate proof groups and the remaining plan work stay open.'
        ],
    })
    print(json.dumps({'source': EXPECTED, 'changed_sources': len(changed), 'new_replay_cases': 384, 'cost_percent_changes': costs}))


if __name__ == '__main__':
    main()
