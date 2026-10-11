#!/usr/bin/env python3
"""Preserve ordinary-query accounting fast paths and their matching replay/cost evidence."""
import gzip
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import record_algebraic_budget_implementation as evidence

E = Path(__file__).resolve().parent
read, sha, write = evidence.read, evidence.sha, evidence.write
SOURCE_RECORD = 'typed-api-prototype-v520-algebraic-budget-tests/source-hashes.json.gz'
EXPECTED = 'd6527953cefc76fb9c5f914434459b74e5949b08a22c755a5b83684a1b4b7deb'


def main():
    assert sha(E / SOURCE_RECORD) == EXPECTED
    before = read(E / 'algebraic-budget-implementation-v517.json')
    old = json.loads(gzip.decompress((E / before['source_record']).read_bytes()))
    new = json.loads(gzip.decompress((E / SOURCE_RECORD).read_bytes()))
    assert old.keys() == new.keys()
    changed = {k: {'before': old[k], 'after': v} for k, v in new.items() if old[k] != v}
    assert set(changed) == {'crates/hyperreal/src/computable/' + p for p in ['budget.rs', 'node/structural_analysis.rs']}
    assert all(sha(evidence.SOURCE / name) == value for name, value in new.items())
    passes = {}
    for phase in ['typed-api-prototype-v520-algebraic-budget-tests', 'typed-api-prototype-v521-stack-clippy', 'typed-api-prototype-v522-replay-build']:
        r = read(E / phase / 'result.json')
        assert r['exit_code'] == 0 and r['main_unchanged'] and r['source_unchanged']
        assert sha(E / phase / 'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert '4 passed; 0 failed;' in (E / 'typed-api-prototype-v520-algebraic-budget-tests/stdout.log').read_text()
    assert not any(line.startswith('warning:') for line in (E / 'typed-api-prototype-v521-stack-clippy/stderr.log').read_text().splitlines())
    a, b = 'typed-api-prototype-v515-algebraic-budget-replay', 'typed-api-prototype-v523-algebraic-budget-replay'
    for phase in [a, b]:
        r = evidence.check_artifacts(phase)
        assert r['cases'] == r['zero_budget_retained_proofs'] == 288 and r['numeric_observations'] == 576
        assert r['incomplete_cases'] == r['invalid_samples'] == 0
    assert r['source_hashes_sha256'] == EXPECTED
    for name in ['probe.rs', 'reproduce.py']:
        assert sha(E / a / name) == sha(E / b / name)
    old_rows, new_rows = read(E / a / 'receipts.json'), read(E / b / 'receipts.json')
    assert [evidence.contract(r) for r in old_rows] == [evidence.contract(r) for r in new_rows]
    assert [r['records'] for r in old_rows] == [r['records'] for r in new_rows]
    evidence.EXPECTED = EXPECTED
    algebraic = evidence.cost('typed-api-prototype-v507-algebraic-cost-baseline', 'typed-api-prototype-v524-algebraic-cost', 'algebraic-budget-cost-comparison-v525.json', 'certified')
    default = evidence.cost('typed-api-prototype-v442-evaluation-budget-cost', 'typed-api-prototype-v525-default-evaluation-cost', 'algebraic-budget-default-cost-comparison-v525.json', 'checksums')
    first, final = read(E / 'typed-api-prototype-v516-algebraic-cost/summary.json'), read(E / 'typed-api-prototype-v524-algebraic-cost/summary.json')
    comparison = {k: {'first_median_ns': v['median_ns'], 'final_median_ns': final['cases'][k]['median_ns'], 'median_ratio': final['cases'][k]['median_ns'] / v['median_ns']} for k, v in first['cases'].items()}
    write('algebraic-budget-fast-path-v525.json', {
        'status': 'Isolated ordinary-query fast path passes unchanged four regressions, strict five-crate Clippy, identical 288 replay outcomes/work counts and two timing corpora; complete checkpoint validation pending; no runtime admission',
        'prior_implementation_record': 'algebraic-budget-implementation-v517.json', 'prior_implementation_commit': '6cd8b3990',
        'source_record': SOURCE_RECORD, 'source_record_sha256': EXPECTED, 'changed_sources': changed,
        'change': 'Read whether a caller allowance is active before algebraic metadata traversal and dependency loops. Ordinary calls skip accounting and use the existing exact structural comparator. Active requests retain every debit and fallible comparison. No new public API, certificate cache or altered arithmetic.',
        'source_level_argument': 'The private synchronous parser and dependency helpers cannot introduce caller budget scopes; they have no external callback. Their metering flag is scoped to the current call, never retained on a node. Active includes exhausted scopes, so the flag cannot hide a failed allowance. Existing nested/atomic/retry tests remain required.',
        'trigger': 'First-trial cube/fourth-root medians were about 17/20 percent slower than the frozen baseline; both raw timing sets remain archived.',
        'replay': {'phase': b, 'cases': 288, 'numeric_observations': 576, 'zero_budget_retained_proofs': 288, 'all_answers_samples_and_work_counters_equal_to_first_implementation': True},
        'cost_records': ['algebraic-budget-cost-comparison-v525.json', 'algebraic-budget-default-cost-comparison-v525.json'],
        'observed_median_percent_changes': {'algebraic': algebraic, 'default': default},
        'compared_with_first_trial': comparison, 'passing_final_source_validation': passes,
        'all_prior_tests_unchanged': True, 'limits': before['limits'][:-1] + ['The implementation is outside the unchanged conditional Verus target. Thirteen whole-crate proof groups remain open. All 39 properties, 14 targets and crate future work remain in scope with their existing statuses preserved.'],
    })
    print(json.dumps({'source': EXPECTED, 'algebraic_median_percent_changes': algebraic, 'default_median_percent_changes': default, 'first_trial_comparison': comparison}))


if __name__ == '__main__':
    main()
