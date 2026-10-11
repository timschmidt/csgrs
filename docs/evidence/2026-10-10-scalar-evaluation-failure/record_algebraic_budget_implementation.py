#!/usr/bin/env python3
"""Check and retain the algebraic budget regression, repair, replay and cost evidence."""
from collections import Counter, defaultdict
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
EXPECTED = '8cf89a1bf55963e29f8aa939eefdd964b5754e7a56bd0c156f718b777cac81e8'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def write(name, value):
    p = E / name
    assert not p.exists(), p
    p.write_text(json.dumps(value, indent=2) + '\n')


def contract(row):
    c = row.get('command', row)
    return {k: [re.sub(r'/target/typed-api-prototype-v\d+-[^/]+/', '/target/PHASE/', part) for part in v] if k == 'command' else v
            for k, v in c.items() if k in ['name', 'command', 'timeout_seconds', 'address_space_cap_mib', 'cpu_seconds']}


def check_artifacts(phase):
    d = read(E / phase / 'summary.json')
    assert d['guards_unchanged']
    for name, value in d['artifacts_sha256'].items():
        assert sha(E / phase / name) == value, (phase, name)
    return d


def cost(before, after, out, checksum):
    a, b = check_artifacts(before), check_artifacts(after)
    assert b['source_hashes_sha256'] == EXPECTED
    assert a['processes'] == b['processes'] == 54
    for name in ['probe.rs', 'reproduce.py']:
        assert sha(E / before / name) == sha(E / after / name)
    old = [row for row in read(E / before / 'commands.json') if row['name'] not in ['format', 'build']]
    new = [row for row in read(E / after / 'commands.json') if row['name'] not in ['format', 'build']]
    # Compare inputs and caps; timings and library locations intentionally differ.
    def command_contract(row):
        return contract({'command': row})
    assert [command_contract(row) for row in old] == [command_contract(row) for row in new]
    assert a['cases'].keys() == b['cases'].keys()
    cases = {}
    for name, old in a['cases'].items():
        new = b['cases'][name]
        assert old['iterations'] == new['iterations'] and old[checksum] == new[checksum]
        assert len(old['elapsed_ns']) == len(new['elapsed_ns']) == 9
        cases[name] = {'before': old, 'after': new, 'median_ratio': new['median_ns'] / old['median_ns']}
    result = {'baseline_phase': before, 'candidate_phase': after,
              'baseline_summary_sha256': sha(E / before / 'summary.json'),
              'candidate_summary_sha256': sha(E / after / 'summary.json'),
              'source_record_sha256': EXPECTED, 'probe_and_harness_identical': True,
              'inputs_order_caps_checksums_and_iterations_preserved': True, 'cases': cases,
              'scope': 'Six scoped workloads, nine sequential separate-process repetitions each. Local timing observations; no allocation, binary-size, whole-application or production admission claim.'}
    write(out, result)
    return {name: round(100 * (row['median_ratio'] - 1), 2) for name, row in cases.items()}


def answers(phase):
    counts = defaultdict(Counter)
    for row in read(E / phase / 'receipts.json'):
        r = row['records'][0]
        if r['perturbation'] == 0:
            counts[str(r['allowance'])][r['answer'] or 'Unknown'] += 1
    return {k: dict(v) for k, v in counts.items()}


def main():
    source_record = 'typed-api-prototype-v512-algebraic-budget-tests/source-hashes.json.gz'
    assert sha(E / source_record) == EXPECTED
    sources = json.loads(gzip.decompress((E / source_record).read_bytes()))
    previous = read(E / 'typed-api-prototype-v504-state.json')
    prior = json.loads(gzip.decompress((E / previous['source_record']).read_bytes()))
    assert set(prior) <= set(sources)
    changed = {k: {'before': prior.get(k), 'after': v} for k, v in sources.items() if prior.get(k) != v}
    assert set(changed) == {'crates/hyperreal/src/computable/' + p for p in ['budget.rs', 'node.rs', 'node/structural_analysis.rs', 'node/algebraic_budget_tests.rs']}
    assert all(prior[name] == sources[name] for name in prior if 'test' in name)
    assert all(sha(SOURCE / name) == value for name, value in sources.items())
    expected_status = read(E.parent / '2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
    status = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).splitlines()
    assert [line for line in status if not line[3:].startswith(str(E.relative_to(ROOT))) and line[3:] != 'docs/properties-status.json'] == expected_status
    assert len(expected_status) == 49
    passes = {}
    for phase in ['typed-api-prototype-v512-algebraic-budget-tests', 'typed-api-prototype-v513-stack-clippy', 'typed-api-prototype-v514-replay-build']:
        r = read(E / phase / 'result.json')
        assert r['exit_code'] == 0 and r['main_unchanged'] and r['source_unchanged']
        assert sha(E / phase / 'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert not any(line.startswith('warning:') for line in (E / 'typed-api-prototype-v513-stack-clippy/stderr.log').read_text().splitlines())
    regression = 'typed-api-prototype-v509-algebraic-budget-regression'
    assert read(E / regression / 'result.json')['exit_code'] == 101
    assert '1 passed; 3 failed;' in (E / regression / 'stdout.log').read_text()
    assert read(E / 'typed-api-prototype-v508-algebraic-budget-regression/result.json')['exit_code'] == 101
    assert 'type annotations needed' in (E / 'typed-api-prototype-v508-algebraic-budget-regression/stderr.log').read_text()
    before, after = 'typed-api-prototype-v506-algebraic-budget-baseline', 'typed-api-prototype-v515-algebraic-budget-replay'
    for phase in [before, after]:
        r = check_artifacts(phase)
        assert r['cases'] == r['zero_budget_retained_proofs'] == 288 and r['numeric_observations'] == 576
        assert r['incomplete_cases'] == r['invalid_samples'] == 0
    assert r['source_hashes_sha256'] == EXPECTED
    for name in ['probe.rs', 'reproduce.py']:
        assert sha(E / before / name) == sha(E / after / name)
    assert [contract(r) for r in read(E / before / 'receipts.json')] == [contract(r) for r in read(E / after / 'receipts.json')]
    algebraic_cost = cost('typed-api-prototype-v507-algebraic-cost-baseline', 'typed-api-prototype-v516-algebraic-cost', 'algebraic-budget-cost-comparison-v517.json', 'certified')
    default_cost = cost('typed-api-prototype-v442-evaluation-budget-cost', 'typed-api-prototype-v517-default-evaluation-cost', 'algebraic-budget-default-cost-comparison-v517.json', 'checksums')
    write('algebraic-budget-implementation-v517.json', {
        'status': 'Isolated algebraic work accounting passes focused tests, strict five-crate Clippy, external replay and scoped cost checks; complete checkpoint validation remains pending; no runtime admission',
        'previous_checkpoint': 'typed-api-prototype-v504-state.json', 'baseline_commit': 'ee49f52aa',
        'source_record': source_record, 'source_record_sha256': EXPECTED, 'changed_sources': changed,
        'contract_decision': 'p4-contract-decision-v173.json', 'prior_named_tests_unchanged': True,
        'changes': [
            'Debit shared allowance for algebraic metadata parser visits, generator merging, candidate/base visits and dependency combinations.',
            'Use fallible generator structural comparison; exhaustion propagates as unavailable metadata instead of inequality or independence.',
            'Charge coefficient-size work before dependency-search rational multiplication, division and exact nth-root extraction. Units use squared counts of 64-bit words in the operand numerator/denominator lengths.',
            'Preserve every unlimited proof path, local search limit and prior sign/cache certificate. No new public API or permanent failure cache.'
        ],
        'retained_failures': {'test_compile': 'typed-api-prototype-v508-algebraic-budget-regression', 'compile_correction': 'Explicit BigInt type for the exact scaled perturbation; no assertions changed.', 'accounting_regression': regression, 'failed_tests': 3, 'already_passing_public_contract_test': 1},
        'unit_coverage': {'functions': 4, 'metadata_allowances': 'Every value from zero through measured successful cost plus two; retained root facts, generous retry and zero-budget proof reuse.', 'coefficient_bits': [16, 256, 1024], 'dependency_allowances': ['0', '1', 'needed/4', 'needed-1', 'needed', 'needed+1'], 'structural_depth': 512, 'public_identity_schedules': 180, 'public_degrees': [2, 3, 4], 'public_factor_counts': [2, 3], 'reciprocal_modes': [False, True], 'perturbations': ['-2^-256', '0', '2^-256'], 'public_allowances': [0, 4, 16, 256, 100000]},
        'replay': {'phase': after, 'cases': 288, 'numeric_observations': 576, 'retained_zero_budget_proofs': 288, 'probe_harness_arguments_order_caps_preserved': True, 'zero_answers_before': answers(before), 'zero_answers_after': answers(after)},
        'cost_records': ['algebraic-budget-cost-comparison-v517.json', 'algebraic-budget-default-cost-comparison-v517.json'],
        'observed_median_percent_changes': {'algebraic': algebraic_cost, 'default': default_cost},
        'passing_final_source_validation': passes,
        'limits': ['Cooperative work accounting, not elapsed-time, allocation or CPU-instruction bounds. Rational clones, individual GCD/root durations and other algebraic helpers remain incompletely metered.', 'The 288-case replay warms numeric caches before the explicit allowance. It checks independent exact rational values and retained proofs, not whole-program resource use.', 'Four new tests and scoped micro-cost observations do not establish exhaustive kernel interruption/concurrency coverage or whole-application admission.', 'The new implementation is outside the unchanged conditional Verus target. All thirteen whole-crate proof groups, all 39 properties, all 14 targets and crate future work remain open as previously recorded.'],
        'main_runtime_unchanged': True, 'original_worktree_entries_preserved': 49,
    })
    print(json.dumps({'source': EXPECTED, 'changed_files': len(changed), 'algebraic_median_percent_change': algebraic_cost, 'default_median_percent_change': default_cost, 'answers_after': answers(after)}))


if __name__ == '__main__':
    main()
