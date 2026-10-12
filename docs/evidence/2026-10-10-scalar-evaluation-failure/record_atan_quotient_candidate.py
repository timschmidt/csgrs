"""Freeze the quotient guard and initial costs before its controlled comparison."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
hashes = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    source_record = E/'typed-api-prototype-v1554-atan-quotient-zero-release-build/source-hashes.json.gz'
    expected = 'c789297f8e3463ae9b76b28e48ad3106bb59426c9d96783fcdb57b9d154fe23d'
    assert sha(source_record) == expected
    source = hashes(source_record)
    baseline = read(E/'atan-zero-term-baseline-v1515.json')
    old = hashes(E/baseline['source_record'])
    changed = [n for n in sorted(source.keys() | old.keys()) if source.get(n) != old.get(n)]
    assert changed == ['crates/hyperreal/src/computable/approximation/inverse_trig.rs']
    before = (E/'typed-api-prototype-v1513-atan-baseline-build/inputs'/changed[0]).read_text()
    original = ('        let scaled_1 = signed::ONE.deref() << (-calc_precision);\n'
                '        let big_op_squared: BigInt = i * i;\n'
                '        let inverse: BigInt = scaled_1 / i;\n')
    revised = ('        let scaled_1 = signed::ONE.deref() << (-calc_precision);\n'
               '        let inverse: BigInt = scaled_1 / i;\n'
               '        // A zero leading quotient leaves the sum zero and skips every term.\n'
               '        // Form the denominator square only when a nonzero term needs it.\n'
               '        if inverse.is_zero() {\n'
               '            return Ok(BigInt::zero());\n'
               '        }\n'
               '        let big_op_squared: BigInt = i * i;\n')
    assert before.count(original) == 1 and (SOURCE/changed[0]).read_text() == before.replace(original, revised, 1)
    assert all(sha(SOURCE/n) == h for n, h in source.items())
    main_guard = hashes(E/'candidate-preparation/main-hashes.json.gz')
    assert all(sha(ROOT/n) == h for n, h in main_guard.items())
    focus = read(E/'atan-quotient-zero-focus-v1565.json')
    caps = read(E/'atan-quotient-zero-caps-v1551.json')
    assert focus['source_record_sha256'] == caps['source_record_sha256'] == expected
    assert len(caps['tests']) == 9
    assert all(r['observations_except_allocations_identical'] and r['cap']['exit_code'] == 0 for r in caps['tests'].values())
    units = read(E/focus['test_summary'])
    assert units['top_level_passed'] == 1063 and units['child_process_passed'] == 1
    assert units['top_level_failed'] == units['child_process_failed'] == 0
    cost_run = read(E/'atan-quotient-zero-cost-orchestration-v1561.json')
    assert cost_run['completed'] and cost_run['source_record_sha256'] == expected
    assert sum(r['processes'] for r in cost_run['results']) == 468
    baselines = read(E/'atan-zero-term-paired-cost-v1535/summary.json')['provenance']
    costs = {}
    for row in cost_run['results']:
        phase = row['phase']; family = row['family']
        summary = read(E/phase/'summary.json')
        assert sha(E/phase/'summary.json') == row['summary_sha256']
        assert summary['guards_unchanged'] and summary['source_hashes_sha256'] == expected
        old_phase = baselines[family]['phases']['baseline']
        commands = lambda p: [r for r in read(E/p/'commands.json') if r['name'] not in ['format', 'build']]
        a, b = commands(old_phase), commands(phase)
        assert len(a) == len(b) == row['processes']
        for x, y in zip(a, b):
            assert x['name'] == y['name'] and x['command'][1:] == y['command'][1:]
            lo = read(E/old_phase/(x['name']+'.stdout'))
            hi = read(E/phase/(y['name']+'.stdout'))
            assert {k:v for k,v in lo.items() if k != 'elapsed_ns'} == {k:v for k,v in hi.items() if k != 'elapsed_ns'}
        binary = Path(b[0]['command'][0])
        costs[family] = {'phase': phase, 'baseline_phase': old_phase,
            'binary_sha256': sha(binary), 'binary_bytes': binary.stat().st_size,
            'summary_sha256': sha(E/phase/'summary.json'), 'cases': summary['cases']}
    out = E/'atan-quotient-implementation-v1568.json'
    assert not out.exists()
    out.write_text(json.dumps({
        'status': 'Unretained isolated quotient-guard candidate; controlled comparison pending',
        'baseline_record': 'atan-zero-term-baseline-v1515.json',
        'rejected_variant_record': 'atan-magnitude-guard-rejection-v1541.json',
        'source_record': str(source_record.relative_to(E)), 'source_record_sha256': expected,
        'production_changes_from_frozen_baseline': changed, 'frozen_test_sources_unchanged': True,
        'focus_record': 'atan-quotient-zero-focus-v1565.json', 'caps_record': 'atan-quotient-zero-caps-v1551.json',
        'allocation_comparison_record': 'atan-quotient-allocation-comparison-v1566.json',
        'correctness_observations_preserved': {'survey': 320, 'allocation': 24, 'independent_enclosures': 164},
        'scalar_tests': {'passed': 1063, 'child_passed': 1}, 'strict_clippy_and_format_checks_pass': True,
        'cost_processes': 468, 'complete_outputs_preserved': True, 'costs': costs,
        'comparison_protocol': 'atan-quotient-comparison-protocol-v1567.json',
        'reasoning': 'Compute the existing integer quotient before its independent denominator square. When the quotient is zero, the old sum, power and term are zero, its loop is skipped without any charge/cancellation check, and final scaling returns zero. Nonzero terms retain the same exact arithmetic, with quotient and square evaluated in the opposite order. Existing precision, zero-denominator, endpoint and dispatch checks are unchanged.',
        'limits': ['The quotient guard uses one additional eight-byte allocation in 140 measured rows compared with the rejected magnitude guard; 368 structured rows are identical. The million-bit direct query uses 24 bytes versus the original 4801400.',
                   'Cooperative work units are not a byte/time bound; public rational queries retain input-sized denominator copies.',
                   'All original timings and the rejected variant remain retained. No controlled-comparison outcome, broad-stack or formal-proof rerun is claimed at this record.',
                   'No main runtime admission, push, full-board or whole-plan completion is claimed.']
    }, indent=2)+'\n')
    print(json.dumps({'record': out.name, 'cost_processes': 468, 'outputs_preserved': True}))


if __name__ == '__main__':
    main()
