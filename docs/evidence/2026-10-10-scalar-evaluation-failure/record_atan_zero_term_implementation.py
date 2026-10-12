"""Preserve the tested atan candidate and unresolved paired timing observations."""
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
hashes = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    source_record = E/'typed-api-prototype-v1527-atan-zero-term-release-build/source-hashes.json.gz'
    expected = sha(source_record)
    assert expected == 'f904a928c9c10f8865a31148ab29671bb275a5cd83d6d86c3c7b43844c5303ff'
    source = hashes(source_record)
    baseline = read(E/'atan-zero-term-baseline-v1515.json')
    old = hashes(E/baseline['source_record'])
    changed = [name for name in sorted(source.keys() | old.keys()) if source.get(name) != old.get(name)]
    assert changed == ['crates/hyperreal/src/computable/approximation/inverse_trig.rs']
    added = ('        // A zero leading quotient leaves the sum zero and skips every term.\n'
             '        // Compare borrowed magnitudes before forming the unused wide square.\n'
             '        if scaled_1.magnitude() < i.magnitude() {\n'
             '            return Ok(BigInt::zero());\n'
             '        }\n')
    before = (E/'typed-api-prototype-v1513-atan-baseline-build/inputs'/changed[0]).read_text()
    after = (SOURCE/changed[0]).read_text()
    assert after.count(added) == 1 and after.replace(added, '', 1) == before
    assert all(sha(SOURCE/name) == value for name, value in source.items())
    main_guard = hashes(E/'candidate-preparation/main-hashes.json.gz')
    assert all(sha(ROOT/name) == value for name, value in main_guard.items())
    caps = read(E/'atan-zero-term-caps-v1525.json')
    assert caps['source_record_sha256'] == expected and len(caps['tests']) == 9
    counts = {'survey': 0, 'allocation': 0, 'enclosure': 0}
    allocations = []
    pattern = re.compile(r'Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}')
    for phase, row in caps['tests'].items():
        assert row['observations_except_allocations_identical']
        assert row['cap']['exit_code'] == row['result']['exit_code'] == 0
        assert row['result']['source_unchanged'] and row['result']['main_unchanged']
        old_lines = (E/row['baseline_phase']/'stderr.log').read_text().splitlines()
        new_lines = (E/phase/'stderr.log').read_text().splitlines()
        for label, prefix in [('survey', 'atan numeric survey '), ('allocation', 'atan zero-term '),
                              ('enclosure', 'atan kernel enclosure ')]:
            a = [line for line in old_lines if line.startswith(prefix)]
            b = [line for line in new_lines if line.startswith(prefix)]
            assert [line.split(', allocations=')[0] for line in a] == [line.split(', allocations=')[0] for line in b]
            counts[label] += len(b)
            if label == 'allocation':
                for before_line, after_line in zip(a, b):
                    values = [dict(zip(['calls', 'bytes', 'largest_request'], map(int, pattern.search(line).groups())))
                              for line in [before_line, after_line]]
                    allocations.append({'query': after_line.split(', allocations=')[0],
                                        'baseline': values[0], 'candidate': values[1]})
    assert counts == {'survey': 320, 'allocation': 24, 'enclosure': 164}
    validations = {}
    for phase in ['typed-api-prototype-v1516-atan-zero-term-build',
                  'typed-api-prototype-v1526-atan-zero-term-unit-tests',
                  'typed-api-prototype-v1527-atan-zero-term-release-build',
                  'typed-api-prototype-v1537-atan-zero-term-clippy',
                  'typed-api-prototype-v1538-atan-zero-term-workspace-fmt',
                  'typed-api-prototype-v1539-atan-zero-term-included-fmt']:
        result = read(E/phase/'result.json')
        assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == expected
        validations[phase] = result
    units = read(E/'typed-api-prototype-v1526-test-summary.json')
    assert units['top_level_passed'] == 1063 and units['child_process_passed'] == 1
    assert units['top_level_failed'] == units['child_process_failed'] == 0
    costs = read(E/'atan-zero-term-cost-orchestration-v1534.json')
    assert costs['completed'] and costs['source_record_sha256'] == expected
    assert sum(row['processes'] for row in costs['results']) == 468
    paired = read(E/'atan-zero-term-paired-cost-v1535/summary.json')
    assert paired['source_record_sha256'] == expected and paired['guards_unchanged'] and paired['processes'] == 936
    assert sum(len(rows) for rows in paired['cases'].values()) == 52
    out = E/'atan-zero-term-implementation-v1540.json'
    assert not out.exists()
    out.write_text(json.dumps({
        'status': 'Unretained isolated candidate; allocation and correctness checks pass; paired timing regressions unresolved',
        'baseline_commit': '71c245225', 'baseline_record': 'atan-zero-term-baseline-v1515.json',
        'source_record': str(source_record.relative_to(E)), 'source_record_sha256': expected,
        'production_changes_from_frozen_baseline': changed,
        'frozen_test_sources_unchanged': True, 'observations_except_allocations_identical': counts,
        'allocation_rows': allocations, 'scalar_tests': units, 'validations': validations,
        'strict_clippy': 'Five crates, all targets/features, release, -D warnings',
        'included_rustfmt_files': 107,
        'cost_record': 'atan-zero-term-cost-orchestration-v1534.json',
        'paired_record': 'atan-zero-term-paired-cost-v1535/summary.json',
        'paired_cases': paired['cases'], 'cost_binary_provenance': paired['provenance'],
        'paired_increases_above_five_percent': [dict(family=family, **row)
            for family, rows in paired['cases'].items() for row in rows if row['median_percent_change'] > 5],
        'reasoning': 'The numerator is a positive power of two. For either nonzero denominator sign, strictly larger denominator magnitude makes integer division truncate to zero. The old term, power and sum are therefore all zero, the loop performs no stop/charge check, and scaling zero returns zero. Existing dispatch, precision, denominator-zero, coarse-precision and unit-denominator checks remain before the new guard. Nonzero-term operation order is unchanged.',
        'limits': [
            'The comparison is a scoped optimization, not a new cooperative-budget byte/time contract.',
            'Raw dispatch allocation is constant in the measured coefficient widths; public rational routing still copies an input-sized denominator.',
            'All unpaired and paired trials remain retained, including slower small/cache and integer controls.',
            'The supplemental fixed-CPU/preparation comparison has been prepared but has not run at this record.',
            'No broad-stack/replay/formal-proof rerun is claimed for this candidate at this record.',
            'No main-runtime admission, push, full-board or whole-plan completion is claimed.'
        ]}, indent=2)+'\n')
    print(json.dumps({'record': out.name, 'counts': counts, 'unit_tests': 1063,
        'paired_processes': paired['processes'], 'status': 'unretained pending timing follow-up'}))


if __name__ == '__main__':
    main()
