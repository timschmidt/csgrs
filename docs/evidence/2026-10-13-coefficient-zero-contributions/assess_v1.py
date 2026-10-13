"""Verify executed zero-contribution coverage without inferring saved runtime."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'diagnostic-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    assert not (E / 'assessment-v1.json').exists()
    protocol = json.loads((OUT / 'protocol.json').read_text())
    result = json.loads((OUT / 'result.json').read_text())
    rows = json.loads((OUT / 'rows.json').read_text())
    linear = json.loads((OUT / 'linear-rows.json').read_text())
    assert result['complete'] and len(rows) == 64 and len(linear) == 4
    for name, field in [
        ('diagnose_v1.py', 'runner_sha256'), ('assess_v1.py', 'assessor_sha256'),
        ('coefficient_zero_diagnostic.rs', 'helper_input_sha256'),
        ('coefficient_zero_site_tests.rs', 'site_tests_input_sha256'),
    ]:
        assert sha(E / name) == protocol[field]
    assert sha(ROOT / 'docs/evidence/2026-10-13-coefficient-scaling/decision-v1.json') == protocol['prior_scaling_rejection_sha256']
    assert result['tests'] == [
        {'name': 'counter-calibration', 'passed': 4, 'failed': 0, 'ignored': 0},
        {'name': 'cache-frontiers', 'passed': 6, 'failed': 0, 'ignored': 0},
        {'name': 'endpoint-regressions', 'passed': 5, 'failed': 0, 'ignored': 0},
    ]
    commands = json.loads((OUT / 'commands.json').read_text())
    assert [row['name'] for row in commands] == [
        'format', 'counter-calibration', 'cache-frontiers', 'endpoint-regressions',
        'strict-clippy', 'build-board', 'full-board',
    ]
    for command in commands:
        assert command['exit_code'] == (result['full_board_exit_code'] if command['name'] == 'full-board' else 0)
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (command['name'] + '.' + stream)) == command[stream + '_sha256']
    expected = json.loads(gzip.decompress((OUT / 'parent-source-hashes.json.gz').read_bytes()))
    assert len(expected) == protocol['parent_source_count'] == 10130
    assert all(sha(Path(protocol['parent_source']) / name) == digest for name, digest in expected.items())
    assert all(sha(Path(protocol['source']) / name) == digest
               for name, digest in (expected | protocol['source_changes_sha256']).items())
    assert sources() == protocol['main_source_sha256']
    assert len(protocol['pending_native_sha256']) == 35
    assert all(sha(ROOT / name) == digest for name, digest in protocol['pending_native_sha256'].items())
    binary = json.loads((OUT / 'binary.json').read_text())
    assert sha(Path(binary['path'])) == binary['sha256']
    assert Path(binary['manifest_path']) == Path(protocol['source']) / 'crates/hyperdrc/Cargo.toml'
    assert Path(binary['path']).is_relative_to(Path(protocol['target']))
    assert binary['target']['kind'] == ['lib'] and binary['profile']['test']

    raw = (OUT / 'full-board.stdout').read_text() + (OUT / 'full-board.stderr').read_text()
    assert 'skipping complex project fixture test' not in raw
    assert raw.count('hyperdrc: finished emit report') == 2
    assert '12 active finding(s)' in raw and '197 active finding(s)' in raw
    assert protocol['original_worker_cpu_limit_seconds'] == result['original_worker_cpu_limit_seconds'] == 120
    if result['full_board_exit_code'] == 101:
        elapsed = re.search(r'took ([0-9.]+)s of worker time', raw)
        assert elapsed and float(elapsed[1]) == result['instrumented_worker_cpu_seconds'] >= 120
        assert '0 passed; 1 failed' in raw
    else:
        assert result['full_board_exit_code'] == 0 and '1 passed; 0 failed' in raw
    parsed, parsed_linear = [], []
    for line in raw.splitlines():
        if line.startswith('COEFFICIENT_ZERO_SHAPE\t'):
            _, left, right, left_zero, right_zero, *values = line.split('\t')
            assert len(values) == 5 and left_zero in ['0', '1'] and right_zero in ['0', '1']
            parsed.append({'left': left, 'right': right, 'left_zero': bool(int(left_zero)),
                           'right_zero': bool(int(right_zero)),
                           **dict(zip(protocol['row_fields'], map(int, values)))})
        elif line.startswith('COEFFICIENT_ZERO_TERM\t'):
            _, term, entries, zero = line.split('\t')
            parsed_linear.append({'term': term, 'entries': int(entries), 'zero_scale_entries': int(zero)})
    assert parsed == rows and parsed_linear == linear == result['linear_rows']
    assert {(row['left'], row['right'], row['left_zero'], row['right_zero']) for row in rows} == {
        (a, b, x, y) for a in protocol['shapes'] for b in protocol['shapes'] for x in [False, True] for y in [False, True]}
    assert [row['term'] for row in linear] == protocol['linear_terms']
    total = sum(row['calls'] for row in rows)
    assert total == result['eligible_multiplication_calls'] and total > 0
    eligible_pairs = {
        ('rational', 'rational'), ('rational', 'radical'), ('rational', 'opaque'), ('rational', 'mixed'),
        ('radical', 'rational'), ('radical', 'radical'), ('opaque', 'rational'), ('mixed', 'rational'),
    }
    for row in rows:
        assert all(row[field] >= 0 for field in protocol['row_fields'])
        if (row['left'], row['right']) not in eligible_pairs:
            assert all(row[field] == 0 for field in protocol['row_fields'])
        for side in ['left', 'right']:
            radical = row[side + '_radical_length_sum']
            opaque = row[side + '_atom_length_sum']
            assert radical <= 8 * row['calls']
            if row['calls'] == 0:
                assert radical == opaque == 0
                continue
            assert (radical > 0) == (row[side] in ['radical', 'mixed'])
            assert (opaque > 0) == (row[side] in ['opaque', 'mixed'])
            assert radical == 0 or radical >= row['calls']
            assert opaque == 0 or opaque >= row['calls']
    for row in linear:
        term = row['term']
        other_zero = 'right_zero' if term.startswith('left_') else 'left_zero'
        potential = sum(shape[term + '_length_sum'] for shape in rows)
        potential_zero = sum(shape[term + '_length_sum'] for shape in rows if shape[other_zero])
        assert 0 <= row['zero_scale_entries'] <= row['entries'] <= potential
        assert row['zero_scale_entries'] <= potential_zero
        assert row['entries'] - row['zero_scale_entries'] <= potential - potential_zero
        row['input_length_sum'] = potential
        row['input_zero_scale_length_sum'] = potential_zero
        row['entries_not_reached'] = potential - row['entries']
        row['zero_scale_entries_not_reached'] = potential_zero - row['zero_scale_entries']
        row['zero_percent_of_executed_entries'] = 100 * row['zero_scale_entries'] / row['entries'] if row['entries'] else None
    entries = sum(row['entries'] for row in linear)
    zero = sum(row['zero_scale_entries'] for row in linear)
    assert entries > 0
    screen = zero >= 100000 and zero * 20 >= entries
    assessment = {
        'decision': 'complete-original-full-board-zero-contribution-observation',
        'assessor_sha256': sha(Path(__file__)), 'tests': result['tests'],
        'strict_all_feature_all_target_clippy_and_format_passed': True,
        'original_worker_cpu_limit_seconds': 120,
        'instrumented_worker_cpu_seconds': result['instrumented_worker_cpu_seconds'],
        'original_assertion_passed': result['full_board_exit_code'] == 0,
        'both_reports_emitted': True, 'finding_counts': [12, 197],
        'eligible_multiplication_calls': total, 'shape_zero_rows': rows,
        'executed_linear_entries': entries, 'executed_zero_scale_entries': zero,
        'zero_percent_of_executed_linear_entries': 100 * zero / entries, 'linear_rows': linear,
        'coverage_screen': {'minimum_executed_zero_entries': 100000, 'minimum_percent': 5,
                            'passed': screen, 'meaning': 'Permission to investigate a separate isolated runtime candidate, never runtime admission or a speedup prediction.'},
        'source_review': 'The original arithmetic and early-return body is recovered after removing only five fixed-counter calls and the private test-module declaration. Zero flags read stored rational signs. Linear counters are inside the four original loop bodies. Reporting occurs after the original assertion, including unwind, with no measured workers still running.',
        'limits': [
            protocol['shape_definition'], protocol['limits'],
            'Input-length sums are upper bounds on loop entries. Early insertion failure may make them differ. Executed zero counts come from the sites, not those sums.',
            'Rational zero multiplication already returns before canonicalization and product-cache work. These counts do not imply saved allocations or expensive arithmetic.',
            'Report finding counts are not complete report-byte identity. The endpoint parent remains unadmitted, and its original timing and memory tradeoffs remain open.',
        ],
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'source_parent_and_main_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({key: assessment[key] for key in [
        'eligible_multiplication_calls', 'executed_linear_entries', 'executed_zero_scale_entries',
        'zero_percent_of_executed_linear_entries', 'coverage_screen', 'instrumented_worker_cpu_seconds',
    ]}))


if __name__ == '__main__':
    main()
