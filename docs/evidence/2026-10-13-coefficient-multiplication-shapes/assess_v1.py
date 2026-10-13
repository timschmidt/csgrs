"""Verify the complete operand-shape observation and summarize actual coverage."""
import gzip
import hashlib
import json
from pathlib import Path
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
    assert result['complete'] and len(rows) == 16
    assert sha(E / 'diagnose_v1.py') == protocol['runner_sha256']
    assert sha(E / 'coefficient_shape_diagnostic.rs') == protocol['helper_input_sha256']
    assert result['tests'] == [
        {'name': 'counter-calibration', 'passed': 2, 'failed': 0, 'ignored': 0},
        {'name': 'cache-frontiers', 'passed': 6, 'failed': 0, 'ignored': 0},
        {'name': 'endpoint-regressions', 'passed': 5, 'failed': 0, 'ignored': 0},
    ]
    for command in json.loads((OUT / 'commands.json').read_text()):
        assert command['exit_code'] == (result['full_board_exit_code'] if command['name'] == 'full-board' else 0)
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (command['name'] + '.' + stream)) == command[stream + '_sha256']
    expected = json.loads(gzip.decompress((OUT / 'parent-source-hashes.json.gz').read_bytes()))
    assert all(sha(Path(protocol['parent_source']) / name) == digest for name, digest in expected.items())
    assert all(sha(Path(protocol['source']) / name) == digest
               for name, digest in (expected | protocol['source_changes_sha256']).items())
    assert sources() == protocol['main_source_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in protocol['pending_native_sha256'].items())
    binary = json.loads((OUT / 'binary.json').read_text())
    assert sha(Path(binary['path'])) == binary['sha256']
    assert Path(binary['manifest_path']) == Path(protocol['source']) / 'crates/hyperdrc/Cargo.toml'
    assert Path(binary['path']).is_relative_to(Path(protocol['target']))
    total = sum(row['calls'] for row in rows)
    assert total == result['coefficient_multiplications'] and total > 0
    for row in rows:
        assert all(row[field] >= 0 for field in protocol['row_fields'])
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
    for row in rows:
        row['percent_of_calls'] = 100 * row['calls'] / total
    right_scale = [row for row in rows if row['right'] == 'rational']
    reusable = [row for row in right_scale if row['left'] != 'rational']
    other = [row for row in rows if row['right'] != 'rational']
    scale_calls = sum(row['calls'] for row in right_scale)
    reusable_calls = sum(row['calls'] for row in reusable)
    assessment = {
        'decision': 'complete-original-full-board-coefficient-shape-observation',
        'assessor_sha256': sha(Path(__file__)), 'tests': result['tests'],
        'strict_all_feature_all_target_clippy_and_format_passed': True,
        'original_worker_cpu_limit_seconds': result['original_worker_cpu_limit_seconds'],
        'instrumented_worker_cpu_seconds': result['instrumented_worker_cpu_seconds'],
        'original_assertion_passed': result['full_board_exit_code'] == 0,
        'both_reports_emitted': result['both_reports_emitted'], 'finding_counts': result['finding_counts'],
        'total_multiplication_calls': total, 'rows': rows,
        'right_rational_calls': scale_calls, 'right_rational_percent': 100 * scale_calls / total,
        'right_rational_with_owned_left_vectors_calls': reusable_calls,
        'right_rational_with_owned_left_vectors_percent': 100 * reusable_calls / total,
        'right_rational_with_owned_left_radical_elements': sum(row['left_radical_length_sum'] for row in reusable),
        'right_rational_with_owned_left_atom_elements': sum(row['left_atom_length_sum'] for row in reusable),
        'other_right_shapes_calls': sum(row['calls'] for row in other),
        'source_review': 'Only a fixed-counter record call is inserted at the original multiply entry; its arguments are four existing vector lengths. The original function body, unsupported-product guard, scalar operations, vector insertion order, normalizer passes and cache publication are unchanged. The report guard is created after the geometry-test lock and prints after the original timer assertion.',
        'limits': [
            protocol['shape_definition'], protocol['limits'],
            'A call with existing left vectors is potential reuse coverage, not proof of saved allocations or bytes. No rational coefficient value or zero/one scale distribution was queried.',
            'The full-board report counts are diagnostic checks; the prior complete byte-identical report replay remains a separate artifact.',
            'The original failed release gate and all fixed endpoint timing/memory tradeoffs remain unresolved. A diagnostic does not admit a runtime change.',
        ],
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'source_parent_and_main_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({key: assessment[key] for key in ['total_multiplication_calls', 'right_rational_calls',
        'right_rational_percent', 'right_rational_with_owned_left_vectors_calls',
        'right_rational_with_owned_left_vectors_percent', 'instrumented_worker_cpu_seconds']}))


if __name__ == '__main__':
    main()
