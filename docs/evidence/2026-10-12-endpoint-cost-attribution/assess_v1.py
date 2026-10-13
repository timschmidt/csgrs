"""Check all diagnostic counters and compare windows without using their times."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
OUT = E / 'observation-v1'
ROOT = E.parents[2]
TRIAL = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
FIELDS = ['entries', 'allocations', 'allocated_bytes', 'deallocations', 'deallocated_bytes']
SPANS = ['probe', 'endpoint', 'coordinate_compare', 'distance_squared', 'policy_point_equal',
         'is_zero', 'normal_form', 'normal_form_recursion', 'arithmetic_reuse',
         'cache_initialize', 'cache_store_laurent', 'cache_load_laurent', 'node_new',
         'rational_linear_retention', 'rational_product_retention', 'rational_negation_retention']
EVENTS = ['normal_form_reused', 'normal_form_fallback', 'cache_created',
          'cache_published_laurent', 'arithmetic_reuse_ready', 'arithmetic_reuse_first']


def read_windows(path):
    windows = {}
    for line in path.read_text().splitlines():
        kind, label, stage, key, *values = line.split('\t')
        window = windows.setdefault((label, stage), {'spans': {}, 'events': {}})
        numbers = [int(value) for value in values]
        assert all(value >= 0 for value in numbers)
        if kind == 'ENDPOINT_COST':
            assert key in SPANS and key not in window['spans'] and len(numbers) == 5
            window['spans'][key] = dict(zip(FIELDS, numbers))
        else:
            assert kind == 'ENDPOINT_EVENT'
            assert key in EVENTS and key not in window['events'] and len(numbers) == 1
            window['events'][key] = numbers[0]
    assert windows
    for window in windows.values():
        spans, events = window['spans'], window['events']
        assert set(spans) == set(SPANS) and set(events) == set(EVENTS)
        assert spans['probe']['entries'] == 1
        for row in spans.values():
            assert all(row[field] <= spans['probe'][field] for field in FIELDS[1:])
        assert events['cache_created'] <= spans['cache_initialize']['entries']
        assert events['cache_published_laurent'] <= spans['cache_store_laurent']['entries']
        assert events['normal_form_reused'] <= spans['normal_form_recursion']['entries']
        assert events['normal_form_fallback'] <= spans['normal_form']['entries']
        assert events['arithmetic_reuse_ready'] + events['arithmetic_reuse_first'] == spans['arithmetic_reuse']['entries']
    return windows


def main():
    assert not (E / 'assessment-v1.json').exists()
    result = json.loads((OUT / 'result.json').read_text())
    assert result['complete'] and result['all_original_witnesses_match']
    commands = json.loads((OUT / 'commands.json').read_text())
    for row in commands:
        assert row['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    protocol = json.loads((OUT / 'protocol.json').read_text())
    assert sha(E / 'observe_v1.py') == protocol['runner_sha256']
    assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
    assert sha(TRIAL / 'controls-v1/trials.json') == protocol['prior_trials_sha256']
    prior = json.loads((TRIAL / 'controls-v1/trials.json').read_text())
    observations = json.loads((OUT / 'observations.json').read_text())
    assert len(observations) == 4
    windows = {}
    allocation_replay = []
    for observation in observations:
        mode, variant = observation['mode'], observation['variant']
        key = (mode, variant)
        assert key not in windows
        current = read_windows(OUT / f'observe-{variant}-{mode}.stderr')
        windows[key] = current
        if mode == 'arc':
            measures = [row for row in observation['rows'] if 'name' in row]
            assert len(current) == len(measures) * 2 + 1 == 97
            for row in measures:
                sample = current[(row['name'], 'measured')]['spans']['probe']
                assert sample['allocations'] == row['allocation_calls']
                assert sample['allocated_bytes'] == row['allocated_bytes']
                assert sample['allocated_bytes'] - sample['deallocated_bytes'] == row['retained_bytes_delta']
            identity = [{k: v for k, v in row.items() if k != 'nanos'} for row in measures]
            for original in prior:
                if original['mode'] != 'memory' or original['variant'] != variant:
                    continue
                old = [{k: v for k, v in row.items() if k != 'nanos'}
                       for row in original['rows'] if 'name' in row]
                differences = [{'original': a, 'diagnostic': b}
                               for a, b in zip(old, identity) if a != b]
                allocation_replay.append({'variant': variant, 'original_pair': original['index'],
                                          'differences': differences})
    comparisons = {}
    for mode in ['arc', 'broad']:
        baseline, candidate = windows[(mode, 'baseline')], windows[(mode, 'candidate')]
        assert baseline.keys() == candidate.keys()
        rows = []
        for key in baseline:
            a, b = baseline[key], candidate[key]
            delta = {'spans': {span: {field: b['spans'][span][field] - a['spans'][span][field]
                                     for field in FIELDS} for span in SPANS},
                     'events': {event: b['events'][event] - a['events'][event] for event in EVENTS}}
            rows.append({'label': key[0], 'stage': key[1], 'baseline': a, 'candidate': b,
                         'candidate_minus_baseline': delta, 'all_counters_identical': a == b})
        comparisons[mode] = rows
    compressed = E / 'all-window-comparisons-v1.json.gz'
    compressed.write_bytes(gzip.compress((json.dumps(comparisons, indent=2) + '\n').encode(), mtime=0))
    arc_labels = ['opaque_translated_non_endpoint_construction', 'off_circle_warm', 'aborted_unknown_first',
                  'approximate_non_endpoint_first', 'near_endpoint_off_circle_warm']
    broad_labels = ['mixed_quadratic_aggregation_and_bounds', 'warm_polygon_area_17',
                    'rectangle_construct_unit', 'stencil_sparse_1002_pads',
                    'ipc_drill_diameter', 'ipc_annotation_invalid_diameter',
                    'point_return_huge_radius', 'fixture_copper_huge_radius',
                    'pad_round_warm_area_samples_128']
    # Require each intended selected row to exist; never substitute a zero-match filter.
    for label in arc_labels:
        assert any(row['label'] == label and row['stage'] == 'measured' for row in comparisons['arc']), label
    for label in broad_labels:
        assert any(row['label'] == label and row['stage'] == 'measured' for row in comparisons['broad']), label
    selected = {mode: [row for row in comparisons[mode] if row['label'] in labels]
                for mode, labels in [('arc', arc_labels), ('broad', broad_labels)]}
    assessment = {
        'decision': 'integer-counter-observations-complete-runtime-admission-still-open',
        'assessor_sha256': sha(Path(__file__)), 'observation_result': result,
        'window_counts': {mode: len(rows) for mode, rows in comparisons.items()},
        'windows_with_different_counters': {
            mode: [{'label': row['label'], 'stage': row['stage']} for row in rows
                   if not row['all_counters_identical']] for mode, rows in comparisons.items()},
        'all_arc_measurement_totals_match_existing_allocator': True,
        'arc_allocation_replay_against_all_five_original_pairs': allocation_replay,
        'selected_windows': selected,
        'artifacts_sha256': {
            str(path.relative_to(ROOT)): sha(path) for path in
            [OUT / 'protocol.json', OUT / 'result.json', OUT / 'commands.json',
             OUT / 'observations.json', OUT / 'libraries.json', OUT / 'binaries.json', compressed]},
        'limits': [
            'Rows are inclusive and overlap; their allocation totals cannot be added across spans.',
            'Only the original query sequence and instrumented executions are observed; equal counters are not a universal cache-semantics proof.',
            'Zero endpoint calls in a window excludes direct execution of that changed path in that window, not prior cache effects or compiler/layout costs.',
            'No instrumented duration is an added benchmark sample; all original adverse five-pair timing and phase-memory rows remain.',
            'Rust allocation requests exclude native C heaps; node_new entries exclude the enclosing Arc allocation.',
            'Scopes are thread-local and are not propagated to worker threads; these rows describe work on the active probe thread.',
        ],
        'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'windows': assessment['window_counts'],
                      'different_windows': {mode: len(rows) for mode, rows in assessment['windows_with_different_counters'].items()},
                      'original_memory_pairs_with_differences': sum(bool(row['differences']) for row in allocation_replay)}))


if __name__ == '__main__':
    main()
