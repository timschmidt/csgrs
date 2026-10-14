"""Assess every original integer allocation phase; leave runtime admission open."""
import gzip
import json
import re

from common_v1 import E, PARENT, PREP, sha, dump, verify

FIELDS = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']


def records(raw):
    return [json.loads(line) for line in raw.splitlines() if line.startswith(b'{')]


def main():
    destination = E / 'qualification-assessment-v1.json'
    assert not destination.exists()
    out = E / 'qualification-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 11
    assert result['scalar_tests'] == 876 and result['fixed_allocation_processes'] == 6
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    verify(json.loads((PREP / 'protocol.json').read_text()), full=True)
    protocol = json.loads((out / 'protocol.json').read_text())
    assert protocol['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert protocol['review_sha256'] == sha(E / 'source-review-v1.json')
    assert protocol['runner_sha256'] == sha(E / 'qualify_v1.py')
    binaries = json.loads((out / 'binaries.json').read_text())
    assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
    library = json.loads((out / 'library.json').read_text())
    assert sha(library['path']) == library['sha256']
    commands = json.loads((out / 'commands.json').read_text())
    expected_commands = ['clippy', 'build-tests', 'build-library', 'scalar-library', 'link-integer-memory']
    expected_commands += ['allocations-' + str(i) + '-' + label
                          for i, order in enumerate(protocol['orders'], 1) for label in order]
    assert [r['name'] for r in commands] == expected_commands
    assert all(r['exit_code'] == 0 for r in commands)
    raw = {}
    for command in commands:
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[command['name'] + '.' + stream] = gzip.decompress(path.read_bytes())
    assert re.search(rb'test result: ok\. 876 passed; 0 failed; 0 ignored;', raw['scalar-library.stdout'])
    tests = json.loads((out / 'scalar-tests.json').read_text())
    assert tests['original_874_retained'] and tests['P4_tests'] == 11 and tests['persistent_cache_tests'] == 6
    reference = records(gzip.decompress((PARENT / 'library-costs-v1/sample-integer-memory-1-baseline.stdout.gz').read_bytes()))
    identity = [r for r in reference if 'witness' in r or 'fixture' in r]
    phases = [r['name'] for r in reference if 'name' in r]
    samples = json.loads((out / 'samples.json').read_text())
    assert [(s['round'], s['variant']) for s in samples] == [
        (i, label) for i, order in enumerate(protocol['orders'], 1) for label in order]
    for sample in samples:
        rows = records(raw['allocations-' + str(sample['round']) + '-' + sample['variant'] + '.stdout'])
        assert [r for r in rows if 'witness' in r or 'fixture' in r] == identity
        costs = [r for r in rows if 'name' in r]
        assert costs == sample['rows'] and [r['name'] for r in costs] == phases
        assert all(r['iterations'] == 1000 and r['allocation_accounting'] for r in costs)
    comparisons, repeated = [], True
    indexed = {}
    for label in ['main', 'full-cache', 'candidate']:
        own = [s for s in samples if s['variant'] == label]
        assert len(own) == 2
        indexed[label] = [{r['name']: {f: r[f] for f in FIELDS} for r in s['rows']} for s in own]
        repeated &= indexed[label][0] == indexed[label][1]
    for left, right in [('main', 'candidate'), ('full-cache', 'candidate'), ('main', 'full-cache')]:
        for phase in phases:
            comparisons.append({'baseline': left, 'candidate': right, 'phase': phase,
                'fields': {field: {'baseline': [r[phase][field] for r in indexed[left]],
                                   'candidate': [r[phase][field] for r in indexed[right]],
                                   'paired_deltas': [b[phase][field] - a[phase][field]
                                       for a, b in zip(indexed[left], indexed[right], strict=True)]}
                           for field in FIELDS}})
    assert len(comparisons) == 36
    dump(E / 'integer-allocation-comparisons-v1.json', comparisons)
    old = json.loads((PARENT / 'library-assessment-v1.json').read_text())
    historical_match = all(
        all(value == current[row['name']][field] for value in row['fields'][field][old_label])
        for label, old_label in [('main', 'baseline'), ('full-cache', 'candidate')]
        for current in indexed[label] for row in old['memory_rows'] if row['suite'] == 'integer' for field in FIELDS)
    signed = [r for r in comparisons if r['phase'] == 'signed_word_boundary_construction']
    main_pair = next(r for r in signed if r['baseline'] == 'main' and r['candidate'] == 'candidate')
    parent_pair = next(r for r in signed if r['baseline'] == 'full-cache')
    repaired = all(all(delta == 0 for delta in row['paired_deltas']) for row in main_pair['fields'].values())
    repaired &= parent_pair['fields']['allocated_bytes']['paired_deltas'] == [-12024, -12024]
    repaired &= parent_pair['fields']['peak_bytes_delta']['paired_deltas'] == [-24, -24]
    increases = [r for r in comparisons if r['candidate'] == 'candidate'
                 and any(any(d > 0 for d in f['paired_deltas']) for f in r['fields'].values())]
    assessment = {
        'status': 'signed-word-fixture-allocation-repaired-broader-costs-pending' if repaired and repeated and historical_match
                  else 'integer-allocation-qualification-needs-review',
        'assessor_sha256': sha(__file__), 'result_sha256': sha(out / 'result.json'),
        'comparisons_sha256': sha(E / 'integer-allocation-comparisons-v1.json'),
        'commands_passed': 11, 'scalar_tests_passed': 876, 'P4_tests_passed': 11,
        'persistent_cache_tests_passed': 6, 'new_tests_passed': 2,
        'fixed_allocation_processes': 6, 'allocation_rows': 72, 'comparison_rows': 36,
        'all_exact_witnesses_and_representation_inventories_match': True,
        'all_allocation_rows_repeat_within_variant': repeated,
        'unchanged_main_and_full_cache_allocations_match_closed_history': historical_match,
        'signed_word_construction_allocation_restored_to_main': repaired,
        'signed_word_construction_comparisons': signed,
        'all_candidate_memory_increase_rows': increases,
        'native_gates_rerun': False, 'full_library_cost_matrix_pending': True,
        'limits': ['All original integer phases are included, but rational/quadratic/arc/metadata/broad cost matrices have not been measured for this new candidate.',
                   'The unchanged allocation probe prints durations. Those values are retained as raw output and are not a timing comparison.',
                   'Earlier construction regressions, cache retention costs, broad-operation costs and native deadline failures remain open. No runtime source is admitted into main.'],
        'runtime_candidate': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    dump(destination, assessment)
    print(json.dumps({k: assessment[k] for k in ['status', 'scalar_tests_passed', 'fixed_allocation_processes',
        'all_allocation_rows_repeat_within_variant', 'signed_word_construction_allocation_restored_to_main',
        'full_library_cost_matrix_pending', 'runtime_admitted']}))


if __name__ == '__main__':
    main()
