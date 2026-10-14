"""Check diagnostic counts independently and keep elapsed-cost causes open."""
import gzip
import json
from pathlib import Path

from common_v1 import E, PREP, sha, dump, verify


def records(data):
    return [json.loads(line) for line in data.splitlines() if line.startswith(b'{')]


def flatten(value, prefix=''):
    result = {}
    for key, item in value.items():
        if isinstance(item, dict):
            result.update(flatten(item, prefix + key + '.'))
        else:
            result[prefix + key] = item
    return result


def main():
    destination = E / 'construction-trace-assessment-v1.json'
    assert not destination.exists()
    out = E / 'construction-trace-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 9
    assert result['trace_rows'] == result['comparison_rows'] == 192
    assert result['new_library_builds'] == 0
    assert all(sha(out / p) == digest for p, digest in result['artifact_sha256'].items())
    protocol = json.loads((out / 'protocol.json').read_text())
    assert all(sha(p) == digest for p, digest in protocol['inputs_sha256'].items())
    verify(json.loads((PREP / 'protocol.json').read_text()), full=True)
    original = [Path(p) for p in protocol['inputs_sha256'] if p.endswith('/probes-v1/quadratic_probe.rs')]
    assert len(original) == 1
    old_text, new_text = original[0].read_text(), (out / 'probe.rs').read_text()
    marker = 'fn inputs(index: usize)'
    assert old_text[old_text.index(marker):] == new_text[new_text.index(marker):]
    binaries = json.loads((out / 'binaries.json').read_text())
    assert set(binaries) == {'main', 'full-cache', 'no-persistent'}
    assert len({r['sha256'] for r in binaries.values()}) == 3
    assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
    assert all(sha(r['path']) == r['sha256'] for group in protocol['libraries'].values() for r in group.values())
    commands = json.loads((out / 'commands.json').read_text())
    assert len(commands) == 9 and all(r['exit_code'] == 0 for r in commands)
    assert [r['name'] for r in commands[:3]] == ['build-main', 'build-full-cache', 'build-no-persistent']
    assert [r['name'] for r in commands[3:]] == ['trace-' + str(i) + '-' + label for i, label in protocol['schedule']]
    samples = json.loads((out / 'samples.json').read_text())
    witnesses = []
    for command, sample in zip(commands[3:], samples, strict=True):
        label = sample['variant']
        assert command['command'] == ['taskset', '-c', '0', binaries[label]['path'], '4096']
        raw = {}
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[stream] = gzip.decompress(path.read_bytes())
        values = records(raw['stdout'])
        assert sample['trace'] == [r for r in values if 'trace_phase' in r]
        assert len(sample['trace']) == 32
        assert all(r['iterations'] == 4096 for r in sample['trace'])
        witnesses.append([r for r in values if 'witness' in r or 'fixture' in r])
    assert all(value == witnesses[0] for value in witnesses)
    assert len(witnesses[0]) == 32
    historical = [Path(p) for p in protocol['inputs_sha256']
                  if '/sample-quadratic-time-' in p and p.endswith('.stdout.gz')]
    assert len(historical) == 20
    for path in historical:
        assert [r for r in records(gzip.decompress(path.read_bytes())) if 'witness' in r or 'fixture' in r] == witnesses[0]
    for label in binaries:
        own = [r['trace'] for r in samples if r['variant'] == label]
        assert len(own) == 2 and own[0] == own[1]
    comparisons = json.loads((out / 'comparisons.json').read_text())
    expected_keys = {(i, left, right, r['trace_phase']) for i in [1, 2]
                     for left, right in [('main', 'full-cache'), ('main', 'no-persistent'), ('no-persistent', 'full-cache')]
                     for r in samples[0]['trace']}
    assert {(r['repetition'], r['baseline'], r['candidate'], r['phase']) for r in comparisons} == expected_keys
    assert len(comparisons) == len(expected_keys) == 192
    for row in comparisons:
        pair = {label: next(t for s in samples if s['variant'] == label and s['repetition'] == row['repetition']
                           for t in s['trace'] if t['trace_phase'] == row['phase'])
                for label in [row['baseline'], row['candidate']]}
        left, right = pair[row['baseline']], pair[row['candidate']]
        a, b = flatten(left['rational']), flatten(right['rational'])
        assert row['rational_candidate_minus_baseline'] == {k: b[k] - a[k] for k in a}
        a, b = ({(r['layer'], r['operation'], r['path']): r['count'] for r in value['dispatch']}
                for value in [left, right])
        assert row['dispatch_candidate_minus_baseline'] == [
            {'layer': k[0], 'operation': k[1], 'path': k[2], 'count_delta': b.get(k, 0) - a.get(k, 0)}
            for k in sorted(set(a) | set(b))]
        assert row['all_recorded_counters_equal'] == (left == right)
    construction = [r for r in comparisons if r['phase'].endswith('_construction')]
    assert len(construction) == 48 and all(r['all_recorded_counters_equal'] for r in construction)
    cache_query = [r for r in comparisons if r['baseline'] == 'no-persistent' and r['candidate'] == 'full-cache'
                   and r['repetition'] == 1 and not r['all_recorded_counters_equal']]
    assert len(cache_query) == 4
    assessment = {'status': 'construction-arithmetic-counts-equal-elapsed-cost-attribution-open',
                  'assessor_sha256': sha(__file__), 'result_sha256': sha(out / 'result.json'),
                  'new_library_builds': 0, 'diagnostic_builds': 3, 'diagnostic_processes': 6,
                  'inputs_each_process': 4096, 'trace_rows': 192, 'comparison_rows': 192,
                  'all_counts_repeat_exactly_within_variant': True,
                  'all_eight_construction_phases_equal_across_three_variants_and_two_runs': True,
                  'all_exact_witnesses_and_representations_match_twenty_closed_cost_processes': True,
                  'nonidentical_cache_vs_no_cache_query_rows_one_repetition': cache_query,
                  'interpretation': [
                      'No extra recorded rational temporary, reduction, GCD or dispatch event occurs in any construction phase.',
                      'This includes both earlier large construction regressions: negated-scaled surds and multiradical squares.',
                      'The retained-cache build reduces recorded work in four warm/cloned query phases; every other recorded phase matches the cache-free build.',
                      'These diagnostics do not measure time or requested allocation bytes. The public counters do not count every cache lookup, payload copy, lock or allocator action.',
                      'Equal recorded work does not establish equal runtime. Neither cache handling, code generation/layout nor environment effects have been established as the cause of the closed timing differences.'
                  ],
                  'next_step': 'Use a separate instrumented snapshot to establish whether retained-cache lookup, payload copying/publication and evaluation-cache allocation execute in the expensive construction windows. Preserve all eight families and first/warm/cloned controls; compare witnesses and existing counters with this frozen full-cache driver before drawing conclusions. No runtime candidate is selected yet.',
                  'closed_cost_samples_extended_or_discarded': False,
                  'native_failure_and_library_costs_remain_open': True,
                  'runtime_admitted': False, 'plan_complete': False}
    dump(destination, assessment)
    print(json.dumps({k: v for k, v in assessment.items() if k not in [
        'nonidentical_cache_vs_no_cache_query_rows_one_repetition', 'interpretation', 'next_step']}))


if __name__ == '__main__':
    main()
