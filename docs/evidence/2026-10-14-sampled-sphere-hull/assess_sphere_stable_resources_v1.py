"""Verify a closed allocation trial and report every resource observation."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-stable-selection-resources-v1'
CASES = ['fifteen_degrees', 'original_medium', 'fifth_turn', 'seventh_turn']
PHASES = ['construct', 'cold_hull', 'warm_hull', 'replay_hull',
          'drop_outputs_and_input', 'clear_grammar_cache']
METRICS = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def describe(values):
    return {'observations': values, 'minimum': min(values), 'maximum': max(values),
            'median': statistics.median(values), 'all_identical': len(set(values)) == 1}


def main():
    destination = E / 'sphere-stable-selection-resources-assessment-v1.json'
    assert not destination.exists()
    result = json.loads((OUT / 'result.json').read_text())
    assert result['complete'] and result['commands_all_zero'] == 34
    assert result['exact_output_transcripts_rechecked'] == 81
    assert result['all_frozen_sources_inputs_and_libraries_unchanged']
    assert all(sha(OUT / name) == digest for name, digest in result['artifacts_sha256'].items())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    assert all(sha(row['path']) == row['sha256'] for libs in protocol['libraries'].values() for row in libs.values())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
    root_mapping = json.loads(gzip.decompress((OUT / 'root-source-hashes.json.gz').read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
    for group, preparation in [('baseline', 'mesh-identity-preparation-v1'),
                               ('image', 'sphere-cyclotomic-image-v1'), ('stable', 'sphere-stable-selection-v1')]:
        mapping_path = E / preparation / 'source-hashes.json.gz'
        assert sha(mapping_path) == protocol['source_mappings_sha256'][group]
        mapping = json.loads(gzip.decompress(mapping_path.read_bytes()))
        assert all(sha(Path(protocol['sources'][group]) / name) == digest for name, digest in mapping.items())
    commands = json.loads((OUT / 'commands.json').read_text())
    assert len(commands) == 34 and all(row['exit_code'] == 0 for row in commands)
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    samples = json.loads((OUT / 'samples.json').read_text())
    orders = [['baseline', 'image', 'stable'], ['image', 'stable', 'baseline'], ['stable', 'baseline', 'image']]
    schedule = [(number, position, group, case) for number, order in enumerate(orders, 1)
                for position, group in enumerate(order, 1)
                for case in (CASES[:1] if group == 'baseline' else CASES)]
    assert [(s['round'], s['position'], s['group'], s['case']) for s in samples] == schedule
    assert [r['name'] for r in commands[7:]] == [s['name'] for s in samples]
    for sample in samples:
        raw = (OUT / (sample['name'] + '.stdout')).read_text()
        objects = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
        rows = [row for row in objects if 'phase' in row]
        assert rows == sample['rows'] and [row['phase'] for row in rows] == PHASES
        assert objects[-1] == sample['lifecycle']
        group, case = sample['group'], sample['case']
        reference = E / protocol['reference_transcripts'][group][case]
        assert sha(reference) == protocol['reference_output_sha256'][group][case]
        reference_raw = reference.read_text()
        assert reference_raw.splitlines()[0] == f'replay_case={case} repetitions=3 grammar_policy=default allocator=default'
        reference_parts = reference_raw.split('witness_phase=')[1:]
        assert len(reference_parts) == 3
        expected = {}
        for part, phase in zip(reference_parts, ['cold', 'warm', 'replay']):
            lines = part.splitlines()
            assert len(lines) == 5 and lines[0] == phase
            expected[phase] = '\n'.join(lines[1:]) + '\n'
            assert hashlib.sha256(expected[phase].encode()).hexdigest() == protocol['phase_witness_sha256'][group][case][phase]
            assert len(expected[phase].encode()) == protocol['phase_witness_bytes'][group][case][phase]
        assert sample['all_three_exact_outputs_match_group_phase_reference']
        assert sample['all_three_exact_outputs_identical'] == (len(set(expected.values())) == 1)
        parts = raw.split('witness_phase=')[1:]
        assert len(parts) == 3
        for part, phase in zip(parts, ['cold', 'warm', 'replay']):
            lines = part.splitlines()
            assert lines[0] == phase
            witness = '\n'.join(line for line in lines[1:] if not line.startswith('{')) + '\n'
            assert witness == expected[phase]
        for row in rows:
            assert row['allocation_calls'] >= 0 and row['allocated_bytes'] >= 0
            assert row['peak_bytes_delta'] >= max(0, row['retained_bytes_delta'])
    by_group = {}
    for group in ['baseline', 'image', 'stable']:
        by_group[group] = {}
        for case in (CASES[:1] if group == 'baseline' else CASES):
            selected = [s for s in samples if s['group'] == group and s['case'] == case]
            assert len(selected) == 3
            phases = {phase: {metric: describe([s['rows'][index][metric] for s in selected])
                             for metric in METRICS} for index, phase in enumerate(PHASES)}
            lifecycle = describe([s['lifecycle']['lifecycle_retained_requested_bytes'] for s in selected])
            outside_phases = describe([s['lifecycle']['lifecycle_retained_requested_bytes']
                - sum(row['retained_bytes_delta'] for row in s['rows']) for s in selected])
            by_group[group][case] = {'phases': phases, 'lifecycle_retained_requested_bytes': lifecycle,
                'retention_outside_measured_phases': outside_phases}
    comparisons = {}
    for candidate, reference in [('image', 'baseline'), ('stable', 'baseline'), ('stable', 'image')]:
        comparison = candidate + '_minus_' + reference
        comparisons[comparison] = {}
        for case in by_group[reference]:
            delta = {}
            for phase in PHASES:
                delta[phase] = {}
                for metric in METRICS:
                    before = by_group[reference][case]['phases'][phase][metric]['observations']
                    after = by_group[candidate][case]['phases'][phase][metric]['observations']
                    delta[phase][metric] = describe([a - b for a, b in zip(after, before)])
            comparisons[comparison][case] = delta
    text_bytes = {}
    for group in binaries:
        lines = (OUT / ('size-' + group + '.stdout')).read_text().splitlines()
        sizes = [int(line.split()[1]) for line in lines if line.split() and line.split()[0] == '.text']
        assert len(sizes) == 1
        text_bytes[group] = sizes[0]
    value = {
        'complete': True, 'assessor_sha256': sha(__file__), 'result_sha256': sha(OUT / 'result.json'),
        'commands_all_zero': 34, 'fixed_sample_processes': 27, 'exact_transcripts_rechecked': 81,
        'resources': by_group, 'paired_resource_deltas': comparisons,
        'phase_witness_sha256': protocol['phase_witness_sha256'],
        'phase_witness_bytes': protocol['phase_witness_bytes'],
        'all_outputs_match_group_phase_reference': True,
        'old_trial_samples_pooled': False,
        'interpretation': 'Every phase is compared against its own independently validated group reference. The parent 32x16 warm rows differ from its cold rows, and deterministic candidate rows differ from the parent. Resource deltas describe the same original finite-point hull task with those explicit changes; they are not row-preserving optimization evidence.',
        'instrumented_binary_raw_bytes': {group: row['bytes'] for group, row in binaries.items()},
        'instrumented_binary_text_bytes': text_bytes,
        'limits': [
            'Requested global allocator bytes only: no stack-byte, allocator usable-size or RSS claim.',
            'All three per-case observations are included; instrumentation times are not used as performance estimates.',
            'Only the already-certified 24x12 case has an admitted-runtime resource comparison.',
            'Cold/warm/replay share the same input and keep prior outputs alive. No per-query allocation is assumed to be a persistent proof cache.',
            'Validation/serialization occur outside measured phases; their retained delta is reported separately.',
            'Process-global shared scalar caches can survive input disposal and grammar-cache clearing.',
            'No production formal proof, directed campaign, complete resource-bound proof, full workspace CI or runtime admission is claimed.'
        ],
        'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(value, indent=2) + '\n')
    print(json.dumps({'complete': True, 'assessment_sha256': sha(destination),
        'lifecycle_retained': {g: {c: r['lifecycle_retained_requested_bytes'] for c, r in cases.items()}
                               for g, cases in by_group.items()}}))


if __name__ == '__main__':
    main()
