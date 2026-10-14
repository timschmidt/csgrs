"""Assess every fixed pair for the deterministic-selection correctness repair."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import statistics

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-stable-selection-costs-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    destination = E / 'sphere-stable-selection-cost-assessment-v1.json'
    assert not destination.exists()
    result = json.loads((OUT / 'result.json').read_text())
    assert result['complete'] and result['commands_all_zero'] == 58
    assert result['preflight_processes'] == 8 and result['sample_processes'] == 48
    assert result['all_outputs_match_group_specific_exactly_validated_reference']
    assert all(sha(OUT / name) == digest for name, digest in result['artifacts_sha256'].items())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    root_mapping = json.loads(gzip.decompress((OUT / 'root-source-hashes.json.gz').read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
    priors = {'image': E / 'sphere-cyclotomic-image-v1', 'stable': E / 'sphere-stable-selection-v1'}
    for group, prior in priors.items():
        assert sha(prior / 'protocol.json') == protocol['source_protocols_sha256'][group]
        source = Path(json.loads((prior / 'protocol.json').read_text())['source'])
        mapping = json.loads(gzip.decompress((prior / 'source-hashes.json.gz').read_bytes()))
        assert all(sha(source / name) == digest for name, digest in mapping.items())
        binary = protocol['binaries'][group]
        assert sha(binary['path']) == binary['sha256']
        assert Path(binary['path']).stat().st_size == binary['bytes']
        assert all(sha(row['path']) == row['sha256'] for row in protocol['libraries'][group].values())
    commands = json.loads((OUT / 'commands.json').read_text())
    assert len(commands) == 58 and all(row['exit_code'] == 0 for row in commands)
    samples = json.loads((OUT / 'samples.json').read_text())
    expected_order = []
    for round_number, order in [(0, ['image', 'stable']), *enumerate(protocol['orders'], 1)]:
        for position, group in enumerate(order, 1):
            for case in protocol['cases']:
                expected_order.append((round_number, position, group, case))
    assert [(row['round'], row['position'], row['group'], row['case']) for row in samples] == expected_order
    assert len(samples) == 56
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    for row in samples:
        assert row['preflight'] == (row['round'] == 0)
        assert row['full_output_matches_group_reference']
        group, case = row['group'], row['case']
        expected = E / protocol['reference_transcripts'][group][case]
        assert sha(expected) == protocol['reference_output_sha256'][group][case]
        assert expected.stat().st_size == protocol['reference_output_bytes'][group][case]
        assert (OUT / (row['name'] + '.stdout')).read_bytes() == expected.read_bytes()
    cases = []
    for case in protocol['cases']:
        pairs = []
        for round_number in range(1, 7):
            pair = {row['group']: row for row in samples if row['case'] == case and row['round'] == round_number}
            assert set(pair) == {'image', 'stable'}
            image, stable = pair['image'], pair['stable']
            image_cpu = image['child_user_cpu_seconds'] + image['child_system_cpu_seconds']
            stable_cpu = stable['child_user_cpu_seconds'] + stable['child_system_cpu_seconds']
            pairs.append({'round': round_number,
                'image_process_wall_seconds': image['process_wall_seconds'],
                'stable_process_wall_seconds': stable['process_wall_seconds'],
                'image_child_cpu_seconds': image_cpu, 'stable_child_cpu_seconds': stable_cpu,
                'process_wall_ratio': stable['process_wall_seconds'] / image['process_wall_seconds'],
                'child_cpu_ratio': stable_cpu / image_cpu})
        ratios = [pair['process_wall_ratio'] for pair in pairs]
        median = statistics.median(ratios)
        flagged = all(ratio > 1.10 for ratio in ratios) and median > 1.15
        cases.append({'case': case, 'pairs': pairs, 'median_process_wall_ratio': median,
            'min_process_wall_ratio': min(ratios), 'max_process_wall_ratio': max(ratios),
            'median_child_cpu_ratio': statistics.median(pair['child_cpu_ratio'] for pair in pairs),
            'median_process_wall_seconds': {group: statistics.median(pair[group + '_process_wall_seconds'] for pair in pairs) for group in priors},
            'median_child_cpu_seconds': {group: statistics.median(pair[group + '_child_cpu_seconds'] for pair in pairs) for group in priors},
            'reproducible_cost_increase_flagged': flagged})
    sections = {}
    for group in priors:
        raw = (OUT / ('size-' + group + '.stdout')).read_text()
        sections[group] = {name: int(size) for name, size in re.findall(r'^(\.\S+)\s+(\d+)\s+\d+\s*$', raw, re.MULTILINE)}
        assert '.text' in sections[group]
    value = {'complete': True, 'assessor_sha256': sha(__file__), 'result_sha256': sha(OUT / 'result.json'),
        'commands_all_zero': 58, 'preflight_processes': 8, 'fixed_sample_processes': 48,
        'all_outputs_match_group_specific_exactly_validated_reference': True,
        'cross_group_row_changes_explicitly_retained': True,
        'reference_output_bytes': protocol['reference_output_bytes'],
        'reference_output_sha256': protocol['reference_output_sha256'], 'cases': cases,
        'binary_file_bytes': {group: row['bytes'] for group, row in protocol['binaries'].items()},
        'binary_sections_bytes': sections,
        'binary_file_delta_stable_minus_image': protocol['binaries']['stable']['bytes'] - protocol['binaries']['image']['bytes'],
        'text_delta_stable_minus_image': sections['stable']['.text'] - sections['image']['.text'],
        'interpretation': 'Descriptive fixed paired whole-process cost comparison of the same original finite-point hull tasks. Every process performs three queries. The proposed repair changes serialized rows while preserving the independently verified exact geometry. All eight preflights are retained and excluded from the six predeclared measurement pairs.',
        'limits': ['No allocation, retained-byte, RSS, check-body or separate per-query timing evidence.',
            'This is not a row-preserving optimization comparison. Unflagged cases do not prove absence of regression.',
            'No universal performance, formal verification, fuzz-campaign or runtime-admission claim.'],
        'runtime_admitted': False, 'plan_complete': False}
    destination.write_text(json.dumps(value, indent=2) + '\n')
    print(json.dumps({'complete': True, 'cases': [{key: row[key] for key in
        ['case', 'median_process_wall_ratio', 'reproducible_cost_increase_flagged']} for row in cases],
        'assessment_sha256': sha(destination)}))


if __name__ == '__main__':
    main()
