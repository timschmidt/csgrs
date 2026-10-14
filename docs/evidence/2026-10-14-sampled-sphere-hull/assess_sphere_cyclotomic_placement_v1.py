"""Assess every sample in the frozen native placement timing screen."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import statistics

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-cyclotomic-placement-costs-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    destination = E / 'sphere-cyclotomic-placement-cost-assessment-v1.json'
    assert not destination.exists()
    result = json.loads((OUT / 'result.json').read_text())
    assert result['complete'] and result['commands_all_zero'] == 58
    assert result['preflight_processes'] == 8 and result['sample_processes'] == 48
    assert all(sha(OUT / name) == digest for name, digest in result['artifacts_sha256'].items())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    root_mapping = json.loads(gzip.decompress((OUT / 'root-source-hashes.json.gz').read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
    priors = {'late': E / 'sphere-cyclotomic-v2', 'early': E / 'sphere-cyclotomic-early-v1'}
    for group, prior in priors.items():
        assert sha(prior / 'protocol.json') == protocol['source_protocols_sha256'][group]
        source = Path(json.loads((prior / 'protocol.json').read_text())['source'])
        mapping = json.loads(gzip.decompress((prior / 'source-hashes.json.gz').read_bytes()))
        assert all(sha(source / name) == digest for name, digest in mapping.items())
        binary = protocol['binaries'][group]
        assert sha(binary['path']) == binary['sha256']
        assert Path(binary['path']).stat().st_size == binary['bytes']
    commands = json.loads((OUT / 'commands.json').read_text())
    assert len(commands) == 58 and all(row['exit_code'] == 0 for row in commands)
    samples = json.loads((OUT / 'samples.json').read_text())
    expected_order = []
    for round_number, order in [(0, ['late', 'early']), *enumerate(protocol['orders'], 1)]:
        for position, group in enumerate(order, 1):
            for case in protocol['cases']:
                expected_order.append((round_number, position, group, case))
    assert [(s['round'], s['position'], s['group'], s['case']) for s in samples] == expected_order
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    for row in samples:
        assert row['preflight'] == (row['round'] == 0)
        assert row['full_output_identical']
        expected = priors['late'] / ('candidate-' + row['case'] + '.stdout')
        assert (OUT / (row['name'] + '.stdout')).read_bytes() == expected.read_bytes()
    cases = []
    for case in protocol['cases']:
        pairs = []
        for round_number in range(1, 7):
            pair = {row['group']: row for row in samples if row['case'] == case and row['round'] == round_number}
            assert set(pair) == {'late', 'early'}
            late, early = pair['late'], pair['early']
            late_cpu = late['child_user_cpu_seconds'] + late['child_system_cpu_seconds']
            early_cpu = early['child_user_cpu_seconds'] + early['child_system_cpu_seconds']
            pairs.append({'round': round_number,
                'late_process_wall_seconds': late['process_wall_seconds'],
                'early_process_wall_seconds': early['process_wall_seconds'],
                'process_wall_ratio': early['process_wall_seconds'] / late['process_wall_seconds'],
                'child_cpu_ratio': early_cpu / late_cpu})
        ratios = [pair['process_wall_ratio'] for pair in pairs]
        median = statistics.median(ratios)
        flagged = all(ratio > 1.10 for ratio in ratios) and median > 1.15
        cases.append({'case': case, 'pairs': pairs, 'median_process_wall_ratio': median,
            'min_process_wall_ratio': min(ratios), 'max_process_wall_ratio': max(ratios),
            'median_child_cpu_ratio': statistics.median(pair['child_cpu_ratio'] for pair in pairs),
            'reproducible_cost_regression_flagged': flagged})
    sections = {}
    for group in priors:
        raw = (OUT / ('size-' + group + '.stdout')).read_text()
        sections[group] = {name: int(size) for name, size in re.findall(r'^(\.\S+)\s+(\d+)\s+\d+\s*$', raw, re.MULTILINE)}
        assert '.text' in sections[group]
    value = {'complete': True, 'assessor_sha256': sha(__file__), 'result_sha256': sha(OUT / 'result.json'),
        'commands_all_zero': 58, 'preflight_processes': 8, 'fixed_sample_processes': 48,
        'all_original_native_output_rows_and_certainty_identical': True,
        'cases': cases,
        'binary_file_bytes': {group: row['bytes'] for group, row in protocol['binaries'].items()},
        'binary_sections_bytes': sections,
        'binary_file_delta_early_minus_late': protocol['binaries']['early']['bytes'] - protocol['binaries']['late']['bytes'],
        'text_delta_early_minus_late': sections['early']['.text'] - sections['late']['.text'],
        'interpretation': 'Descriptive fixed paired whole-process comparison for four original inputs and two isolated implementations. Preflight samples were retained but excluded from the six predeclared measurement pairs.',
        'limits': ['No allocation, retained-byte, RSS or check-body timing evidence.',
                   'No general performance, formal verification, broad early-filter qualification or runtime admission claim.'],
        'runtime_admitted': False, 'plan_complete': False}
    destination.write_text(json.dumps(value, indent=2) + '\n')
    print(json.dumps({'complete': True, 'cases': [{key: row[key] for key in
        ['case', 'median_process_wall_ratio', 'reproducible_cost_regression_flagged']} for row in cases],
        'assessment_sha256': sha(destination)}))


if __name__ == '__main__':
    main()
