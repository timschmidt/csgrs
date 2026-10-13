"""Verify complete report identity and production executable sizes."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'report-replay-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from qualify_v1 import verify as verify_sources


def main():
    result = json.loads((OUT / 'result.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    assert result['complete'] and result['inputs_and_sources_unchanged']
    assert sha(E / 'replay_reports_v1.py') == protocol['runner_sha256']
    assert sha(Path(__file__)) == protocol['assessor_sha256']
    assert sha(E / 'native-assessment-v1.json') == protocol['native_assessment_sha256']
    assert sha(E / 'native-controls-assessment-v1.json') == protocol['controls_assessment_sha256']
    assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
    assert sha(E / 'native-validation-v1/result.json') == protocol['native_result_sha256']
    assert sha(E / 'native-controls-v1/result.json') == protocol['controls_result_sha256']
    assert sha(OUT / 'input-hashes.json.gz') == protocol['input_hashes_sha256']
    assert sha(ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse/report-replay-v1/protocol.json') == protocol['prior_input_protocol_sha256']
    assert protocol['normalization'] is None
    verify_sources(preparation)
    for variant, row in preparation['variants'].items():
        assert Path(binaries[variant]['manifest_path']) == Path(row['source']) / 'crates/hyperdrc/Cargo.toml'
        assert binaries[variant]['features'] == binaries['baseline']['features']
        assert Path(binaries[variant]['emitted_path']).is_relative_to(Path(protocol['targets'][variant]))
        assert sha(Path(binaries[variant]['emitted_path'])) == binaries[variant]['sha256']
        assert binaries[variant]['target']['kind'] == ['bin'] and not binaries[variant]['profile']['test']
        assert Path(binaries[variant]['path']).stat().st_size == binaries[variant]['bytes']
        events = [json.loads(line) for line in (OUT / ('build-' + variant + '.stdout')).read_text().splitlines() if line.startswith('{')]
        artifacts = [event for event in events if event.get('reason') == 'compiler-artifact'
                     and event.get('executable') and event['target']['name'] == 'hyperdrc'
                     and event['target']['kind'] == ['bin']]
        assert len(artifacts) == 1
        artifact = artifacts[0]
        assert artifact['executable'] == binaries[variant]['emitted_path']
        assert all(artifact[key] == binaries[variant][key] for key in ['features', 'manifest_path', 'package_id', 'target', 'profile'])
    for directory, library_name, binary_name in [('cost-v1', 'libraries.json', 'binaries.json'),
                                                   ('native-controls-v1', 'libraries.json', 'binaries.json')]:
        libraries = json.loads((E / directory / library_name).read_text())
        if directory == 'cost-v1':
            rows = libraries.values()
        else:
            rows = [row for entries in libraries.values() for row in entries.values()]
        assert all(sha(Path(row['path'])) == row['sha256'] for row in rows)
        assert all(sha(Path(row['path'])) == row['sha256'] for row in json.loads((E / directory / binary_name).read_text()).values())
    inputs = json.loads(gzip.decompress((OUT / 'input-hashes.json.gz').read_bytes()))
    assert len(inputs) == 108
    input_root = ROOT / 'target/endpoint-coordinate-reuse/report-replay-v1/inputs'
    assert all(sha(input_root / name) == digest for name, digest in inputs.items())
    assert all(sha(Path(value['path'])) == value['sha256'] for value in binaries.values())
    commands = json.loads((OUT / 'commands.json').read_text())
    assert [row['name'] for row in commands] == ['build-baseline', 'build-candidate'] + [
        fixture['label'] + '-' + variant for fixture in protocol['fixtures'] for variant in fixture['order']]
    assert len(result['comparisons']) == 2
    processes = []
    for command in commands:
        name = command['name']
        stderr = OUT / (name + '.stderr')
        assert sha(stderr) == command['stderr_sha256']
        if name.startswith('build-'):
            assert command['exit_code'] == 0
            assert sha(OUT / (name + '.stdout')) == command['stdout_sha256']
            continue
        data = gzip.decompress((OUT / (name + '.json.gz')).read_bytes())
        assert hashlib.sha256(data).hexdigest() == command['stdout_sha256']
        assert len(data) == command['stdout_bytes']
        report = json.loads(data)
        assert command['exit_code'] == 1
        assert report['violation_count'] == len(report['violations'])
        rss = re.search(r'Maximum resident set size \(kbytes\): (\d+)', stderr.read_text())
        assert rss
        processes.append({'name': name, 'process_seconds': command['seconds'],
                          'maximum_rss_kib': int(rss[1]), 'findings': report['violation_count']})
    assert len(processes) == 4
    historical_matches = []
    for comparison in result['comparisons']:
        label = comparison['fixture']
        baseline = gzip.decompress((OUT / (label + '-baseline.json.gz')).read_bytes())
        candidate = gzip.decompress((OUT / (label + '-' + comparison['variant'] + '.json.gz')).read_bytes())
        prior = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse/report-replay-v1' / (label + '-candidate.json.gz')
        historical = gzip.decompress(prior.read_bytes())
        historical_matches.append({'fixture': label,
            'baseline_matches_prior_bytes': baseline == historical,
            'baseline_matches_prior_json': json.loads(baseline) == json.loads(historical),
            'prior_bytes': len(historical), 'prior_sha256': hashlib.sha256(historical).hexdigest(),
            'baseline_sha256': hashlib.sha256(baseline).hexdigest()})
        assert (baseline == candidate) == comparison['complete_bytes_identical']
        assert (json.loads(baseline) == json.loads(candidate)) == comparison['complete_json_values_identical']
    assert historical_matches == result['historical_parent_comparisons']
    assert result['prior_complete_endpoint_reports_byte_identical'] == all(row['baseline_matches_prior_bytes'] for row in historical_matches)
    assert result['all_reports_byte_identical'] == all(row['complete_bytes_identical'] for row in result['comparisons'])
    assert result['all_report_values_identical'] == all(row['complete_json_values_identical'] for row in result['comparisons'])
    assessment = {
        'decision': 'complete-report-replay-recorded',
        'assessor_sha256': sha(Path(__file__)),
        'complete_reports_byte_identical': result['all_reports_byte_identical'],
        'complete_report_values_identical': result['all_report_values_identical'],
        'comparisons': result['comparisons'],
        'production_binaries': binaries,
        'production_binary_byte_delta': {variant: binaries[variant]['bytes'] - binaries['baseline']['bytes'] for variant in ['candidate']},
        'prior_complete_endpoint_reports_byte_identical': result['prior_complete_endpoint_reports_byte_identical'],
        'historical_parent_comparisons': historical_matches,
        'scalar_and_native_control_libraries_and_binaries_still_match': True,
        'process_observations': processes,
        'limits': [
            'Both complete original boards use shared absolute paths and all original input bytes. No report fields are normalized or omitted.',
            'Active findings explain CLI exit code 1; JSON structure, complete bytes and counts are checked independently.',
            'Four standalone CLI executions are report verification, not a fixed repeated application benchmark or a substitute for either original runtime assertion.',
            'Process RSS includes fixture import, setup and reports; it does not isolate constructor allocations or prove graph reclamation.',
            'The same absolute input paths are reused from the endpoint trial. Historical parent comparisons are recorded separately because this baseline also contains the later P4 repair. Any differences require review; none is normalized away.',
        ],
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'main_runtime_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    assert not (E / 'report-assessment-v1.json').exists()
    (E / 'report-assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({key: assessment[key] for key in ['complete_reports_byte_identical',
                      'production_binary_byte_delta', 'comparisons', 'process_observations']}))


if __name__ == '__main__':
    main()
