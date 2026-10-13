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
from validate import sources


def main():
    result = json.loads((OUT / 'result.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    assert result['complete'] and result['inputs_and_sources_unchanged']
    assert sha(E / 'replay_reports_v1.py') == protocol['runner_sha256']
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in preparation['pending_native_sha256'].items())
    parent = json.loads(gzip.decompress((E / 'preparation-v1/parent-source-hashes.json.gz').read_bytes()))
    assert all(sha(Path(preparation['parent_source']) / name) == digest for name, digest in parent.items())
    for variant, row in preparation['variants'].items():
        assert all(sha(Path(row['source']) / name) == digest
                   for name, digest in (parent | row['source_changes_sha256']).items())
        assert Path(binaries[variant]['manifest_path']) == Path(row['source']) / 'crates/hyperdrc/Cargo.toml'
        assert binaries[variant]['features'] == binaries['baseline']['features']
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
    assert len(processes) == 6
    for comparison in result['comparisons']:
        label = comparison['fixture']
        baseline = gzip.decompress((OUT / (label + '-baseline.json.gz')).read_bytes())
        candidate = gzip.decompress((OUT / (label + '-' + comparison['variant'] + '.json.gz')).read_bytes())
        prior = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse/report-replay-v1' / (label + '-candidate.json.gz')
        assert baseline == gzip.decompress(prior.read_bytes())
        assert (baseline == candidate) == comparison['complete_bytes_identical']
        assert (json.loads(baseline) == json.loads(candidate)) == comparison['complete_json_values_identical']
    assessment = {
        'decision': 'complete-report-replay-recorded',
        'complete_reports_byte_identical': result['all_reports_byte_identical'],
        'complete_report_values_identical': result['all_report_values_identical'],
        'comparisons': result['comparisons'],
        'production_binaries': binaries,
        'production_binary_byte_delta': {variant: binaries[variant]['bytes'] - binaries['baseline']['bytes'] for variant in ['right', 'both']},
        'prior_complete_endpoint_reports_byte_identical': True,
        'scalar_and_native_control_libraries_and_binaries_still_match': True,
        'process_observations': processes,
        'limits': [
            'Both complete original boards use shared absolute paths and all original input bytes. No report fields are normalized or omitted.',
            'Active findings explain CLI exit code 1; JSON structure, complete bytes and counts are checked independently.',
            'Six standalone CLI executions are report verification, not a fixed repeated application benchmark or a substitute for either original runtime assertion.',
            'Process RSS includes fixture import, setup and reports; it does not isolate constructor allocations or prove graph reclamation.',
            'The same absolute input paths are reused from the endpoint trial; complete baseline bytes also match those prior reports.',
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
