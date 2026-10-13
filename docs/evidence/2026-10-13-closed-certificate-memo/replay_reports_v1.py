"""Replay both complete boards through the frozen endpoint/P4 baseline and memo candidate.

Use the existing endpoint experiment's exact 108 extracted files at the same
absolute paths, preserving report path fields without normalization. Build both
CLIs in new, separate targets before replay. Original checks, minimum
width, complete reports and findings stay intact. Process wall time and RSS are
standalone observations, not matched speedups or internal native gate metrics.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/closed-certificate-memo'
PRIOR = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse/report-replay-v1'
INPUT_ROOT = ROOT / 'target/endpoint-coordinate-reuse/report-replay-v1/inputs'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from qualify_v1 import verify as verify_sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def write_gzip(path, data):
    with path.open('xb') as raw:
        with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as compressed:
            compressed.write(data)


def main():
    out = E / 'report-replay-v1'
    out.mkdir(exist_ok=False)
    work = WORK / 'report-replay-v1'
    work.mkdir(exist_ok=False)
    (work / 'bins').mkdir()
    (work / 'raw').mkdir()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    original = json.loads(gzip.decompress((E / 'preparation-v1/parent-source-hashes.json.gz').read_bytes()))
    dirs = {name: Path(row['source']) for name, row in preparation['variants'].items()}
    targets = {name: WORK / name / 'report-target' for name in dirs}
    variants = ['baseline', 'candidate']
    assert list(dirs) == variants
    assert all(not path.exists() for path in targets.values())
    native = json.loads((E / 'native-validation-v1/result.json').read_text())
    controls = json.loads((E / 'native-controls-v1/result.json').read_text())
    assert native['complete'] and controls['complete'] and controls['all_variant_identities_match']
    prior_protocol = json.loads((PRIOR / 'protocol.json').read_text())
    fixtures = prior_protocol['fixtures']
    assert len(fixtures) == 2
    fixtures[0]['order'] = ['baseline', 'candidate']
    fixtures[1]['order'] = ['candidate', 'baseline']
    inputs = json.loads(gzip.decompress((PRIOR / 'input-hashes.json.gz').read_bytes()))
    assert len(inputs) == 108
    assert all(sha(INPUT_ROOT / name) == digest for name, digest in inputs.items())
    for fixture in fixtures:
        relative = 'crates/hyperdrc/docs/' + fixture['zip']
        assert all(sha(directory / relative) == original[relative] == fixture['archive_sha256'] for directory in dirs.values())
        assert Path(fixture['board_path']).is_relative_to(INPUT_ROOT)
        assert sha(Path(fixture['board_path'])) == fixture['board_sha256']
    write_gzip(out / 'input-hashes.json.gz', (json.dumps(inputs, indent=2) + '\n').encode())
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'assessor_sha256': sha(E / 'assess_reports_v1.py'),
                'native_assessment_sha256': sha(E / 'native-assessment-v1.json'),
                'controls_assessment_sha256': sha(E / 'native-controls-assessment-v1.json'),
                'preparation_sha256': sha(E / 'preparation-v1/protocol.json'),
                'native_result_sha256': sha(E / 'native-validation-v1/result.json'),
                'controls_result_sha256': sha(E / 'native-controls-v1/result.json'),
                'prior_input_protocol_sha256': sha(PRIOR / 'protocol.json'),
                'sources': {k: str(v) for k, v in dirs.items()},
                'targets': {k: str(v) for k, v in targets.items()},
                'fixtures': fixtures, 'input_hashes_sha256': sha(out / 'input-hashes.json.gz'),
                'extracted_input_files': len(inputs), 'environment': env,
                'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'expected_cli_exit_code': 1,
                'exit_code_contract': 'run_cli exits 1 when a complete report has active findings; JSON parsing and finding counts are checked separately.',
                'normalization': None,
                'historical_comparison': 'The baseline also contains the later P4 repair. Compare its full bytes with the earlier endpoint reports, record every difference and require review; do not assume those unmeasured parent reports already match.', 'command_timeout_seconds': 1800, 'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands = []

    def verify():
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / 'assess_reports_v1.py') == protocol['assessor_sha256']
        assert sha(E / 'native-assessment-v1.json') == protocol['native_assessment_sha256']
        assert sha(E / 'native-controls-assessment-v1.json') == protocol['controls_assessment_sha256']
        verify_sources(preparation)
        assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
        assert sha(E / 'native-validation-v1/result.json') == protocol['native_result_sha256']
        assert sha(E / 'native-controls-v1/result.json') == protocol['controls_result_sha256']
        assert all(sha(INPUT_ROOT / name) == digest for name, digest in inputs.items())
        assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())

    def run(name, command, cwd, stdout_path, extra_env=None):
        verify()
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with stdout_path.open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command], cwd=cwd, env=os.environ | env | (extra_env or {}),
                                  stdout=stdout, stderr=stderr).returncode
        verify()
        record = {'name': name, 'command': command, 'cwd': str(cwd),
                  'extra_environment': extra_env or {}, 'exit_code': code,
                  'seconds': time.monotonic() - started, 'stdout_sha256': sha(stdout_path),
                  'stdout_bytes': stdout_path.stat().st_size,
                  'stderr_sha256': sha(out / (name + '.stderr'))}
        commands.append(record)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps(record), flush=True)
        return code

    binaries = {}
    for variant in variants:
        log = out / ('build-' + variant + '.stdout')
        code = run('build-' + variant,
                   ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--bin', 'hyperdrc', '--message-format=json'],
                   dirs[variant], log, {'CARGO_TARGET_DIR': str(targets[variant])})
        assert code == 0
        events = [json.loads(line) for line in log.read_text().splitlines() if line.startswith('{')]
        artifacts = [e for e in events if e.get('reason') == 'compiler-artifact'
                     and e.get('executable') and e['target']['name'] == 'hyperdrc'
                     and 'bin' in e['target']['kind']]
        assert len(artifacts) == 1
        artifact = artifacts[0]
        assert Path(artifact['manifest_path']) == dirs[variant] / 'crates/hyperdrc/Cargo.toml'
        executable = Path(artifact['executable'])
        assert executable.is_relative_to(targets[variant])
        assert artifact['target']['kind'] == ['bin'] and not artifact['profile']['test']
        binary = work / 'bins' / variant
        shutil.copy2(executable, binary)
        binaries[variant] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                             'features': artifact['features'], 'manifest_path': artifact['manifest_path'],
                             'package_id': artifact['package_id'], 'fresh': artifact['fresh'],
                             'emitted_path': str(executable), 'target': artifact['target'], 'profile': artifact['profile']}
        (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
    assert all(binaries['baseline']['features'] == binaries[variant]['features'] for variant in variants)
    (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
    comparisons, historical_matches = [], []
    for fixture in fixtures:
        reports = {}
        for variant in fixture['order']:
            binary = binaries[variant]
            assert sha(Path(binary['path'])) == binary['sha256']
            label = fixture['label'] + '-' + variant
            raw = work / 'raw' / (label + '.json')
            code = run(label, ['/usr/bin/time', '-v', binary['path'],
                               '--kicad-pcb', fixture['board_path'],
                               '--check', 'layer-sanity', '--check', 'board-outline-sanity',
                               '--check', 'min-copper-neck', '--check', 'drill-spacing',
                               '--min-width', '0.0762', '--format', 'json'], ROOT, raw)
            data = raw.read_bytes()
            write_gzip(out / (label + '.json.gz'), data)
            report = json.loads(data)
            assert code == 1
            assert report['violation_count'] == fixture['expected_findings']
            assert len(report['violations']) == fixture['expected_findings']
            assert report['inputs']
            reports[variant] = {'raw': data, 'report': report}
        for variant in ['candidate']:
            same_bytes = reports['baseline']['raw'] == reports[variant]['raw']
            same_value = reports['baseline']['report'] == reports[variant]['report']
            comparisons.append({'fixture': fixture['label'], 'variant': variant,
                                'complete_bytes_identical': same_bytes,
                                'complete_json_values_identical': same_value,
                                'baseline_bytes': len(reports['baseline']['raw']),
                                'candidate_bytes': len(reports[variant]['raw']),
                                'baseline_sha256': hashlib.sha256(reports['baseline']['raw']).hexdigest(),
                                'candidate_sha256': hashlib.sha256(reports[variant]['raw']).hexdigest(),
                                'violation_count': fixture['expected_findings']})
        historical = gzip.decompress((PRIOR / (fixture['label'] + '-candidate.json.gz')).read_bytes())
        historical_matches.append({'fixture': fixture['label'],
            'baseline_matches_prior_bytes': reports['baseline']['raw'] == historical,
            'baseline_matches_prior_json': reports['baseline']['report'] == json.loads(historical),
            'prior_bytes': len(historical), 'prior_sha256': hashlib.sha256(historical).hexdigest(),
            'baseline_sha256': hashlib.sha256(reports['baseline']['raw']).hexdigest()})
        (out / 'result.json').write_text(json.dumps({'complete': False, 'comparisons': comparisons,
                                                    'runtime_admitted': False}, indent=2) + '\n')
    verify()
    result = {'complete': True, 'comparisons': comparisons,
              'all_reports_byte_identical': all(r['complete_bytes_identical'] for r in comparisons),
              'all_report_values_identical': all(r['complete_json_values_identical'] for r in comparisons),
              'prior_complete_endpoint_reports_byte_identical': all(row['baseline_matches_prior_bytes'] for row in historical_matches),
              'historical_parent_comparisons': historical_matches,
              'inputs_and_sources_unchanged': True, 'main_runtime_unchanged': True,
              'runtime_admitted': False, 'application_speedup_claimed': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
