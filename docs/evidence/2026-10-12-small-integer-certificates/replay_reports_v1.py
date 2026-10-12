"""Compare complete full-board JSON reports without normalizing any fields.

The frozen persistent-cache baseline and integer-leaf candidate read identical
full-resolution archives extracted to shared absolute input paths. Each board
runs once per variant in a fresh CLI process, with the original four checks and
minimum width. Both executables are built and bound before replay. This closes
a report comparison gap; it is not a matched application speedup experiment and
does not replace the failed original 120-second worker CPU gate.
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
import zipfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/small-integer-certificates'
BASE = ROOT / 'target/persistent-laurent-cache/candidate'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

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
    original = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    delta = json.loads((E / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    maps = {'baseline': original, 'candidate': original | delta}
    dirs = {'baseline': BASE / 'source', 'candidate': WORK / 'source'}
    targets = {'baseline': BASE / 'cost-v2-target', 'candidate': WORK / 'target'}
    preparation = json.loads((E / 'preparation.json').read_text())
    native = json.loads((E / 'native-gate-v1/result.json').read_text())
    assert native['complete'] and not native['all_passed']
    for variant, entries in maps.items():
        assert all(sha(dirs[variant] / p) == expected for p, expected in entries.items())
    fixtures = [
        {'label': 'cparti-fpga', 'zip': 'CPArti FPGA dev board.zip',
         'board': 'CPArti FPGA dev board.kicad_pcb', 'expected_findings': 12,
         'order': ['baseline', 'candidate']},
        {'label': 'hvp109a', 'zip': 'HVP109A.zip', 'board': 'HVP109A.kicad_pcb',
         'expected_findings': 197, 'order': ['candidate', 'baseline']},
    ]
    inputs = {}
    for fixture in fixtures:
        relative = 'crates/hyperdrc/docs/' + fixture['zip']
        archive = dirs['baseline'] / relative
        assert sha(archive) == original[relative]
        destination = work / 'inputs' / fixture['label']
        destination.mkdir(parents=True)
        with zipfile.ZipFile(archive) as zipped:
            for member in zipped.infolist():
                assert (destination / member.filename).resolve().is_relative_to(destination.resolve())
            zipped.extractall(destination)
        board = destination / fixture['board']
        assert board.is_file()
        fixture.update({'archive_sha256': sha(archive), 'board_path': str(board),
                        'board_sha256': sha(board)})
        inputs.update({str(p.relative_to(work / 'inputs')): sha(p)
                       for p in sorted(destination.rglob('*')) if p.is_file()})
    write_gzip(out / 'input-hashes.json.gz', (json.dumps(inputs, indent=2) + '\n').encode())
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'preparation_sha256': sha(E / 'preparation.json'),
                'native_result_sha256': sha(E / 'native-gate-v1/result.json'),
                'native_assessment_sha256': sha(E / 'native-assessment-v1.json'),
                'source_changes_sha256': delta,
                'sources': {k: str(v) for k, v in dirs.items()},
                'targets': {k: str(v) for k, v in targets.items()},
                'fixtures': fixtures, 'input_hashes_sha256': sha(out / 'input-hashes.json.gz'),
                'extracted_input_files': len(inputs), 'environment': env,
                'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'expected_cli_exit_code': 1,
                'exit_code_contract': 'run_cli exits 1 when a complete report has active findings; JSON parsing and finding counts are checked separately.',
                'normalization': None, 'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands = []

    def verify():
        assert sources() == preparation['main_source_sha256']
        for variant, entries in maps.items():
            assert all(sha(dirs[variant] / name) == entries[name] for name in delta)
        assert all(sha(work / 'inputs' / name) == expected for name, expected in inputs.items())

    def run(name, command, cwd, stdout_path, extra_env=None):
        verify()
        started = time.monotonic()
        with stdout_path.open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            code = subprocess.run(command, cwd=cwd, env=os.environ | env | (extra_env or {}),
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
    for variant in ['baseline', 'candidate']:
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
        binary = work / 'bins' / variant
        shutil.copy2(executable, binary)
        binaries[variant] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                             'features': artifact['features'], 'manifest_path': artifact['manifest_path'],
                             'package_id': artifact['package_id'], 'fresh': artifact['fresh']}
    assert binaries['baseline']['features'] == binaries['candidate']['features']
    assert binaries['baseline']['sha256'] != binaries['candidate']['sha256']
    (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
    comparisons = []
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
        same_bytes = reports['baseline']['raw'] == reports['candidate']['raw']
        same_value = reports['baseline']['report'] == reports['candidate']['report']
        comparisons.append({'fixture': fixture['label'], 'complete_bytes_identical': same_bytes,
                            'complete_json_values_identical': same_value,
                            'baseline_bytes': len(reports['baseline']['raw']),
                            'candidate_bytes': len(reports['candidate']['raw']),
                            'baseline_sha256': hashlib.sha256(reports['baseline']['raw']).hexdigest(),
                            'candidate_sha256': hashlib.sha256(reports['candidate']['raw']).hexdigest(),
                            'violation_count': fixture['expected_findings']})
        (out / 'result.json').write_text(json.dumps({'complete': False, 'comparisons': comparisons,
                                                    'runtime_admitted': False}, indent=2) + '\n')
    verify()
    for variant, entries in maps.items():
        assert all(sha(dirs[variant] / p) == expected for p, expected in entries.items())
    result = {'complete': True, 'comparisons': comparisons,
              'all_reports_byte_identical': all(r['complete_bytes_identical'] for r in comparisons),
              'all_report_values_identical': all(r['complete_json_values_identical'] for r in comparisons),
              'inputs_and_sources_unchanged': True, 'main_runtime_unchanged': True,
              'runtime_admitted': False, 'application_speedup_claimed': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
