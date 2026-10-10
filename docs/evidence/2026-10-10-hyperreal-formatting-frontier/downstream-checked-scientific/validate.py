"""Check downstream suites, original board gate, witnesses and complete reports."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PREVIOUS = EVIDENCE.parent / '2026-10-09-hyperdrc-native-linear-strokes'
variant = sys.argv[1]
assert variant in ['checked-scientific', 'class-precheck']
OUT = EVIDENCE / ('downstream-' + variant)
WORK = ROOT / ('target/hyperreal-downstream-' + variant)
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
info = lambda path: {'bytes': path.stat().st_size, 'sha256': digest(path)}
record_path = EVIDENCE / variant / 'sources.json'
record = read(record_path)
source, target = Path(record['source_directory']), Path(record['target_directory'])
hash_file = Path(record['source_hashes_file'])
assert digest(hash_file) == record['source_hashes_file_sha256']
hashes = json.loads(gzip.decompress(hash_file.read_bytes()))
native = read(Path(record['main_source_record']))['source_hashes']
assert read(EVIDENCE / 'class-precheck-cost/source-verification.json')['sampling_complete']
OUT.mkdir()
WORK.mkdir()
(OUT / '.gitignore').write_text('/reports-1/\n/reports-2/\n')
shutil.copyfile(__file__, OUT / 'validate.py')
probes = OUT / 'probe_sources'
probes.mkdir()
previous = EVIDENCE.parent / '2026-10-09-hyperdrc-native-pad-primitives'
for path in (previous / 'cost').glob('*.rs'):
    shutil.copyfile(path, probes / path.name)
shutil.copyfile(previous / 'full_report_replay.rs', probes / 'full_report_replay.rs')
probe_hashes = {path.name: digest(path) for path in probes.iterdir()}
input_record = read(previous / 'report-replay/manifest.json')
inputs = Path(input_record['trials'][0]['command'][-2])
inventory = lambda: {str(path.relative_to(inputs)): info(path) for path in sorted(inputs.rglob('*')) if path.is_file()}
assert inventory() == input_record['input_files']
assert all(info(Path(name)) == expected for name, expected in input_record['archives'].items())
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_TARGET_DIR': str(target), 'RUSTDOCFLAGS': '-D warnings'}
commands = []


def verify():
    assert all(digest(source / name) == value for name, value in hashes.items())
    assert all(digest(ROOT / name) == value for name, value in native.items())
    assert all(digest(probes / name) == value for name, value in probe_hashes.items())


def run(name, command, tests=False):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    path = OUT / (name + '.log')
    with path.open('w') as log:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=5s', '1200', *command], cwd=source, env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
    row = {'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start}
    commands.append(row)
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name
    raw = path.read_text()
    if tests:
        counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', raw)
        assert counts and sum(int(item[0]) for item in counts) > 0
        row['test_binaries'] = len(counts)
        row['tests_passed'] = sum(int(item[0]) for item in counts)
        row['tests_ignored'] = sum(int(item[2]) for item in counts)
        assert all(int(item[1]) == 0 for item in counts)
        (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    return raw


run('curve-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--all-targets', '--', '-D', 'warnings'])
run('curve-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', '--tests', '--', '--test-threads=2'], tests=True)
run('curve-doc', ['cargo', 'doc', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', '--no-deps'])
build = run('build-drc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--no-run', '--message-format=json'])
events = [json.loads(line) for line in build.splitlines() if line.startswith('{')]
executables = [event['executable'] for event in events if event.get('reason') == 'compiler-artifact' and event.get('executable') and event['target']['name'] == 'hyperdrc']
assert len(executables) == 1
test_binary = Path(executables[0])
test_binary_info = {'path': str(test_binary), **info(test_binary)}
run('drc-tests', [str(test_binary), '--test-threads=2'], tests=True)
run('drc-integration-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--test', 'kicad_pad_sources', '--test', 'readme', '--', '--test-threads=2'], tests=True)
run('drc-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-targets', '--', '-D', 'warnings'])
run('drc-doc', ['cargo', 'doc', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-deps'])
assert info(test_binary) == {key: value for key, value in test_binary_info.items() if key != 'path'}
board = run('full-board-smoke', ['/usr/bin/time', '-v', '-o', str(OUT / 'full-board-smoke-time.log'), str(test_binary), 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'], tests=True)
assert 'skipping complex project fixture test' not in board
assert board.count('hyperdrc: finished emit report') == 2
build = run('production-library-build', ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--message-format=json'])
events = [json.loads(line) for line in build.splitlines() if line.startswith('{')]
libraries = {crate: next(Path(filename) for event in events if event.get('reason') == 'compiler-artifact' and event['target']['name'] == crate for filename in event['filenames'] if filename.endswith('.rlib')) for crate in ['hyperdrc', 'hyperlimit', 'hypercurve', 'serde_json', 'clap']}
binaries = {}
for kind, filename, crates in [('operation', 'operation_probe.rs', ['hyperdrc', 'hyperlimit', 'hypercurve', 'serde_json']), ('report', 'full_report_replay.rs', ['hyperdrc', 'clap', 'serde_json'])]:
    binary = WORK / kind
    command = ['rustc', '--edition=2024', '-O', str(probes / filename), '-L', 'dependency=' + str(target / 'release/deps')]
    for crate in crates:
        command += ['--extern', crate + '=' + str(libraries[crate])]
    run(kind + '-build', command + ['-o', str(binary)])
    binaries[kind] = {'path': str(binary), **info(binary)}
historical_path = EVIDENCE.parent / '2026-10-09-hyperdrc-kicad-pad-sources/cost/rows.json'
historical = read(historical_path)
for index in [1, 2]:
    raw = run('operation-' + str(index), ['taskset', '-c', '0', str(WORK / 'operation')])
    rows = []
    for line in raw.splitlines():
        if line.startswith('operation_'):
            name, count, nanos, witness = line.split('\t')
            rows.append({'name': name, 'iterations': int(count), 'witness': witness})
    assert len(rows) == 231 and rows == historical
(OUT / 'rows.json').write_text(json.dumps(historical, sort_keys=True, separators=(',', ':')) + '\n')
reports = {}
for index in [1, 2]:
    destination = OUT / ('reports-' + str(index))
    run('reports-' + str(index), ['taskset', '-c', '0', '/usr/bin/time', '-v', str(WORK / 'report'), str(inputs), str(destination)])
    for board in ['cparti-fpga', 'hvp109a']:
        path = destination / (board + '.json')
        raw = path.read_bytes()
        admitted_path = PREVIOUS / 'contact-index-directed-replay' / ('candidate-reports-1-' + board + '.json.gz')
        assert raw == gzip.decompress(admitted_path.read_bytes())
        data = json.loads(raw)
        assert data['violation_count'] == len(data['violations'])
        (OUT / f'reports-{index}-{board}.json.gz').write_bytes(gzip.compress(raw, mtime=0))
        report = {**info(path), 'violation_count': data['violation_count'], 'admitted_report': str(admitted_path), 'admitted_report_sha256': digest(admitted_path)}
        assert board not in reports or reports[board] == report
        reports[board] = report
verify()
assert inventory() == input_record['input_files']
assert all(info(Path(record['path'])) == {key: value for key, value in record.items() if key != 'path'} for record in binaries.values())
summary = {
    'decision': 'scoped downstream suites, original board gate, all 231 witnesses and complete report bytes pass; final runtime retention remains',
    'source_record': str(record_path), 'source_record_sha256': digest(record_path),
    'source_files': len(hashes), 'main_files': len(native), 'all_source_and_probe_hashes_unchanged': True,
    'commands': commands, 'test_binary': test_binary_info, 'probe_binaries': binaries,
    'libraries': {str(path): info(path) for path in libraries.values()},
    'historical_witnesses': 231, 'witness_processes': 2, 'historical_rows': str(historical_path), 'historical_rows_sha256': digest(historical_path),
    'report_processes': 2, 'reports': reports, 'input_files': input_record['input_files'], 'archives': input_record['archives'],
    'limitations': 'No matched downstream performance claim, whole-workspace/pre-push validation, native-prototype admission, or full property completion. Formatting and strict scalar checks are reused only from the identical frozen source validation.',
}
summary['artifacts'] = {str(path.relative_to(EVIDENCE)): info(path) for path in sorted(OUT.rglob('*')) if path.is_file() and path.parts[-2] not in ['reports-1', 'reports-2']}
summary['script_sha256'] = digest(Path(__file__))
(EVIDENCE / ('downstream-' + variant + '.json')).write_text(json.dumps(summary, indent=2) + '\n')
print('Downstream validation, 231 witnesses and complete repeated report bytes pass.', flush=True)
