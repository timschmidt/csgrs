"""Replay retained capability rows and complete reports for endpoint reuse."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-production-replay'
PRODUCTION = EVIDENCE / 'arc-endpoint-certificate'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
info = lambda path: {'bytes': path.stat().st_size, 'sha256': digest(path)}
source = read(PRODUCTION / 'candidate-sources.json')
base = read(PRODUCTION / 'baseline-sources.json')
native = read(EVIDENCE / 'native-exactness-gate/sources.json')
status = read(EVIDENCE / 'arc-endpoint-production-validation/status.json')
assert len(status) == 10 and all(row['exit_code'] == 0 for row in status)
assert read(EVIDENCE / 'diagnostics-79.json')['tests']['drc-integration-tests']['passed'] == 35
assert read(EVIDENCE / 'diagnostics-80.json')['final_query_abort_verified']
assert read(EVIDENCE / 'diagnostics-78.json')['decision'].startswith('early production resource gate passed;')
variants = {}
for name, record in [('baseline', base), ('candidate', source)]:
    path = Path(record['source_hashes_file'])
    assert digest(path) == record['source_hashes_file_sha256']
    variants[name] = {'source': Path(record['source_directory']), 'target': Path(record['target_directory']), 'hashes': json.loads(gzip.decompress(path.read_bytes()))}
WORK = ROOT / 'target/arc-endpoint-production-replay'
WORK.mkdir()
OUT.mkdir()
(OUT / '.gitignore').write_text('/baseline-reports-*/\n/candidate-reports-*/\n')
previous = ROOT / 'docs/evidence/2026-10-09-hyperdrc-native-pad-primitives'
input_record = read(previous / 'report-replay/manifest.json')
inputs = Path(input_record['trials'][0]['command'][-2])
inventory = lambda: {str(path.relative_to(inputs)): info(path) for path in sorted(inputs.rglob('*')) if path.is_file()}
assert inventory() == input_record['input_files']
assert all(info(Path(name)) == expected for name, expected in input_record['archives'].items())
probe_dir = OUT / 'probe_sources'
probe_dir.mkdir()
for path in (previous / 'cost').glob('*.rs'):
    shutil.copyfile(path, probe_dir / path.name)
shutil.copyfile(previous / 'full_report_replay.rs', probe_dir / 'full_report_replay.rs')
shutil.copyfile(__file__, OUT / 'reproduce.py')
commands = []


def verify():
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
    for variant in variants.values():
        assert all(digest(variant['source'] / name) == value for name, value in variant['hashes'].items())


def run(label, command, cwd=ROOT, env=None):
    verify()
    print('start', label, flush=True)
    start = time.monotonic()
    with (OUT / (label + '.log')).open('w') as log:
        result = subprocess.run(command, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT)
    commands.append({'name': label, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    assert result.returncode == 0, label
    print('finish', label, flush=True)
    return (OUT / (label + '.log')).read_text()


for name, variant in variants.items():
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(variant['target']), 'CCACHE_DIR': '/tmp/csgrs-properties-ccache'}
    text = run(name + '-library-build', ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--message-format=json'], variant['source'], env)
    events = [json.loads(line) for line in text.splitlines() if line.startswith('{')]
    libraries = {
        crate: next(Path(filename) for event in events if event.get('reason') == 'compiler-artifact' and event['target']['name'] == crate for filename in event['filenames'] if filename.endswith('.rlib'))
        for crate in ['hyperdrc', 'hyperlimit', 'hypercurve', 'serde_json', 'clap']
    }
    variant['libraries'] = libraries
    variant['binaries'] = {}
    for kind, filename, crates in [
        ('operation', 'operation_probe.rs', ['hyperdrc', 'hyperlimit', 'hypercurve', 'serde_json']),
        ('report', 'full_report_replay.rs', ['hyperdrc', 'clap', 'serde_json']),
    ]:
        binary = WORK / ('replay-' + name + '-' + kind)
        command = ['rustc', '--edition=2024', '-O', str(probe_dir / filename), '-L', 'dependency=' + str(variant['target'] / 'release/deps')]
        for crate in crates:
            command += ['--extern', crate + '=' + str(libraries[crate])]
        run(name + '-' + kind + '-build', command + ['-o', str(binary)])
        variant['binaries'][kind] = binary

trials = []
for index in range(1, 6):
    for name in (('baseline', 'candidate') if index % 2 else ('candidate', 'baseline')):
        text = run(f'{name}-operation-{index}', ['taskset', '-c', '0', '/usr/bin/time', '-v', str(variants[name]['binaries']['operation'])])
        rows = []
        for line in text.splitlines():
            if line.startswith('operation_'):
                label, count, nanos, witness = line.split('\t')
                rows.append({'name': label, 'iterations': int(count), 'elapsed_ns': int(nanos), 'witness': witness})
        assert len(rows) == 231
        trials.append({'variant': name, 'index': index, 'rows': rows})
identity = lambda trial: [{key: value for key, value in row.items() if key != 'elapsed_ns'} for row in trial['rows']]
canonical = identity(trials[0])
assert all(identity(trial) == canonical for trial in trials)
historical = read(ROOT / 'docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/cost/rows.json')
assert canonical == historical
(OUT / 'rows.json').write_text(json.dumps(canonical, sort_keys=True, separators=(',', ':')) + '\n')
report_trials = []
for index in [1, 2]:
    for name in (('baseline', 'candidate') if index == 1 else ('candidate', 'baseline')):
        destination = OUT / f'{name}-reports-{index}'
        run(f'{name}-reports-{index}', ['taskset', '-c', '0', '/usr/bin/time', '-v', str(variants[name]['binaries']['report']), str(inputs), str(destination)])
        report_trials.append({'variant': name, 'index': index})
reports = {}
for board in ['cparti-fpga', 'hvp109a']:
    paths = [OUT / f"{trial['variant']}-reports-{trial['index']}" / (board + '.json') for trial in report_trials]
    raw = paths[0].read_bytes()
    assert all(path.read_bytes() == raw for path in paths)
    retained = EVIDENCE / 'contact-index-directed-replay' / ('candidate-reports-1-' + board + '.json.gz')
    assert gzip.decompress(retained.read_bytes()) == raw
    data = json.loads(raw)
    assert len(data['violations']) == data['violation_count']
    for trial, path in zip(report_trials, paths):
        (OUT / f"{trial['variant']}-reports-{trial['index']}-{board}.json.gz").write_bytes(gzip.compress(path.read_bytes(), mtime=0))
    reports[board] = {'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest(), 'violation_count': data['violation_count'], 'summary': data['summary'], 'complete_bytes_identical_across_versions_repetitions_and_admitted_report': True}
verify()
assert inventory() == input_record['input_files']
manifest = {
    'scope': 'Production-only local exact-negative endpoint certificate reuse: 231 exact historical capability witnesses in five fresh CPU-0-pinned processes per variant, and complete twelve-check reports for both original boards in two fresh processes per variant. All report bytes also match the admitted contact-index checkpoint. No normalization or broad speedup claim.',
    'production_source_record': str(PRODUCTION / 'candidate-sources.json'),
    'baseline_source_record': str(PRODUCTION / 'baseline-sources.json'),
    'only_modified_runtime_source': source['only_runtime_change_from_admitted_source'],
    'source_unchanged': True, 'commands': commands, 'cost_trials': trials,
    'canonical_rows': info(OUT / 'rows.json'), 'historical_row_deltas': 0,
    'reports': reports, 'input_files': input_record['input_files'], 'archives': input_record['archives'],
    'variants': {name: {
        'source_directory': str(variant['source']),
        'libraries': {str(path): info(path) for path in variant['libraries'].values()},
        'binaries': {kind: {'path': str(path), **info(path)} for kind, path in variant['binaries'].items()},
    } for name, variant in variants.items()},
    'probe_sources': {path.name: info(path) for path in probe_dir.iterdir()},
}
(OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Production capability/report replay complete', flush=True)
