"""Broaden validation of the root-entry candidate, then run the original board gate."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
CANDIDATE = EVIDENCE / 'laurent-root-memo-omission'
OUT = EVIDENCE / 'laurent-root-native-validation'
SOURCE = ROOT / 'target/laurent-root-memo-omission/candidate/source'
TARGET = ROOT / 'target/laurent-root-memo-omission/candidate/target'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(CANDIDATE / 'candidate-sources.json')
native = read(Path(source['native_source_record']))
assert read(EVIDENCE / 'diagnostics-64.json')['decision'].startswith('advance unused root memo omission')
retained = read(EVIDENCE / 'laurent-root-memo-omission-cost/retained-libraries.json')
assert all(digest(Path(row['retained_path'])) == row['sha256'] for row in retained['libraries'].values())
OUT.mkdir()
shutil.copyfile(__file__, OUT / 'reproduce.py')
ENV = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(TARGET)}
status = []


def verify():
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
    assert all(digest(SOURCE / name) == value for name, value in source['source_hashes'].items())


def run(name, command):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.log')).open('w') as log:
        result = subprocess.run(command, cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)


run('scalar-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--features', 'serde', '--lib', '--tests', '--', '--test-threads=2'])
run('curve-focused', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation,hyperreal/default,hyperreal/serde', '--test', 'contour_area_normal_form', '--test', 'native_linear_stroke', '--test', 'ray_filter_extreme_coordinates', '--', '--test-threads=2'])
run('curve-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation,hyperreal/default,hyperreal/serde', '--all-targets', '--', '-D', 'warnings'])
run('build-board-smoke', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--no-run', '--message-format=json'])
executables = []
for line in (OUT / 'build-board-smoke.log').read_text().splitlines():
    if line.startswith('{'):
        event = json.loads(line)
        if event.get('reason') == 'compiler-artifact' and event.get('executable') and event['target']['name'] == 'hyperdrc':
            executables.append(event['executable'])
assert len(executables) == 1
(OUT / 'binary.json').write_text(json.dumps({'path': executables[0], 'sha256': digest(Path(executables[0]))}, indent=2) + '\n')
run('drc-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-targets', '--', '-D', 'warnings'])
run('full-board-smoke', ['/usr/bin/time', '-v', '-o', str(OUT / 'full-board-smoke-time.log'), executables[0], 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'])
log = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log and log.count('hyperdrc: finished emit report') == 2
print('Broader validation and original board gate passed', flush=True)
