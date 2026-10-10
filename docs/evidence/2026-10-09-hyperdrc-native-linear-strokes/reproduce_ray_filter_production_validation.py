"""Validate the frozen ray-filter repair after focused exactness checks."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
CANDIDATE = EVIDENCE / 'ray-filter-finite-guard'
OUT = EVIDENCE / 'ray-filter-production-validation'
SOURCE = ROOT / 'target/ray-filter-finite-guard/source'
TARGET = ROOT / 'target/ray-filter-finite-guard/target'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(CANDIDATE / 'sources.json')
native = read(Path(source['main_source_record']))
focused = read(CANDIDATE / 'status.json')
assert len(focused) == 3 and all(command['exit_code'] == 0 for command in focused)
OUT.mkdir()
shutil.copyfile(__file__, OUT / 'reproduce.py')
ENV = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_TARGET_DIR': str(TARGET), 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'RUSTDOCFLAGS': '-D warnings'}
status = []

def verify():
    assert all(digest(SOURCE / name) == value for name, value in source['source_hashes'].items())
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())

def run(name, command):
    verify(); start = time.monotonic(); print('start', name, flush=True)
    with (OUT / (name + '.log')).open('w') as log:
        result = subprocess.run(command, cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic()-start})
    (OUT / 'status.json').write_text(json.dumps(status, indent=2)+'\n')
    verify(); print('finish', name, result.returncode, flush=True)
    if result.returncode: raise SystemExit(result.returncode)

run('curve-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', '--tests', '--', '--test-threads=2'])
run('curve-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--all-targets', '--', '-D', 'warnings'])
run('curve-doc', ['cargo', 'doc', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', '--no-deps'])
run('build-drc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--no-run', '--message-format=json'])
executables = []
for line in (OUT / 'build-drc-tests.log').read_text().splitlines():
    if line.startswith('{'):
        event = json.loads(line)
        if event.get('reason') == 'compiler-artifact' and event.get('executable') and event['target']['name'] == 'hyperdrc': executables.append(event['executable'])
assert len(executables) == 1
(OUT / 'binary.json').write_text(json.dumps({'path': executables[0], 'sha256': digest(Path(executables[0]))}, indent=2)+'\n')
run('drc-tests', [executables[0], '--test-threads=2'])
run('drc-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-targets', '--', '-D', 'warnings'])
run('full-board-smoke', ['/usr/bin/time', '-v', '-o', str(OUT / 'full-board-smoke-time.log'), executables[0], 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'])
log = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log and log.count('hyperdrc: finished emit report') == 2
(OUT / 'source-verification.json').write_text(json.dumps({'source_record': str(CANDIDATE / 'sources.json'), 'source_record_sha256': digest(CANDIDATE / 'sources.json'), 'all_candidate_and_main_source_hashes_unchanged': True, 'scope': 'Complete release curve and DRC suites, strict lints/docs and original full-board gate on isolated production source; not complete workspace/pre-push validation.'}, indent=2)+'\n')
print('Production ray-filter validation complete', flush=True)
