"""Full scoped production checks after the favorable endpoint resource gate."""
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
PRODUCTION = EVIDENCE / 'arc-endpoint-certificate'
OUT = EVIDENCE / 'arc-endpoint-production-validation'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert read(EVIDENCE / 'diagnostics-78.json')['decision'].startswith('early production resource gate passed;')
records = {variant: read(PRODUCTION / (variant + '-sources.json')) for variant in ['baseline', 'candidate']}
maps = {}
for variant, record in records.items():
    path = Path(record['source_hashes_file'])
    assert digest(path) == record['source_hashes_file_sha256']
    maps[variant] = json.loads(gzip.decompress(path.read_bytes()))
native = read(EVIDENCE / 'native-exactness-gate/sources.json')['source_hashes']
record = records['candidate']
SOURCE, TARGET = Path(record['source_directory']), Path(record['target_directory'])
OUT.mkdir()
shutil.copyfile(__file__, OUT / 'reproduce.py')
ENV = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_TARGET_DIR': str(TARGET), 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'RUSTDOCFLAGS': '-D warnings'}
status = []


def verify():
    for variant, source_record in records.items():
        source = Path(source_record['source_directory'])
        assert all(digest(source / name) == value for name, value in maps[variant].items())
    assert all(digest(ROOT / name) == value for name, value in native.items())


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


run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('curve-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--all-targets', '--', '-D', 'warnings'])
run('curve-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', '--tests', '--', '--test-threads=2'])
run('curve-doc', ['cargo', 'doc', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', '--no-deps'])
run('build-drc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--no-run', '--message-format=json'])
executables = []
for line in (OUT / 'build-drc-tests.log').read_text().splitlines():
    if line.startswith('{'):
        event = json.loads(line)
        if event.get('reason') == 'compiler-artifact' and event.get('executable') and event['target']['name'] == 'hyperdrc':
            executables.append(event['executable'])
assert len(executables) == 1
(OUT / 'binary.json').write_text(json.dumps({'path': executables[0], 'sha256': digest(Path(executables[0]))}, indent=2) + '\n')
run('drc-tests', [executables[0], '--test-threads=2'])
run('drc-integration-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--test', 'kicad_pad_sources', '--test', 'readme', '--', '--test-threads=2'])
run('drc-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-targets', '--', '-D', 'warnings'])
run('drc-doc', ['cargo', 'doc', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-deps'])
run('full-board-smoke', ['/usr/bin/time', '-v', '-o', str(OUT / 'full-board-smoke-time.log'), executables[0], 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'])
raw = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in raw
assert raw.count('hyperdrc: finished emit report') == 2
verify()
(OUT / 'source-verification.json').write_text(json.dumps({'source_record': str(PRODUCTION / 'candidate-sources.json'), 'source_record_sha256': digest(PRODUCTION / 'candidate-sources.json'), 'production_files_each': len(maps['candidate']), 'main_files': len(native), 'all_production_and_main_source_hashes_unchanged': True, 'scope': 'Complete release curve and DRC library/integration suites, strict lints/docs and original full-board gate on isolated production source. Scalar sources are unchanged; no fresh scalar suite or full workspace/pre-push matrix is claimed.'}, indent=2) + '\n')
print('Endpoint production validation complete; historical witnesses and reports remain required.', flush=True)
