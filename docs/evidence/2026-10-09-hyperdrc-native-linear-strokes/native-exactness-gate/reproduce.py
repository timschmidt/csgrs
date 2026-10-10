"""Recheck the complete native prototype after the two admitted exactness repairs."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
BASE = EVIDENCE / 'native-bbox-initial-pair-gate'
OUT = EVIDENCE / 'native-exactness-gate'
OUT.mkdir()
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base = read(BASE / 'sources.json')
paths = set(base['source_hashes'])
for package in ['hyperreal', 'hyperlimit', 'hyperlattice', 'hypersolve', 'hypercurve', 'hyperdrc', 'hypermesh', 'hyperpath', 'hyperphysics', 'hypertri']:
    for path in (ROOT / 'crates' / package).rglob('*'):
        if path.is_file() and 'target' not in path.parts and (path.suffix == '.rs' or path.name in ['Cargo.toml', 'Cargo.lock'] or 'fixtures' in path.parts):
            paths.add(str(path.relative_to(ROOT)))
initial = {name: digest(ROOT / name) for name in sorted(paths)}
changes = [name for name, value in initial.items() if base['source_hashes'].get(name) != value]
assert set(changes) == {
    'crates/hypercurve/src/bezier_region.rs',
    'crates/hypercurve/src/bezier_region/ray_filter_tests.rs',
    'crates/hypercurve/tests/ray_filter_extreme_coordinates.rs',
    'crates/hypercurve/src/curve_region_boolean.rs',
    'crates/hypercurve/src/curve_region_boolean/contact_index_tests.rs',
}
for folder in ['ray-filter-admission', 'contact-index-admission']:
    for name, record in read(EVIDENCE / folder / 'admission.json')['files'].items():
        assert initial[name] == record['worktree_after_sha256']
with tarfile.open(OUT / 'source-delta.tar.gz', 'w:gz') as archive:
    for name in changes:
        archive.add(ROOT / name, arcname=name)
ENV = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(ROOT / 'target')}
record = {
    'scope': 'Current complete native-stroke prototype after separately admitted ray-filter and contact-index exactness repairs. Recheck the eight new regressions and the original unchanged complete-board gate. No broad native admission, reused full-suite claim or matched speedup claim.',
    'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'base_source_record': str(BASE / 'sources.json'),
    'base_source_record_sha256': digest(BASE / 'sources.json'),
    'source_hashes': initial,
    'delta_files': changes,
    'source_delta_sha256': digest(OUT / 'source-delta.tar.gz'),
    'unchanged': True,
    'environment': {key: ENV.get(key) for key in ['CARGO_HOME', 'CCACHE_DIR', 'CARGO_NET_OFFLINE', 'CARGO_TARGET_DIR', 'RUSTFLAGS', 'CARGO_PROFILE_RELEASE_DEBUG']},
}
(OUT / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
shutil.copyfile(__file__, OUT / 'reproduce.py')
status = []


def verify():
    assert all(digest(ROOT / name) == value for name, value in initial.items())


def run(name, command):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.log')).open('w') as log:
        result = subprocess.run(command, cwd=ROOT, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)


run('compiler', ['rustc', '-vV'])
run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('contact-index-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', 'contact_index', '--', '--test-threads=2'])
run('ray-filter-unit', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', 'ray_filter', '--', '--test-threads=2'])
run('ray-filter-public', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--test', 'ray_filter_extreme_coordinates', '--', '--test-threads=2'])
run('build-board-smoke', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--no-run', '--message-format=json'])
executables = []
for line in (OUT / 'build-board-smoke.log').read_text().splitlines():
    if line.startswith('{'):
        event = json.loads(line)
        if event.get('reason') == 'compiler-artifact' and event.get('executable') and event['target']['name'] == 'hyperdrc':
            executables.append(event['executable'])
assert len(executables) == 1
(OUT / 'binary.json').write_text(json.dumps({'path': executables[0], 'sha256': digest(Path(executables[0]))}, indent=2) + '\n')
run('full-board-smoke', ['/usr/bin/time', '-v', '-o', str(OUT / 'full-board-smoke-time.log'), executables[0], 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'])
log = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log and log.count('hyperdrc: finished emit report') == 2
print('Original native-prototype board gate passed', flush=True)
