"""Isolate the reviewed bounding-box change against committed production inputs."""
import difflib
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
BASE = Path('/tmp/drc-area-compaction-final')
WORK = ROOT / 'target/bbox-initial-pair-production'
SOURCE, TARGET = WORK / 'source', WORK / 'target'
OUT = EVIDENCE / 'bbox-initial-pair-production'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base_record = EVIDENCE / 'area-compaction-final-source/sources.json'
base = read(base_record)
native_record = EVIDENCE / 'native-area-compaction-gate/sources.json'
native = read(native_record)
trial_record = EVIDENCE / 'bbox-initial-pair-candidate/sources.json'
trial = read(trial_record)
trial_source = ROOT / 'target/native-bbox-initial-pair/source'
assert all(digest(BASE / 'source' / name) == value for name, value in base['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(trial_source / name) == value for name, value in trial['source_hashes'].items())
revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
packages = read(EVIDENCE / 'area-compaction-admission/committed-source-audit.json')['package_roots']

def selected(name):
    for package in packages:
        prefix = '' if package == '.' else package + '/'
        if name in [prefix + suffix for suffix in ['Cargo.toml', 'Cargo.lock', 'build.rs']]: return True
        if name.endswith('.rs') and any(name.startswith(prefix + folder + '/') for folder in ['src', 'tests', 'benches', 'examples']): return True
    return False

entries = []
for entry in subprocess.check_output(['git', 'ls-tree', '-r', '-z', revision], cwd=ROOT).split(b'\0'):
    if not entry: continue
    metadata, name = entry.split(b'\t')
    name = name.decode()
    if selected(name): entries.append((name, metadata.decode().split()[2]))
raw = subprocess.check_output(['git', 'cat-file', '--batch'], cwd=ROOT, input=''.join(sha + '\n' for _, sha in entries).encode())
offset = 0
committed = {}
for name, sha in entries:
    end = raw.index(b'\n', offset)
    header = raw[offset:end].decode().split()
    assert header[:2] == [sha, 'blob']
    size = int(header[2]); content = raw[end + 1:end + 1 + size]
    committed[name] = hashlib.sha256(content).hexdigest()
    offset = end + 2 + size
assert offset == len(raw)
assert all(digest(BASE / 'source' / name) == value for name, value in committed.items())
fixture = 'crates/hyperdrc/tests/fixtures/native-circle-via.kicad_pcb'
fixture_bytes = subprocess.check_output(['git', 'show', revision + ':' + fixture], cwd=ROOT)
assert (BASE / 'source' / fixture).read_bytes() == fixture_bytes

OUT.mkdir()
WORK.mkdir()
for folder in ['source', 'target']:
    print('copy', folder, flush=True)
    with (OUT / ('copy-' + folder + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE / folder), str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
bbox = 'crates/hypercurve/src/bbox.rs'
regressions = 'crates/hypercurve/tests/bbox_initial_pair.rs'
path = SOURCE / bbox
text = path.read_text()
candidate = (trial_source / bbox).read_text()
start = "    pub fn from_points<'a, I>"
end = '    /// Constructs the bounding box of a finite line segment.'
a, b = text.index(start), text.index(end)
c, d = candidate.index(start), candidate.index(end)
text = text[:a] + candidate[c:d] + text[b:]
helper_start = 'fn ordered_coordinate_pair('
helper_end = 'fn include_coordinate('
a, b = candidate.index(helper_start), candidate.index(helper_end)
assert helper_start not in text
text = text.replace(helper_end, candidate[a:b] + helper_end, 1)
path.write_text(text)
shutil.copyfile(trial_source / regressions, SOURCE / regressions)
ENV = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_TARGET_DIR': str(TARGET), 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'RUSTDOCFLAGS': '-D warnings'}
with (OUT / 'prepare-format.log').open('w') as log:
    subprocess.run(['cargo', 'fmt', '--all'], cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
paths = sorted(set(base['source_hashes']) | {bbox, regressions, fixture})
initial = {name: digest(SOURCE / name) for name in paths}
expected = base['source_hashes'] | {fixture: hashlib.sha256(fixture_bytes).hexdigest()}
assert {name for name, value in initial.items() if expected.get(name) != value} == {bbox, regressions}
assert all(digest(SOURCE / name) == value for name, value in committed.items() if name != bbox)
# Every source input in the selected package directories must be accounted for.
actual = set()
for package in packages:
    prefix = SOURCE if package == '.' else SOURCE / package
    for folder in ['src', 'tests', 'benches', 'examples']:
        actual.update(str(path.relative_to(SOURCE)) for path in (prefix / folder).rglob('*.rs') if path.is_file())
assert actual == {name for name in committed if name.endswith('.rs') and not name.endswith('/build.rs') and name != 'build.rs'} | {regressions}
changes = [bbox, regressions]
with tarfile.open(OUT / 'source-delta.tar.gz', 'w:gz') as archive:
    for name in changes: archive.add(SOURCE / name, arcname=name)
patch = []
for name in changes:
    before = (BASE / 'source' / name).read_text() if (BASE / 'source' / name).exists() else ''
    patch.extend(difflib.unified_diff(before.splitlines(keepends=True), (SOURCE / name).read_text().splitlines(keepends=True), fromfile='a/' + name, tofile='b/' + name))
(OUT / 'source-delta.patch').write_text(''.join(patch))
record = {'scope': 'Only bounding-box first-pair initialization, singleton point reuse and two regression tests on committed production source. Broader native-stroke prototype changes are excluded.', 'production_baseline_revision': revision, 'base_source_record': str(base_record), 'main_source_record': str(native_record), 'only_modified_sources': changes, 'source_hashes': initial, 'source_unchanged': True, 'environment': {key: ENV[key] for key in ['CARGO_HOME', 'CARGO_TARGET_DIR', 'CCACHE_DIR', 'CARGO_NET_OFFLINE', 'RUSTDOCFLAGS']}}
(OUT / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
audit = {'revision': revision, 'package_roots': packages, 'checked_committed_inputs': len(committed), 'all_baseline_inputs_match_git': True, 'candidate_existing_differences': [bbox], 'candidate_new_sources': [regressions], 'all_package_rust_inputs_accounted_for': True, 'fixture_sha256': hashlib.sha256(fixture_bytes).hexdigest(), 'committed_source_hashes': committed}
(OUT / 'committed-source-audit.json').write_text(json.dumps(audit, indent=2) + '\n')
shutil.copyfile(__file__, OUT / 'reproduce.py')
status = []

def verify():
    assert all(digest(SOURCE / name) == value for name, value in initial.items())
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())

def run(name, command):
    verify(); start = time.monotonic(); print('start', name, flush=True)
    with (OUT / (name + '.log')).open('w') as log:
        result = subprocess.run(command, cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify(); print('finish', name, result.returncode, flush=True)
    if result.returncode: raise SystemExit(result.returncode)

run('format', ['cargo', 'fmt', '--all', '--', '--check'])
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
(OUT / 'binary.json').write_text(json.dumps({'path': executables[0], 'sha256': digest(Path(executables[0]))}, indent=2) + '\n')
run('drc-tests', [executables[0], '--test-threads=2'])
run('drc-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-targets', '--', '-D', 'warnings'])
run('full-board-smoke', ['/usr/bin/time', '-v', '-o', str(OUT / 'full-board-smoke-time.log'), executables[0], 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'])
log = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log and log.count('hyperdrc: finished emit report') == 2
print('Isolated production bounding-box validation complete', flush=True)
