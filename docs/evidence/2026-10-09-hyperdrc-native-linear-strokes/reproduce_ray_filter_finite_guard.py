"""Isolate finite-intermediate ray rejection against committed production."""
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
BASE = ROOT / 'target/bbox-initial-pair-production'
WORK = ROOT / 'target/ray-filter-finite-guard'
SOURCE, TARGET = WORK / 'source', WORK / 'target'
OUT = EVIDENCE / 'ray-filter-finite-guard'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base_record = EVIDENCE / 'bbox-initial-pair-production/sources.json'
native_record = EVIDENCE / 'native-bbox-initial-pair-gate/sources.json'
base, native = read(base_record), read(native_record)
assert all(digest(BASE / 'source' / name) == value for name, value in base['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
packages = read(EVIDENCE / 'bbox-initial-pair-production/committed-source-audit.json')['package_roots']

def selected(name):
    for package in packages:
        prefix = '' if package == '.' else package + '/'
        if name in [prefix + suffix for suffix in ['Cargo.toml', 'Cargo.lock', 'build.rs']]: return True
        if name.endswith('.rs') and any(name.startswith(prefix + folder + '/') for folder in ['src', 'tests', 'benches', 'examples']): return True
    return False

entries = []
for entry in subprocess.check_output(['git', 'ls-tree', '-r', '-z', revision], cwd=ROOT).split(b'\0'):
    if not entry: continue
    metadata, name = entry.split(b'\t'); name = name.decode()
    if selected(name): entries.append((name, metadata.decode().split()[2]))
raw = subprocess.check_output(['git', 'cat-file', '--batch'], cwd=ROOT, input=''.join(sha + '\n' for _, sha in entries).encode())
offset, committed = 0, {}
for name, sha in entries:
    end = raw.index(b'\n', offset); header = raw[offset:end].decode().split()
    assert header[:2] == [sha, 'blob']
    size = int(header[2]); content = raw[end + 1:end + 1 + size]
    committed[name] = hashlib.sha256(content).hexdigest(); offset = end + 2 + size
assert offset == len(raw)
assert all(digest(BASE / 'source' / name) == value for name, value in committed.items())
fixture = 'crates/hyperdrc/tests/fixtures/native-circle-via.kicad_pcb'
fixture_bytes = subprocess.check_output(['git', 'show', revision + ':' + fixture], cwd=ROOT)
assert (BASE / 'source' / fixture).read_bytes() == fixture_bytes
OUT.mkdir(); WORK.mkdir()
for folder in ['source', 'target']:
    print('copy', folder, flush=True)
    with (OUT / ('copy-' + folder + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE / folder), str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
implementation = 'crates/hypercurve/src/bezier_region.rs'
unit = 'crates/hypercurve/src/bezier_region/ray_filter_tests.rs'
regressions = 'crates/hypercurve/tests/ray_filter_extreme_coordinates.rs'
path = SOURCE / implementation; text = path.read_text()
before = '''        let forward = dx * delta_x + dy * delta_y;
        if side >= -error {'''
after = '''        let forward = dx * delta_x + dy * delta_y;
        // Finite enclosures can still overflow in midpoint or projection
        // arithmetic. NaN comparisons cannot certify exclusion: retain the
        // fragment for the exact predicate whenever this filter overflows.
        if !error.is_finite() || !side.is_finite() || !forward.is_finite() {
            return false;
        }
        if side >= -error {'''
assert text.count(before) == 1
text = text.replace(before, after)
before = '#[cfg(test)]\nmod tests;'
assert text.count(before) == 1
text = text.replace(before, '#[cfg(test)]\nmod ray_filter_tests;\n\n' + before)
path.write_text(text)
shutil.copyfile(EVIDENCE / 'ray_filter_tests.rs', SOURCE / unit)
shutil.copyfile(EVIDENCE / 'ray_filter_extreme_coordinates.rs', SOURCE / regressions)
ENV = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_TARGET_DIR': str(TARGET), 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'RUSTDOCFLAGS': '-D warnings'}
with (OUT / 'prepare-format.log').open('w') as log:
    subprocess.run(['cargo', 'fmt', '--all'], cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
paths = sorted(set(base['source_hashes']) | {implementation, unit, regressions, fixture})
initial = {name: digest(SOURCE / name) for name in paths}
changes = [implementation, unit, regressions]
assert {name for name, value in initial.items() if base['source_hashes'].get(name) != value} == set(changes)
assert all(digest(SOURCE / name) == value for name, value in committed.items() if name != implementation)
actual = set()
for package in packages:
    prefix = SOURCE if package == '.' else SOURCE / package
    for folder in ['src', 'tests', 'benches', 'examples']:
        actual.update(str(path.relative_to(SOURCE)) for path in (prefix / folder).rglob('*.rs') if path.is_file())
assert actual == {name for name in committed if name.endswith('.rs') and not name.endswith('/build.rs') and name != 'build.rs'} | {unit, regressions}
with tarfile.open(OUT / 'source-delta.tar.gz', 'w:gz') as archive:
    for name in changes: archive.add(SOURCE / name, arcname=name)
patch = []
for name in changes:
    before = (BASE / 'source' / name).read_text() if (BASE / 'source' / name).exists() else ''
    patch.extend(difflib.unified_diff(before.splitlines(keepends=True), (SOURCE / name).read_text().splitlines(keepends=True), fromfile='a/' + name if before else '/dev/null', tofile='b/' + name))
(OUT / 'source-delta.patch').write_text(''.join(patch))
record = {'scope': 'Reject no ray box using non-finite floating-point intermediates; retain exact fallback. One private production guard, two private unit tests and two public regression tests. Broader native-stroke prototype excluded.', 'production_baseline_revision': revision, 'base_source_record': str(base_record), 'main_source_record': str(native_record), 'only_modified_sources': changes, 'source_hashes': initial, 'source_unchanged': True, 'environment': {key: ENV[key] for key in ['CARGO_HOME', 'CARGO_TARGET_DIR', 'CCACHE_DIR', 'CARGO_NET_OFFLINE', 'RUSTDOCFLAGS']}}
(OUT / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
(OUT / 'committed-source-audit.json').write_text(json.dumps({'revision': revision, 'package_roots': packages, 'checked_committed_inputs': len(committed), 'all_baseline_inputs_match_git': True, 'candidate_existing_differences': [implementation], 'candidate_new_sources': [unit, regressions], 'all_package_rust_inputs_accounted_for': True, 'fixture_sha256': hashlib.sha256(fixture_bytes).hexdigest(), 'committed_source_hashes': committed}, indent=2) + '\n')
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
run('ray-filter-unit-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', 'ray_filter_tests', '--', '--test-threads=1'])
run('ray-filter-public-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--test', 'ray_filter_extreme_coordinates', '--', '--test-threads=1'])
print('Focused production ray-filter checks complete', flush=True)
