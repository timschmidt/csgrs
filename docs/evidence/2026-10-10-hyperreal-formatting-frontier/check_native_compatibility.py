"""Freeze and validate checked formatting alongside the existing native prototype."""
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
FROZEN = EVIDENCE / 'checked-native'
WORK = ROOT / 'target/hyperreal-checked-native'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
info = lambda path: {'bytes': path.stat().st_size, 'sha256': digest(path)}
assert sys.argv[1] in ['prepare', 'validate']

if sys.argv[1] == 'prepare':
    assert read(EVIDENCE / 'downstream-checked-scientific.json')['historical_witnesses'] == 231
    parent_path = PREVIOUS / 'arc-endpoint-native/baseline-sources.json'
    parent = read(parent_path)
    path = Path(parent['source_hashes_file'])
    assert digest(path) == parent['source_hashes_file_sha256']
    original = json.loads(gzip.decompress(path.read_bytes()))
    native = read(Path(parent['native_source_record']))['source_hashes']
    base = Path(parent['source_directory'])
    assert all(digest(base / name) == value for name, value in original.items())
    assert all(digest(ROOT / name) == value for name, value in native.items())
    FROZEN.mkdir()
    WORK.mkdir()
    for before, after in [(base, WORK / 'source'), (Path(parent['target_directory']), WORK / 'target')]:
        print('copy', before, 'to', after, flush=True)
        subprocess.run(['cp', '-a', '--reflink=auto', str(before), str(after)], check=True)
    patch = EVIDENCE / 'checked-scientific/candidate.patch'
    source = WORK / 'source'
    with (FROZEN / 'apply.log').open('w') as log:
        subprocess.run(['patch', '--batch', '--fuzz=0', '--no-backup-if-mismatch', '--dry-run', '-p1', '-i', str(patch)], cwd=source, stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run(['patch', '--batch', '--fuzz=0', '--no-backup-if-mismatch', '-p1', '-i', str(patch)], cwd=source, stdout=log, stderr=subprocess.STDOUT, check=True)
    candidate = read(EVIDENCE / 'checked-scientific/sources.json')
    expected = candidate['changed_files']
    hashes = {name: digest(source / name) for name in sorted(set(original) | set(expected))}
    assert {name for name in hashes if original.get(name) != hashes[name]} == set(expected)
    compressed = FROZEN / 'source-hashes.json.gz'
    compressed.write_bytes(gzip.compress((json.dumps(hashes, indent=2) + '\n').encode(), mtime=0))
    record = {
        'scope': 'Compatibility with the existing unadmitted native prototype; only the first checked-formatter patch is added. This does not admit native geometry or repair its known 120-second gate failure.',
        'parent_source_record': str(parent_path), 'parent_source_record_sha256': digest(parent_path),
        'source_directory': str(source), 'target_directory': str(WORK / 'target'),
        'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
        'source_files': len(hashes), 'changed_files': expected,
        'patch': str(patch), 'patch_sha256': digest(patch),
        'main_source_record': parent['native_source_record'], 'main_source_record_sha256': parent['native_source_record_sha256'],
        'main_runtime_unchanged': True,
    }
    (FROZEN / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
    shutil.copyfile(__file__, FROZEN / 'prepare.py')
    assert all(digest(base / name) == value for name, value in original.items())
    assert all(digest(ROOT / name) == value for name, value in native.items())
    print('Frozen native compatibility source:', len(hashes), 'inputs;', len(expected), 'formatter files differ.', flush=True)
    raise SystemExit(0)

record = read(FROZEN / 'sources.json')
path = Path(record['source_hashes_file'])
assert digest(path) == record['source_hashes_file_sha256']
hashes = json.loads(gzip.decompress(path.read_bytes()))
native = read(Path(record['main_source_record']))['source_hashes']
source, target = Path(record['source_directory']), Path(record['target_directory'])
OUT = EVIDENCE / 'checked-native-validation'
OUT.mkdir()
shutil.copyfile(__file__, OUT / 'validate.py')
shutil.copyfile(EVIDENCE / 'checked_frontier_probe.rs', OUT / 'probe.rs')
probe_hash = digest(OUT / 'probe.rs')
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_TARGET_DIR': str(target), 'RUSTDOCFLAGS': '-D warnings'}
status = []


def verify():
    assert all(digest(source / name) == value for name, value in hashes.items())
    assert all(digest(ROOT / name) == value for name, value in native.items())
    assert digest(OUT / 'probe.rs') == probe_hash


def run(name, command, tests=False, expected=None):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    path = OUT / (name + '.log')
    with path.open('w') as log:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=5s', '1200', *command], cwd=source, env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
    row = {'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start}
    status.append(row)
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name
    raw = path.read_text()
    if tests:
        counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', raw)
        assert counts and sum(int(item[0]) for item in counts) > 0
        row['tests_passed'] = sum(int(item[0]) for item in counts)
        row['tests_ignored'] = sum(int(item[2]) for item in counts)
        assert all(int(item[1]) == 0 for item in counts)
        if expected is not None:
            assert row['tests_passed'] == expected and row['tests_ignored'] == 0
        (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    return raw


run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('scalar-default-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--lib', '--tests', '--', '--test-threads=2'], tests=True)
run('scalar-all-feature-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--tests', '--', '--test-threads=2'], tests=True)
run('scalar-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--all-targets', '--', '-D', 'warnings'])
run('scalar-rustdoc', ['cargo', 'doc', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--no-deps'])
run('scalar-doc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--doc', '--', '--test-threads=2'], tests=True)
common = ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation']
run('point-regressions', common + ['--lib', 'point::', '--', '--test-threads=2'], tests=True, expected=14)
run('segment-regressions', common + ['--lib', 'segment::', '--', '--test-threads=2'], tests=True, expected=7)
run('curve-integrations', common + ['--test', 'hypercurve_arc_bezier', '--test', 'hypercurve_circle_predicates', '--test', 'native_linear_stroke', '--', '--test-threads=2'], tests=True, expected=25)
run('drc-native-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--test', 'native_linear_strokes', '--', '--test-threads=2'], tests=True, expected=26)
run('curve-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--all-targets', '--', '-D', 'warnings'])
run('drc-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-targets', '--', '-D', 'warnings'])
build = run('default-library-build', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '--message-format=json'])
events = [json.loads(line) for line in build.splitlines() if line.startswith('{')]
libraries = [Path(filename) for event in events if event.get('reason') == 'compiler-artifact' and event['target']['name'] == 'hyperreal' for filename in event['filenames'] if filename.endswith('.rlib')]
assert len(libraries) == 1
library = libraries[0]
retained = WORK / 'libhyperreal_checked_native.rlib'
shutil.copyfile(library, retained)
binary = WORK / 'frontier'
run('probe-build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs'), '-L', 'dependency=' + str(target / 'release/deps'), '--extern', 'hyperreal=' + str(retained), '-o', str(binary)])
baseline_path = EVIDENCE / 'checked-frontier/canonical-receipts.json.gz'
assert digest(baseline_path) == read(EVIDENCE / 'checked-frontier.json')['artifacts']['checked-frontier/canonical-receipts.json.gz']['sha256']
baseline = json.loads(gzip.decompress(baseline_path.read_bytes()))
key = lambda row: (row['family'], row['stage'], row['places'], row['floor'], row['case'])
differences = []
for order in ['forward', 'reverse']:
    raw = run(order, [str(binary), order])
    rows = sorted([json.loads(line) for line in raw.splitlines() if line.startswith('{')], key=key)
    assert len(rows) == len(baseline) == 3816 and [key(row) for row in rows] == [key(row) for row in baseline]
    differences.extend({'order': order, 'production': old, 'native': new} for old, new in zip(baseline, rows) if old != new)
(OUT / 'receipt-differences.json').write_text(json.dumps(differences, indent=2) + '\n')
assert not differences
verify()
summary = {
    'decision': 'checked formatter is compatible with the frozen current native prototype in the tested scope; no native admission',
    'source_record': str(FROZEN / 'sources.json'), 'source_record_sha256': digest(FROZEN / 'sources.json'),
    'source_files': len(hashes), 'main_files': len(native), 'all_source_hashes_unchanged': True,
    'steps': status, 'native_focused_test_invocations': 72,
    'rows_each_process': 3816, 'processes': 2, 'differences_from_production_checked_candidate': 0,
    'library': {'path': str(retained), **info(retained)}, 'binary': {'path': str(binary), **info(binary)},
    'limitations': 'This compatibility check does not repeat full native board reports, change its known 120-second gate failure, admit the native prototype, or complete the workspace/pre-push matrix or property plan.',
}
summary['artifacts'] = {str(path.relative_to(EVIDENCE)): info(path) for path in sorted(OUT.iterdir()) if path.is_file()}
summary['script_sha256'] = digest(Path(__file__))
(EVIDENCE / 'checked-native-validation.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Native compatibility and 3816 checked receipts per process pass.', flush=True)
