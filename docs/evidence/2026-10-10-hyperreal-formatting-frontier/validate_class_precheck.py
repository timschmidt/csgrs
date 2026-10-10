"""Validate the frozen class precheck and compare every prior checked receipt."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'class-precheck-validation'
WORK = ROOT / 'target/hyperreal-class-precheck-frontier'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
record_path = EVIDENCE / 'class-precheck/sources.json'
record = read(record_path)
parent = read(Path(record['parent_source_record']))
sources = {'first': parent, 'class_precheck': record}
maps = {}
for variant, source in sources.items():
    path = Path(source['source_hashes_file'])
    assert digest(path) == source['source_hashes_file_sha256']
    maps[variant] = json.loads(gzip.decompress(path.read_bytes()))
native = read(Path(record['main_source_record']))['source_hashes']
baseline_record = read(EVIDENCE / 'checked-frontier.json')
baseline_path = EVIDENCE / 'checked-frontier/canonical-receipts.json.gz'
assert digest(baseline_path) == baseline_record['artifacts']['checked-frontier/canonical-receipts.json.gz']['sha256']
baseline = json.loads(gzip.decompress(baseline_path.read_bytes()))
assert len(baseline) == 3816
OUT.mkdir()
WORK.mkdir()
shutil.copyfile(__file__, OUT / 'validate.py')
shutil.copyfile(EVIDENCE / 'checked_frontier_probe.rs', OUT / 'probe.rs')
probe_hash = digest(OUT / 'probe.rs')
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_TARGET_DIR': record['target_directory'], 'RUSTDOCFLAGS': '-D warnings'}
status = []


def verify():
    for variant, source in sources.items():
        assert all(digest(Path(source['source_directory']) / name) == value for name, value in maps[variant].items())
    assert all(digest(ROOT / name) == value for name, value in native.items())
    assert digest(OUT / 'probe.rs') == probe_hash


def run(name, command, tests=False, cwd=None):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    stdout_path = OUT / (name + '.stdout')
    stderr_path = OUT / (name + '.stderr')
    with stdout_path.open('w') as stdout, stderr_path.open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=5s', '600', *command], cwd=cwd or Path(record['source_directory']), env=env, stdout=stdout, stderr=stderr, check=False)
    row = {'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start}
    status.append(row)
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name
    if tests:
        counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', stdout_path.read_text())
        assert counts and sum(int(item[0]) for item in counts) > 0
        row['test_binaries'] = len(counts)
        row['tests_passed'] = sum(int(item[0]) for item in counts)
        row['tests_ignored'] = sum(int(item[2]) for item in counts)
        assert all(int(item[1]) == 0 for item in counts)
        (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    return stdout_path.read_text()


run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('default-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--lib', '--tests', '--', '--test-threads=2'], tests=True)
run('all-feature-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--tests', '--', '--test-threads=2'], tests=True)
run('all-feature-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--all-targets', '--', '-D', 'warnings'])
run('all-feature-rustdoc', ['cargo', 'doc', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--no-deps'])
run('all-feature-doc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--doc', '--', '--test-threads=2'], tests=True)
build = run('default-library-build', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '--message-format=json'])
artifacts = [json.loads(line) for line in build.splitlines() if line.startswith('{')]
libraries = [path for item in artifacts if item.get('reason') == 'compiler-artifact' and item['target']['name'] == 'hyperreal' for path in item['filenames'] if path.endswith('.rlib')]
assert len(libraries) == 1
library = Path(libraries[0])
retained = WORK / 'libhyperreal_class_precheck.rlib'
shutil.copyfile(library, retained)
library_hash = digest(library)
(OUT / 'library.json').write_text(json.dumps({'path': str(library), 'retained_path': str(retained), 'sha256': library_hash, 'bytes': library.stat().st_size}, indent=2) + '\n')
run('probe-format', ['rustfmt', '--edition', '2024', '--check', str(OUT / 'probe.rs')], cwd=ROOT)
binary = WORK / 'probe'
run('probe-build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs'), '-L', 'dependency=' + str(Path(record['target_directory']) / 'release/deps'), '--extern', 'hyperreal=' + str(retained), '-o', str(binary)], cwd=ROOT)
binary_hash = digest(binary)
(OUT / 'binary.json').write_text(json.dumps({'path': str(binary), 'sha256': binary_hash, 'bytes': binary.stat().st_size}, indent=2) + '\n')
key = lambda row: (row['family'], row['stage'], row['places'], row['floor'], row['case'])
differences = []
for order in ['forward', 'reverse']:
    raw = run(order, [str(binary), order], cwd=ROOT)
    rows = sorted([json.loads(line) for line in raw.splitlines()], key=key)
    assert len(rows) == 3816 and [key(row) for row in rows] == [key(row) for row in baseline]
    differences.extend({'order': order, 'first': old, 'class_precheck': new} for old, new in zip(baseline, rows) if old != new)
(OUT / 'receipt-differences.json').write_text(json.dumps(differences, indent=2) + '\n')
assert not differences, 'Preserve and inspect all differing receipts before any measurement.'
verify()
assert digest(library) == digest(retained) == library_hash
assert digest(binary) == binary_hash
summary = {
    'decision': 'exact-class precheck passes scalar suites and all prior checked receipts; matched resource and downstream gates remain',
    'source_record': str(record_path), 'source_record_sha256': digest(record_path),
    'steps': status, 'rows_each_process': 3816, 'processes': 2, 'differences_from_first_candidate': 0,
    'independent_reference': 'All receipts match the first candidate canonical record, whose returned strings were checked against exact rational reference bounds. The original raw receipts and oracle remain linked by content hash.',
    'reference_receipts': str(baseline_path), 'reference_receipts_sha256': digest(baseline_path),
    'all_candidate_parent_and_main_source_hashes_unchanged': True,
    'source_counts': {variant: len(values) for variant, values in maps.items()}, 'main_files': len(native),
    'limitations': 'No universal cache equivalence, arbitrary representation coverage, total-work bound, downstream compatibility, whole-workspace validation or runtime admission is claimed. Existing formatting traits remain unchanged.',
}
summary['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted(OUT.iterdir()) if path.is_file()}
summary['script_sha256'] = digest(Path(__file__))
(EVIDENCE / 'class-precheck-validation.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Scalar checks and 3816 receipts per process pass; no receipt differences.', flush=True)
