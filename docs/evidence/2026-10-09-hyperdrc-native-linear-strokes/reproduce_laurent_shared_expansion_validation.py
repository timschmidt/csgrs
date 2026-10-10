"""Validate shared compound expansion against the unchanged, verified production baseline."""
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
OUT = EVIDENCE / 'laurent-shared-expansion'
PREVIOUS = EVIDENCE / 'laurent-memo-index'
SOURCE = ROOT / 'target/laurent-shared-expansion/candidate/source'
TARGET = ROOT / 'target/laurent-shared-expansion/candidate/target'
BASE = ROOT / 'target/laurent-memo-index/baseline/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
records = {name: read(OUT / (name + '-sources.json')) for name in ['baseline', 'candidate']}
hashes = {}
for name, record in records.items():
    assert digest(Path(record['source_hashes_file'])) == record['source_hashes_file_sha256']
    hashes[name] = json.loads(gzip.decompress(Path(record['source_hashes_file']).read_bytes()))
native = read(Path(records['baseline']['main_source_record']))
old_status = read(PREVIOUS / 'status.json')
baseline_status = [row for row in old_status if row['name'].startswith('baseline-')]
assert len(baseline_status) == 4 and all(row['exit_code'] == 0 for row in baseline_status)
assert read(PREVIOUS / 'validation-complete.json')['frontier_rows_identical'] == 19140
assert read(OUT / 'baseline-sources.json') == read(PREVIOUS / 'baseline-sources.json')
baseline_library = read(PREVIOUS / 'libraries.json')['baseline']
assert digest(Path(baseline_library['path'])) == baseline_library['sha256']
baseline_frontiers = read(PREVIOUS / 'frontiers.json')['baseline']
assert digest(PREVIOUS / 'baseline-frontiers.jsonl.gz') == baseline_frontiers['compressed_sha256']
for name in ['baseline-frontiers.jsonl.gz', 'baseline-normal-form-tests.log.gz', 'baseline-library.log']:
    shutil.copyfile(PREVIOUS / name, OUT / name)
(OUT / 'baseline-status.json').write_text(json.dumps(baseline_status, indent=2) + '\n')
(OUT / 'baseline-reuse.json').write_text(json.dumps({
    'record': str(EVIDENCE / 'diagnostics-68.json'),
    'source_record_sha256': digest(PREVIOUS / 'baseline-sources.json'),
    'scope': 'Reuse only the unchanged production baseline scalar validation, receipts and library. Candidate validation and subsequent matched process measurements are fresh.',
}, indent=2) + '\n')
shutil.copyfile(__file__, OUT / 'validate.py')
status = []


def verify():
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
    assert all(digest(BASE / name) == value for name, value in hashes['baseline'].items())
    assert all(digest(SOURCE / name) == value for name, value in hashes['candidate'].items())


def run(name, command):
    verify()
    label = 'candidate-' + name
    print('start', label, flush=True)
    start = time.monotonic()
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(TARGET)}
    with (OUT / (label + '.log')).open('w') as log:
        result = subprocess.run(command, cwd=SOURCE, env=env, stdout=log, stderr=subprocess.STDOUT)
    status.append({'name': label, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    print('finish', label, result.returncode, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)


run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--features', 'serde', '--all-targets', '--', '-D', 'warnings'])
run('normal-form-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--features', 'serde', '--lib', 'normal_form', '--', '--test-threads=1', '--nocapture'])
log = OUT / 'candidate-normal-form-tests.log'
rows = []
summary = None
for line in log.read_text().splitlines():
    if 'LAURENT_FRONTIER ' in line:
        rows.append(json.loads(line.split('LAURENT_FRONTIER ', 1)[1]))
    elif 'LAURENT_FRONTIER_SUMMARY ' in line:
        summary = json.loads(line.split('LAURENT_FRONTIER_SUMMARY ', 1)[1])
assert summary == {'fixtures': 58, 'rows': 19140} and len(rows) == 19140
canonical = ''.join(json.dumps(row, sort_keys=True, separators=(',', ':')) + '\n' for row in rows).encode()
compressed = gzip.compress(canonical, mtime=0)
assert gzip.decompress(compressed) == canonical
(OUT / 'candidate-frontiers.jsonl.gz').write_bytes(compressed)
raw = log.read_bytes()
compressed_log = gzip.compress(raw, mtime=0)
assert gzip.decompress(compressed_log) == raw
(OUT / (log.name + '.gz')).write_bytes(compressed_log)
log.unlink()
frontiers = {'baseline': baseline_frontiers, 'candidate': {
    'rows': len(rows), 'fixtures': 58, 'uncompressed_bytes': len(canonical),
    'uncompressed_sha256': hashlib.sha256(canonical).hexdigest(),
    'compressed_sha256': hashlib.sha256(compressed).hexdigest(),
    'raw_log': {'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest(), 'gzip_sha256': hashlib.sha256(compressed_log).hexdigest()},
}}
(OUT / 'frontiers.json').write_text(json.dumps(frontiers, indent=2) + '\n')
baseline_rows = gzip.decompress((OUT / 'baseline-frontiers.jsonl.gz').read_bytes()).splitlines()
changes = [(index, a, b) for index, (a, b) in enumerate(zip(baseline_rows, canonical.splitlines(), strict=True)) if a != b]
(OUT / 'frontier-comparison.json').write_text(json.dumps({
    'rows': 19140, 'different_rows': len(changes),
    'baseline_validation_reused': str(EVIDENCE / 'diagnostics-68.json'),
    'first_differences': [{'index': index, 'baseline': json.loads(a), 'candidate': json.loads(b)} for index, a, b in changes[:20]],
    'compared': 'Exact coefficient/atom receipts, original ordered root/recursive memo entries and remaining work. The diagnostic checks strong counts and approximation caches after each query.',
}, indent=2) + '\n')
assert not changes, len(changes)
run('library', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '--features', 'serde', '--lib', '--message-format=json'])
events = [json.loads(line) for line in (OUT / 'candidate-library.log').read_text().splitlines() if line.startswith('{')]
library = next(Path(filename) for event in events if event.get('reason') == 'compiler-artifact' and event['target']['name'] == 'hyperreal' for filename in event['filenames'] if filename.endswith('.rlib'))
(OUT / 'libraries.json').write_text(json.dumps({'baseline': baseline_library, 'candidate': {'path': str(library), 'bytes': library.stat().st_size, 'sha256': digest(library)}}, indent=2) + '\n')
verify()
(OUT / 'validation-complete.json').write_text(json.dumps({'candidate_commands_passed': 4, 'baseline_commands_reused': 4, 'frontier_rows_identical': 19140, 'all_sources_unchanged': True}, indent=2) + '\n')
print('Shared-expansion candidate passes focused checks and preserves all 19,140 frontier receipts.', flush=True)
