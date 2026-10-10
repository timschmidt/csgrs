"""Validate both isolated variants and compare every recorded recognition frontier."""
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
OUT = EVIDENCE / 'laurent-memo-index'
WORK = ROOT / 'target/laurent-memo-index'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
native = read(EVIDENCE / 'native-exactness-gate/sources.json')
sources = {variant: read(OUT / (variant + '-sources.json')) for variant in ['baseline', 'candidate']}
for record in sources.values():
    compressed = Path(record['source_hashes_file'])
    assert digest(compressed) == record['source_hashes_file_sha256']
    record['source_hashes'] = json.loads(gzip.decompress(compressed.read_bytes()))
    assert len(record['source_hashes']) == record['source_files']
shutil.copyfile(__file__, OUT / 'validate.py')
status = []


def verify():
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
    for variant, record in sources.items():
        assert all(digest(WORK / variant / 'source' / name) == value for name, value in record['source_hashes'].items())


def run(variant, name, command):
    verify()
    label = variant + '-' + name
    print('start', label, flush=True)
    start = time.monotonic()
    environment = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(WORK / variant / 'target')}
    with (OUT / (label + '.log')).open('w') as log:
        result = subprocess.run(command, cwd=WORK / variant / 'source', env=environment, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({'name': label, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    print('finish', label, result.returncode, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)


libraries = {}
frontiers = {}
for variant in ['baseline', 'candidate']:
    run(variant, 'format', ['cargo', 'fmt', '--all', '--', '--check'])
    run(variant, 'clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--features', 'serde', '--all-targets', '--', '-D', 'warnings'])
    run(variant, 'normal-form-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--features', 'serde', '--lib', 'normal_form', '--', '--test-threads=1', '--nocapture'])
    log = OUT / (variant + '-normal-form-tests.log')
    rows = []
    summary = None
    with log.open() as stream:
        for line in stream:
            if 'LAURENT_FRONTIER ' in line:
                rows.append(json.loads(line.split('LAURENT_FRONTIER ', 1)[1]))
            elif 'LAURENT_FRONTIER_SUMMARY ' in line:
                summary = json.loads(line.split('LAURENT_FRONTIER_SUMMARY ', 1)[1])
    assert summary == {'fixtures': 58, 'rows': 19140}, summary
    assert len(rows) == 19140
    canonical = ''.join(json.dumps(row, sort_keys=True, separators=(',', ':')) + '\n' for row in rows).encode()
    compressed = gzip.compress(canonical, mtime=0)
    assert gzip.decompress(compressed) == canonical
    (OUT / (variant + '-frontiers.jsonl.gz')).write_bytes(compressed)
    frontiers[variant] = {'rows': len(rows), 'fixtures': 58, 'uncompressed_bytes': len(canonical), 'uncompressed_sha256': hashlib.sha256(canonical).hexdigest(), 'compressed_sha256': hashlib.sha256(compressed).hexdigest()}
    raw = log.read_bytes()
    compressed_raw = gzip.compress(raw, mtime=0)
    assert gzip.decompress(compressed_raw) == raw
    (OUT / (log.name + '.gz')).write_bytes(compressed_raw)
    frontiers[variant]['raw_log'] = {'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest(), 'gzip_sha256': hashlib.sha256(compressed_raw).hexdigest()}
    log.unlink()
    del raw, compressed_raw, canonical, compressed, rows
    run(variant, 'library', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '--features', 'serde', '--lib', '--message-format=json'])
    with (OUT / (variant + '-library.log')).open() as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except ValueError:
                continue
            if event.get('reason') == 'compiler-artifact' and event.get('target', {}).get('name') == 'hyperreal':
                path = Path(next(name for name in event['filenames'] if name.endswith('.rlib')))
                libraries[variant] = {'path': str(path), 'sha256': digest(path), 'bytes': path.stat().st_size}
assert len(libraries) == 2
(OUT / 'libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
(OUT / 'frontiers.json').write_text(json.dumps(frontiers, indent=2) + '\n')
differences = []
different = 0
with gzip.open(OUT / 'baseline-frontiers.jsonl.gz', 'rt') as left, gzip.open(OUT / 'candidate-frontiers.jsonl.gz', 'rt') as right:
    for index, (a, b) in enumerate(zip(left, right, strict=True)):
        if a != b:
            different += 1
            if len(differences) < 20:
                differences.append({'index': index, 'baseline': json.loads(a), 'candidate': json.loads(b)})
comparison = {'rows': 19140, 'different_rows': different, 'first_differences': differences, 'compared': 'Exact coefficient/atom receipts, insertion-ordered memo keys and values, remaining work across all depth/work pairs. Each run also checks that graph strong counts and approximation caches are unchanged by recognition.', 'main_production_unchanged': True}
(OUT / 'frontier-comparison.json').write_text(json.dumps(comparison, indent=2) + '\n')
assert different == 0, different
historical_dir = EVIDENCE / 'laurent-memo-sharing'
historical_record = read(historical_dir / 'frontiers.json')['baseline']
historical_path = historical_dir / 'baseline-frontiers.jsonl.gz'
assert digest(historical_path) == historical_record['compressed_sha256']
historical = gzip.decompress(historical_path.read_bytes()).splitlines()
current = gzip.decompress((OUT / 'baseline-frontiers.jsonl.gz').read_bytes()).splitlines()
assert len(historical) == len(current) == 19140
changes = [(index, a, b) for index, (a, b) in enumerate(zip(historical, current)) if a != b]
(OUT / 'historical-baseline-comparison.json').write_text(json.dumps({
    'rows': 19140, 'different_rows': len(changes),
    'scope': 'Fresh admitted-production baseline versus the retained native-prototype baseline; the diagnostic corpus is unchanged.',
    'first_differences': [{'index': index, 'historical': json.loads(a), 'current': json.loads(b)} for index, a, b in changes[:20]],
}, indent=2) + '\n')
assert not changes, len(changes)
verify()
(OUT / 'validation-complete.json').write_text(json.dumps({'all_eight_commands_passed': True, 'frontier_rows_identical': 19140, 'historical_baseline_rows_identical': 19140, 'main_and_candidate_sources_unchanged': True}, indent=2) + '\n')
print('Both variants pass focused validation; all 19,140 recognition frontier receipts match.', flush=True)
