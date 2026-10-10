"""Validate the unused root memo omission against the unchanged, previously executed baseline."""
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
OUT = EVIDENCE / 'laurent-root-memo-omission'
PREVIOUS = EVIDENCE / 'laurent-memo-sharing'
WORK = ROOT / 'target/laurent-root-memo-omission'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
native = read(EVIDENCE / 'native-exactness-gate/sources.json')
sources = {'candidate': read(OUT / 'candidate-sources.json')}
baseline_sources = read(PREVIOUS / 'baseline-sources.json')
assert all(digest(ROOT / 'target/laurent-memo-sharing/baseline/source' / name) == value for name, value in baseline_sources['source_hashes'].items())
shutil.copyfile(PREVIOUS / 'baseline-frontiers.jsonl.gz', OUT / 'baseline-frontiers.jsonl.gz')
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


libraries = {'baseline': read(PREVIOUS / 'libraries.json')['baseline']}
assert digest(Path(libraries['baseline']['path'])) == libraries['baseline']['sha256']
frontiers = {'baseline': read(PREVIOUS / 'frontiers.json')['baseline']}
for variant in ['candidate']:
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
unexpected = 0
root_entries_omitted = 0
raw_differences = 0
with gzip.open(OUT / 'baseline-frontiers.jsonl.gz', 'rt') as left, gzip.open(OUT / 'candidate-frontiers.jsonl.gz', 'rt') as right:
    for index, (a, b) in enumerate(zip(left, right, strict=True)):
        original = json.loads(a)
        candidate = json.loads(b)
        raw_differences += original != candidate
        expected = original.copy()
        entries = original['memo']
        if entries and entries[-1][0] == 0:
            assert all(entry[0] != 0 for entry in entries[:-1])
            expected['memo'] = entries[:-1] or None
            root_entries_omitted += 1
        else:
            assert not entries or all(entry[0] != 0 for entry in entries)
        if candidate != expected:
            unexpected += 1
            if len(differences) < 20:
                differences.append({'index': index, 'baseline': original, 'expected': expected, 'candidate': candidate})
comparison = {
    'rows': 19140,
    'unexpected_different_rows': unexpected,
    'raw_different_rows': raw_differences,
    'final_root_entries_omitted': root_entries_omitted,
    'first_unexpected_differences': differences,
    'compared': 'Every result coefficient/atom, remaining work value and recursive memo entry/order must match. The sole permitted difference is omission of the final entry with root node identity 0; an otherwise-empty memo then remains absent. Raw receipts are retained without normalization. Each query also checks unchanged graph strong counts and approximation caches after dropping its result and memo.',
    'main_production_unchanged': True,
}
(OUT / 'frontier-comparison.json').write_text(json.dumps(comparison, indent=2) + '\n')
assert unexpected == 0 and raw_differences == root_entries_omitted, comparison
verify()
print('Root memo omission passes focused validation:', root_entries_omitted, 'unused entries omitted, all 19,140 outputs/work values and recursive memo entries preserved.', flush=True)
