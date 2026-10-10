"""Run complete scalar suites on the frozen checked-formatting candidate."""
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
OUT = EVIDENCE / 'checked-scalar-validation'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert read(EVIDENCE / 'checked-frontier.json')['preparation_or_concurrency_receipt_differences_from_fresh'] == 0
record = read(EVIDENCE / 'checked-scientific/sources.json')
source = Path(record['source_directory'])
hash_file = Path(record['source_hashes_file'])
assert digest(hash_file) == record['source_hashes_file_sha256']
hashes = json.loads(gzip.decompress(hash_file.read_bytes()))
native = read(Path(record['main_source_record']))['source_hashes']
OUT.mkdir()
shutil.copyfile(__file__, OUT / 'validate.py')
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_TARGET_DIR': record['target_directory'], 'RUSTDOCFLAGS': '-D warnings'}
status = []


def verify():
    assert all(digest(source / name) == value for name, value in hashes.items())
    assert all(digest(ROOT / name) == value for name, value in native.items())


for name, command in [
    ('default-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--lib', '--tests', '--', '--test-threads=2']),
    ('all-feature-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--tests', '--', '--test-threads=2']),
    ('all-feature-doc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--doc', '--', '--test-threads=2']),
]:
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    path = OUT / (name + '.log')
    with path.open('w') as log:
        result = subprocess.run(command, cwd=source, env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
    row = {'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start}
    status.append(row)
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)
    counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', path.read_text())
    assert counts and sum(int(item[0]) for item in counts) > 0
    row['test_binaries'] = len(counts)
    row['tests_passed'] = sum(int(item[0]) for item in counts)
    row['tests_ignored'] = sum(int(item[2]) for item in counts)
    assert all(int(item[1]) == 0 for item in counts)
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
verify()
record = {'decision': 'complete default/all-feature scalar suites and all-feature doctests pass; resource and downstream gates remain before runtime admission', 'source_record': str(EVIDENCE / 'checked-scientific/sources.json'), 'source_record_sha256': digest(EVIDENCE / 'checked-scientific/sources.json'), 'steps': status, 'all_candidate_and_main_source_hashes_unchanged': True, 'candidate_files': len(hashes), 'main_files': len(native), 'limitations': 'Strict Clippy/rustdoc/formatting are reused from the focused record on identical frozen source. No full workspace/pre-push, downstream suite, performance tradeoff or legacy-formatting-trait completion is claimed.'}
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted(OUT.iterdir()) if path.is_file()}
record['script_sha256'] = digest(Path(__file__))
(EVIDENCE / 'checked-scalar-validation.json').write_text(json.dumps(record, indent=2) + '\n')
print('Complete scalar validation passed:', [(row['name'], row['tests_passed'], row['tests_ignored']) for row in status], flush=True)
