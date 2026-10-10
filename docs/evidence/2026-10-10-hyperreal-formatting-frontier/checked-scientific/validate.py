"""Validate the frozen checked formatter before further resource or downstream work."""
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
OUT = EVIDENCE / 'checked-scientific'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
record = read(OUT / 'sources.json')
source = Path(record['source_directory'])
hash_file = Path(record['source_hashes_file'])
assert digest(hash_file) == record['source_hashes_file_sha256']
hashes = json.loads(gzip.decompress(hash_file.read_bytes()))
native = read(Path(record['main_source_record']))['source_hashes']
assert not (OUT / 'status.json').exists()
shutil.copyfile(__file__, OUT / 'validate.py')
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_TARGET_DIR': record['target_directory'], 'RUSTDOCFLAGS': '-D warnings'}
status = []


def verify():
    assert all(digest(source / name) == value for name, value in hashes.items())
    assert all(digest(ROOT / name) == value for name, value in native.items())


def run(name, command, tests=False):
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
    if tests:
        counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', path.read_text())
        assert counts and sum(int(item[0]) for item in counts) > 0
        row['tests_passed'] = sum(int(item[0]) for item in counts)
        row['tests_ignored'] = sum(int(item[2]) for item in counts)
        assert all(int(item[1]) == 0 for item in counts)
        (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')


run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('new-and-api-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--test', 'checked_scientific', '--test', 'gmp_api_coverage', '--', '--test-threads=2'], tests=True)
run('existing-format-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', 'computable::format::tests::', '--', '--test-threads=2'], tests=True)
run('all-feature-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--all-targets', '--', '-D', 'warnings'])
run('all-feature-rustdoc', ['cargo', 'doc', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--no-deps'])
run('checked-doc-test', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--doc', 'try_scientific', '--', '--test-threads=2'], tests=True)
verify()
(OUT / 'source-verification.json').write_text(json.dumps({'all_candidate_and_main_source_hashes_unchanged': True, 'candidate_files': len(hashes), 'main_files': len(native), 'scope': 'Focused checked formatter and existing formatting tests, API inventory, strict all-feature Clippy/rustdoc and method doctest. No full scalar or downstream suite, resource admission or original-formatting-trait completion is claimed.'}, indent=2) + '\n')
print('Focused checked scientific validation complete.', flush=True)
