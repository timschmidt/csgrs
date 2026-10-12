"""Freeze and test the proposed production/proof dependency identity checker."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
PREVIEW = E / 'dependency-binding-candidate-v2424'
OUT = E / 'dependency-binding-units-v2425'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists()
    checkpoint_path = E / 'typed-api-prototype-v2386-state.json'
    assert sha(checkpoint_path) == '6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9'
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint['source_record'])
    guard = snapshot(E / 'candidate-preparation/main-hashes.json.gz')
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    inputs = {str(p.relative_to(PREVIEW)): sha(p) for p in sorted(PREVIEW.rglob('*')) if p.is_file()}
    assert set(inputs) == {'scripts/verus_dependencies.py', 'verification/test_dependency_identity.py'}
    command = [sys.executable, str(PREVIEW / 'verification/test_dependency_identity.py'), '-v']
    OUT.mkdir()
    protocol = {
        'status': 'Frozen before the nineteen dependency identity unit tests',
        'runner_sha256': sha(Path(__file__)), 'preview': PREVIEW.name, 'preview_files_sha256': inputs,
        'retained_checkpoint_sha256': sha(checkpoint_path), 'source_record_sha256': checkpoint['source_record_sha256'],
        'original_helper_sha256': sha(SOURCE / 'crates/hyperreal/scripts/verus_dependencies.py'),
        'original_test_sha256': sha(SOURCE / 'crates/hyperreal/verification/test_dependency_identity.py'),
        'command': command, 'expected_tests': 19,
        'scope': 'Exercise graph reachability, actual local paths, registry identity, arbitrary dependency input files, configuration additions, re-resolution drift and artifact changes. These unit fixtures mock Cargo metadata and do not substitute for an actual Cargo build or SOURCE integration. No retained proof/runtime coverage change.',
    }
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    started = time.monotonic()
    result = subprocess.run(command, cwd=ROOT, env=os.environ | {'PYTHONDONTWRITEBYTECODE': '1'}, capture_output=True, text=True)
    (OUT / 'stdout.log').write_text(result.stdout)
    (OUT / 'stderr.log').write_text(result.stderr)
    assert {str(p.relative_to(PREVIEW)): sha(p) for p in sorted(PREVIEW.rglob('*')) if p.is_file()} == inputs
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    data = {
        'exit_code': result.returncode, 'seconds': time.monotonic() - started,
        'protocol_sha256': sha(OUT / 'protocol.json'), 'stdout_sha256': sha(OUT / 'stdout.log'),
        'stderr_sha256': sha(OUT / 'stderr.log'), 'source_unchanged': True, 'main_unchanged': True,
        'preview_unchanged': True, 'retained_coverage_changed': False,
        'nineteen_tests_passed': result.returncode == 0 and 'Ran 19 tests in ' in result.stderr and result.stderr.endswith('\nOK\n'),
    }
    (OUT / 'result.json').write_text(json.dumps(data, indent=2) + '\n')
    print(json.dumps(data))
    assert data['nineteen_tests_passed']


if __name__ == '__main__':
    main()
