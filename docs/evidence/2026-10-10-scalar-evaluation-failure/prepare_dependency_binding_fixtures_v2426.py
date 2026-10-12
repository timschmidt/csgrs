"""Freeze real Cargo fixtures for matched and split local BigUint dependencies."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import tomllib

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
WORK = ROOT / 'target/dependency-binding-fixtures-v2426'
OUT = E / 'dependency-binding-fixture-preparation-v2426'
PREVIEW = E / 'dependency-binding-candidate-v2424'
VENDOR = E / 'biguint-metadata-candidate-v2413/vendor/num-bigint'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    result = {}
    for directory, children, names in os.walk(root):
        children[:] = sorted(n for n in children if n not in {'target', '.git', '__pycache__'})
        for name in sorted(names):
            path = Path(directory) / name
            assert not path.is_symlink(), path
            result[str(path.relative_to(root))] = sha(path)
    return result


def main():
    assert not WORK.exists() and not OUT.exists()
    checkpoint_path = E / 'typed-api-prototype-v2386-state.json'
    assert sha(checkpoint_path) == '6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9'
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint['source_record'])
    guard = snapshot(E / 'candidate-preparation/main-hashes.json.gz')
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    units = E / 'dependency-binding-units-v2425'
    assert read(units / 'result.json')['nineteen_tests_passed']
    preview_files = read(units / 'protocol.json')['preview_files_sha256']
    assert files(PREVIEW) == preview_files
    vendor_protocol = read(E / 'biguint-metadata-proof-v2414/protocol.json')
    assert all(sha(VENDOR.parent.parent / n) == h for n, h in vendor_protocol['preview_files_sha256'].items())
    OUT.mkdir()
    production_manifest = '''[package]
name = "production-dependency-fixture"
version = "0.0.0"
edition = "2024"
publish = false

[workspace]
exclude = ["verification/dependencies", "vendor/num-bigint", "vendor-proof/num-bigint"]

[dependencies]
num = { version = "=0.4.3", features = ["num-bigint"] }

[patch.crates-io]
num-bigint = { path = "vendor/num-bigint" }
'''
    fixtures = []
    for name, proof_vendor in [('matched', 'vendor'), ('split', 'vendor-proof')]:
        root = WORK / name
        shutil.copytree(VENDOR, root / 'vendor/num-bigint')
        if name == 'split':
            shutil.copytree(VENDOR, root / 'vendor-proof/num-bigint')
        (root / 'src').mkdir()
        (root / 'src/lib.rs').write_text('pub use num::bigint::{BigInt, BigUint, Sign};\n')
        (root / 'Cargo.toml').write_text(production_manifest)
        proof = root / 'verification/dependencies'
        (proof / 'src').mkdir(parents=True)
        (proof / 'src/lib.rs').write_bytes((SOURCE / 'crates/hyperreal/verification/dependencies/src/lib.rs').read_bytes())
        proof_manifest = (SOURCE / 'crates/hyperreal/verification/dependencies/Cargo.toml').read_text()
        proof_manifest += f'\n[patch.crates-io]\nnum-bigint = {{ path = "../../{proof_vendor}/num-bigint" }}\n'
        (proof / 'Cargo.toml').write_text(proof_manifest)
        for lock in [root / 'Cargo.lock', proof / 'Cargo.lock']:
            lock.write_bytes((SOURCE / 'Cargo.lock').read_bytes())
        fixtures.append({'name': name, 'root': str(root), 'expected_acceptance': name == 'matched'})
    pin = read(SOURCE / 'crates/hyperreal/verification/toolchain.json')
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true',
                        'CARGO_TARGET_DIR': str(WORK / 'target'), 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
                        'CARGO_TERM_COLOR': 'never', 'PYTHONDONTWRITEBYTECODE': '1'}
    rows = []
    for fixture in fixtures:
        root = Path(fixture['root'])
        for kind, manifest in [('production', root / 'Cargo.toml'), ('proof', root / 'verification/dependencies/Cargo.toml')]:
            rows.append({'name': fixture['name'] + '-' + kind, 'root': str(root),
                'command': ['cargo', '+' + pin['rust_toolchain'], 'metadata', '--offline', '--format-version=1', '--manifest-path', str(manifest)]})
    before = files(WORK)
    protocol = {
        'status': 'Frozen before resolving the real Cargo identity fixtures',
        'runner_sha256': sha(Path(__file__)), 'retained_checkpoint_sha256': sha(checkpoint_path),
        'source_record_sha256': checkpoint['source_record_sha256'], 'preview': PREVIEW.name,
        'preview_files_sha256': preview_files, 'vendor_source_files_sha256': files(VENDOR),
        'work': str(WORK), 'work_before_resolution_sha256': before, 'fixtures': fixtures, 'commands': rows,
        'environment': {k: env[k] for k in ['CARGO_HOME', 'CARGO_NET_OFFLINE', 'CARGO_TARGET_DIR', 'CCACHE_DIR', 'CARGO_TERM_COLOR', 'PYTHONDONTWRITEBYTECODE']},
        'scope': 'Both fixtures use the actual num facade and the verified BigUint preview. The split fixture has byte-identical copies at different canonical paths and unchanged package names/versions; it must be rejected as a production/proof mismatch. This setup only resolves dependencies and has not yet run the proposed checker or built proof dependencies. No SOURCE or retained coverage mutation.',
    }
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results = []
    for row in rows:
        started = time.monotonic()
        result = subprocess.run(row['command'], cwd=row['root'], env=env, capture_output=True, text=True)
        (OUT / (row['name'] + '.metadata.json')).write_text(result.stdout)
        (OUT / (row['name'] + '.stderr.log')).write_text(result.stderr)
        results.append(row | {'exit_code': result.returncode, 'seconds': time.monotonic() - started,
            'stdout_sha256': sha(OUT / (row['name'] + '.metadata.json')), 'stderr_sha256': sha(OUT / (row['name'] + '.stderr.log'))})
        (OUT / 'resolution-result.json').write_text(json.dumps({'protocol_sha256': sha(OUT / 'protocol.json'),
            'runs': results, 'complete': len(results) == len(rows) and result.returncode == 0}, indent=2) + '\n')
        assert result.returncode == 0, result.stderr
        print(json.dumps({'resolved': row['name']}), flush=True)
    after = files(WORK)
    assert set(before) == set(after)
    changed = [n for n in before if before[n] != after[n]]
    assert set(changed) == {f'{name}/{suffix}' for name in ['matched', 'split'] for suffix in ['Cargo.lock', 'verification/dependencies/Cargo.lock']}
    identity = lambda p: tuple(p.get(k) for k in ['name', 'version', 'source', 'checksum'])
    production_pins = {identity(p) for p in tomllib.loads((SOURCE / 'Cargo.lock').read_text())['package']}
    for name in after:
        if '/vendor/' not in name and '/vendor-proof/' not in name:
            destination = OUT / 'inputs' / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes((WORK / name).read_bytes())
        if name.endswith('Cargo.lock'):
            for package in tomllib.loads((WORK / name).read_text())['package']:
                if package.get('source'):
                    assert identity(package) in production_pins, package
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    assert files(PREVIEW) == preview_files
    data = {'status': 'Real Cargo identity fixtures frozen before checker execution',
        'protocol_sha256': sha(OUT / 'protocol.json'), 'work': str(WORK), 'work_files_sha256': after,
        'fixtures': fixtures, 'source_unchanged': True, 'main_unchanged': True, 'preview_unchanged': True,
        'retained_coverage_changed': False, 'runtime_admitted': False}
    (OUT / 'result.json').write_text(json.dumps(data, indent=2) + '\n')
    print(json.dumps({'fixtures': len(fixtures), 'frozen_work_files': len(after), 'source_changed': False}))


if __name__ == '__main__':
    main()
