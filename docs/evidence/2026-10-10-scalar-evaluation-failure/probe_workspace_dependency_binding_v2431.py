"""Check actual workspace/proof dependency resolution without rebuilding or editing SOURCE."""
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import tomllib

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
CRATE = SOURCE / 'crates/hyperreal'
PREVIEW = E / 'dependency-binding-candidate-v2424'
OUT = E / 'workspace-dependency-binding-v2431'
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
    units = E / 'dependency-binding-units-v2425'
    inputs = read(units / 'protocol.json')['preview_files_sha256']
    assert all(sha(PREVIEW / n) == h for n, h in inputs.items())
    assert read(E / 'dependency-binding-cargo-v2427/result.json')['complete']
    spec = importlib.util.spec_from_file_location('candidate_dependencies', PREVIEW / 'scripts/verus_dependencies.py')
    dependencies = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dependencies)
    pin = read(CRATE / 'verification/toolchain.json')
    proof_manifest = CRATE / 'verification/dependencies/Cargo.toml'
    production_manifest = CRATE / 'Cargo.toml'
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true',
        'CARGO_TARGET_DIR': str(ROOT / 'target/dependency-binding-workspace-probe-v2431'),
        'CARGO_TERM_COLOR': 'never', 'PYTHONDONTWRITEBYTECODE': '1'}
    configuration = dependencies.configuration_sources(CRATE, SOURCE / 'Cargo.lock', env)
    commands = [['cargo', '+' + pin['rust_toolchain'], 'metadata', '--locked', '--format-version=1', '--manifest-path', str(path)]
                for path in [proof_manifest, production_manifest]]
    OUT.mkdir()
    protocol = {
        'status': 'Frozen before actual workspace and proof metadata commands',
        'runner_sha256': sha(Path(__file__)), 'helper_sha256': sha(PREVIEW / 'scripts/verus_dependencies.py'),
        'retained_checkpoint_sha256': sha(checkpoint_path), 'source_record_sha256': checkpoint['source_record_sha256'],
        'commands': commands, 'cwd': str(CRATE), 'configuration_sources': configuration,
        'environment': {k: env[k] for k in ['CARGO_HOME', 'CARGO_NET_OFFLINE', 'CARGO_TARGET_DIR', 'CARGO_TERM_COLOR', 'PYTHONDONTWRITEBYTECODE']},
        'scope': 'Use real metadata for the retained workspace and standalone proof bundle with the proposed graph/path binding function. Read-only resolution and source hashing; do not rebuild retained dependency libraries or modify SOURCE. This is infrastructure qualification and does not increase proof coverage.',
    }
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results = []
    metadata = []
    for name, command in zip(['proof', 'production'], commands):
        started = time.monotonic()
        result = subprocess.run(command, cwd=CRATE, env=env, capture_output=True, text=True)
        (OUT / f'{name}.metadata.json').write_text(result.stdout)
        (OUT / f'{name}.stderr.log').write_text(result.stderr)
        results.append({'name': name, 'exit_code': result.returncode, 'seconds': time.monotonic() - started,
            'stdout_sha256': sha(OUT / f'{name}.metadata.json'), 'stderr_sha256': sha(OUT / f'{name}.stderr.log')})
        (OUT / 'resolution-result.json').write_text(json.dumps({'protocol_sha256': sha(OUT / 'protocol.json'),
            'runs': results, 'complete': len(results) == 2 and result.returncode == 0}, indent=2) + '\n')
        assert result.returncode == 0, result.stderr
        metadata.append(json.loads(result.stdout))
    lock = tomllib.loads(proof_manifest.with_name('Cargo.lock').read_text())
    packages, bindings = dependencies.bind_resolution(metadata[0], metadata[1], proof_manifest, production_manifest, lock)
    dependency_sources = dependencies.dependency_sources(packages)
    assert packages and all(p['source'] is not None for p in packages)
    assert dependencies.configuration_sources(CRATE, SOURCE / 'Cargo.lock', env) == configuration
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    assert all(sha(PREVIEW / n) == h for n, h in inputs.items())
    data = {
        'status': 'Proposed dependency binding matches the actual retained workspace and proof resolution',
        'protocol_sha256': sha(OUT / 'protocol.json'), 'packages': packages, 'bindings': bindings,
        'dependency_sources': dependency_sources, 'production_metadata_packages': len(metadata[1]['packages']),
        'production_reachable_packages': len(dependencies.reachable_packages(metadata[1], dependencies.root_package(metadata[1], production_manifest)['id'])),
        'source_unchanged': True, 'main_unchanged': True, 'preview_unchanged': True,
        'configuration_unchanged': True, 'retained_coverage_changed': False, 'dependency_builds': 0,
    }
    (OUT / 'result.json').write_text(json.dumps(data, indent=2) + '\n')
    print(json.dumps({'bound_proof_packages': len(packages), 'production_metadata_packages': data['production_metadata_packages'],
        'production_reachable_packages': data['production_reachable_packages'], 'dependency_builds': 0, 'source_changed': False}))


if __name__ == '__main__':
    main()
