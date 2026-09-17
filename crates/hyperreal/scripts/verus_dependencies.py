#!/usr/bin/env python3
"""Build pinned external datatypes for Verus without assuming their operations."""
import json
import os
from pathlib import Path
import subprocess
import tomllib
import hashlib


def sha256(path):
    with path.open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest()


def dependency_sources(packages):
    return {
        package['id']: {
            str(path.relative_to(Path(package['manifest_path']).parent)): sha256(path)
            for path in sorted(Path(package['manifest_path']).parent.rglob('*'))
            if path.is_file() and not {'target', '.git'}.intersection(
                path.relative_to(Path(package['manifest_path']).parent).parts)
        }
        for package in packages
    }


def prepare_dependencies(root, pin, reports):
    manifest = root / 'verification/dependencies/Cargo.toml'
    lock = tomllib.loads(manifest.with_name('Cargo.lock').read_text())
    production = tomllib.loads((root / 'Cargo.lock').read_text())
    identity = lambda package: tuple(package.get(k) for k in ('name', 'version', 'source', 'checksum'))
    production_pins = {identity(package) for package in production['package']}
    for package in lock['package']:
        if package.get('source') and identity(package) not in production_pins:
            raise RuntimeError(f"Proof dependency does not match production lock: {package['name']}")

    target = root / 'target/verification-dependencies'
    environment = os.environ | {'CARGO_TARGET_DIR': str(target), 'CARGO_INCREMENTAL': '0',
                                'CARGO_PROFILE_DEV_DEBUG': '0'}
    cargo = ['cargo', '+' + pin['rust_toolchain']]
    metadata_command = cargo + ['metadata', '--locked', '--format-version=1', '--manifest-path', str(manifest)]
    metadata = json.loads(subprocess.check_output(metadata_command, cwd=root, env=environment, text=True))
    packages = [p for p in metadata['packages'] if p['source'] is not None]
    for package in packages:
        if not any(p['name'] == package['name'] and p['version'] == package['version']
                   and p.get('source') == package['source'] for p in lock['package']):
            raise RuntimeError(f"Unexpected resolved proof dependency: {package['id']}")
    sources = dependency_sources(packages)
    build_command = cargo + ['build', '--locked', '--manifest-path', str(manifest), '--message-format=json']
    result = subprocess.run(build_command, cwd=root, env=environment, capture_output=True, text=True)
    reports.mkdir(parents=True, exist_ok=True)
    (reports / 'dependencies-build.stdout').write_text(result.stdout)
    (reports / 'dependencies-build.stderr').write_text(result.stderr)
    if result.returncode:
        raise RuntimeError(f"Proof dependency build failed (exit {result.returncode}); see {reports}")
    artifacts = [item for item in map(json.loads, result.stdout.splitlines())
                 if item.get('reason') == 'compiler-artifact']
    num = [item for item in artifacts if item['target']['name'] == 'num' and item['target']['kind'] == ['lib']]
    if len(num) != 1:
        raise RuntimeError('Expected exactly one pinned num library')
    libraries = [name for name in num[0]['filenames'] if name.endswith('.rlib')]
    if len(libraries) != 1:
        raise RuntimeError('Expected exactly one pinned num rlib')
    binaries = {name: sha256(Path(name)) for item in artifacts for name in item['filenames']}
    if sources != dependency_sources(packages):
        raise RuntimeError('Proof dependency sources changed during their build')
    arguments = ['--extern', 'num=' + libraries[0], '-L', 'dependency=' + str(target / 'debug/deps')]
    evidence = {
        'metadata_command': metadata_command, 'build_command': build_command,
        'build_exit_code': result.returncode, 'packages': packages,
        'sources': sources, 'artifacts': binaries, 'verus_arguments': arguments,
        'scope': 'Transparent Sign datatype only. No external arithmetic or equality implementation is assumed or verified by this import.',
    }
    (reports / 'dependencies.json').write_text(json.dumps(evidence, indent=2) + '\n')
    return arguments, evidence


def check_dependency_identity(evidence):
    if evidence['sources'] != dependency_sources(evidence['packages']):
        raise RuntimeError('Proof dependency sources changed during verification')
    if any(sha256(Path(name)) != expected for name, expected in evidence['artifacts'].items()):
        raise RuntimeError('Proof dependency artifacts changed during verification')
