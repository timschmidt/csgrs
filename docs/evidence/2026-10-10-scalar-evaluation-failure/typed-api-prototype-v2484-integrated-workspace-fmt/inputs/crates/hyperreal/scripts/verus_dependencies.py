#!/usr/bin/env python3
"""Bind Verus dependency inputs to the actual production Cargo resolution."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tomllib

from verus_bigint import prepare_export, bind_export, check_export


def sha256(path):
    with path.open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest()


def dependency_sources(packages):
    result = {}
    for package in packages:
        root = Path(package['manifest_path']).parent
        sources = {}
        for directory, children, names in os.walk(root):
            children[:] = sorted(name for name in children if name not in {'target', '.git'})
            for name in [*children, *names]:
                path = Path(directory) / name
                if path.is_symlink():
                    raise RuntimeError(f'Symbolic-link dependency input is not supported: {path}')
            for name in sorted(names):
                path = Path(directory) / name
                if path.is_file():
                    sources[str(path.relative_to(root))] = sha256(path)
        result[package['id']] = sources
    return result


def lock_identity(package):
    return tuple(package.get(key) for key in ('name', 'version', 'source', 'checksum'))


def resolved_identity(package):
    # Cargo.lock intentionally lacks local paths. Metadata supplies the actual
    # canonical manifest selected by patches, path dependencies and overrides.
    return (package['name'], package['version'], package.get('source'),
            str(Path(package['manifest_path']).resolve()))


def root_package(metadata, manifest):
    expected = Path(manifest).resolve()
    matches = [p for p in metadata['packages'] if Path(p['manifest_path']).resolve() == expected]
    if len(matches) != 1:
        raise RuntimeError(f'Expected exactly one Cargo root package: {manifest}')
    return matches[0]


def reachable_packages(metadata, root_id):
    nodes = {node['id']: node for node in metadata['resolve']['nodes']}
    packages = {package['id']: package for package in metadata['packages']}
    reached = set()
    pending = [root_id]
    while pending:
        current = pending.pop()
        if current in reached:
            continue
        if current not in nodes or current not in packages:
            raise RuntimeError(f'Incomplete Cargo dependency graph: {current}')
        reached.add(current)
        pending.extend(edge['pkg'] for edge in nodes[current]['deps'])
    return [packages[key] for key in sorted(reached)]


def bind_resolution(proof_metadata, production_metadata, proof_manifest, production_manifest, lock):
    proof_root = root_package(proof_metadata, proof_manifest)
    production_root = root_package(production_metadata, production_manifest)
    proof_packages = [p for p in reachable_packages(proof_metadata, proof_root['id'])
                      if p['id'] != proof_root['id']]
    production_packages = reachable_packages(production_metadata, production_root['id'])
    production = {resolved_identity(p) for p in production_packages}
    bindings = []
    for package in proof_packages:
        if not any(p['name'] == package['name'] and p['version'] == package['version']
                   and p.get('source') == package.get('source') for p in lock['package']):
            raise RuntimeError(f"Unexpected resolved proof dependency: {package['id']}")
        identity = resolved_identity(package)
        if identity not in production:
            raise RuntimeError(f"Proof dependency does not match production resolution: {package['id']}")
        bindings.append({'proof_package_id': package['id'], 'name': identity[0],
                         'version': identity[1], 'source': identity[2], 'manifest_path': identity[3]})
    return proof_packages, bindings


def configuration_sources(root, production_lock, environment):
    paths = {root / 'Cargo.toml', production_lock, production_lock.with_name('Cargo.toml'),
             root / 'verification/dependencies/Cargo.toml', root / 'verification/dependencies/Cargo.lock'}
    for directory in [root, *root.parents]:
        paths.update(directory / '.cargo' / name for name in ['config', 'config.toml'])
    cargo_home = Path(environment.get('CARGO_HOME', Path.home() / '.cargo'))
    paths.update(cargo_home / name for name in ['config', 'config.toml'])
    # Record missing files too: introducing a previously absent Cargo override
    # must invalidate the configuration binding just as editing one does.
    return {str(path): sha256(path) if path.is_file() else None for path in sorted(paths)}


def read_resolution(commands, root, environment, proof_manifest, production_manifest, lock):
    proof, production = [json.loads(subprocess.check_output(command, cwd=root, env=environment, text=True))
                         for command in commands]
    packages, bindings = bind_resolution(proof, production, proof_manifest, production_manifest, lock)
    return proof, packages, bindings


def prepare_dependencies(root, pin, reports, verus_path=None):
    root = Path(root).resolve()
    manifest = root / 'verification/dependencies/Cargo.toml'
    lock = tomllib.loads(manifest.with_name('Cargo.lock').read_text())
    production_lock = next(directory / 'Cargo.lock' for directory in [root, *root.parents]
                           if (directory / 'Cargo.lock').is_file())
    production = tomllib.loads(production_lock.read_text())
    production_pins = {lock_identity(package) for package in production['package']}
    for package in lock['package']:
        if package.get('source') and lock_identity(package) not in production_pins:
            raise RuntimeError(f"Proof dependency does not match production lock: {package['name']}")

    target = root / 'target/verification-dependencies'
    overrides = {'CARGO_TARGET_DIR': str(target), 'CARGO_INCREMENTAL': '0', 'CARGO_PROFILE_DEV_DEBUG': '0'}
    environment = os.environ | overrides
    configuration = configuration_sources(root, production_lock, environment)
    cargo = ['cargo', '+' + pin['rust_toolchain']]
    metadata_commands = [cargo + ['metadata', '--locked', '--format-version=1', '--manifest-path', str(path)]
                         for path in [manifest, root / 'Cargo.toml']]
    metadata, packages, bindings = read_resolution(metadata_commands, root, environment, manifest, root / 'Cargo.toml', lock)
    sources = dependency_sources(packages)
    export_context, verified_overrides = prepare_export(root, pin, packages, target, verus_path)
    overrides.update(verified_overrides)
    environment = os.environ | overrides
    if configuration != configuration_sources(root, production_lock, environment):
        raise RuntimeError('Proof dependency configuration changed during resolution')
    build_command = cargo + ['build', '--locked', '--manifest-path', str(manifest), '--message-format=json']
    result = subprocess.run(build_command, cwd=root, env=environment, capture_output=True, text=True)
    reports.mkdir(parents=True, exist_ok=True)
    (reports / 'dependencies-build.stdout').write_text(result.stdout)
    (reports / 'dependencies-build.stderr').write_text(result.stderr)
    if result.returncode:
        raise RuntimeError(f'Proof dependency build failed (exit {result.returncode}); see {reports}')
    artifacts = [item for item in map(json.loads, result.stdout.splitlines())
                 if item.get('reason') == 'compiler-artifact']
    allowed_ids = {p['id'] for p in packages} | {root_package(metadata, manifest)['id']}
    if any(item['package_id'] not in allowed_ids for item in artifacts):
        raise RuntimeError('Unexpected package in proof dependency build artifacts')
    num_ids = {p['id'] for p in packages if p['name'] == 'num'}
    num = [item for item in artifacts if item['package_id'] in num_ids
           and item['target']['name'] == 'num' and item['target']['kind'] == ['lib']]
    if len(num) != 1 or len(num_ids) != 1:
        raise RuntimeError('Expected exactly one production-bound num library')
    libraries = [name for name in num[0]['filenames'] if name.endswith('.rlib')]
    if len(libraries) != 1:
        raise RuntimeError('Expected exactly one production-bound num rlib')
    binaries = {name: sha256(Path(name)) for item in artifacts for name in item['filenames']}
    imports, verified_metadata = bind_export(export_context, artifacts)
    arguments = ['--extern', 'num=' + libraries[0], '-L', 'dependency=' + str(Path(libraries[0]).parent), *imports]
    binaries[verified_metadata['export']] = verified_metadata['export_sha256']
    evidence = {
        'metadata_commands': metadata_commands, 'build_command': build_command,
        'build_exit_code': result.returncode, 'packages': packages, 'bindings': bindings,
        'sources': sources, 'artifacts': binaries, 'verus_arguments': arguments,
        'verified_metadata': verified_metadata,
        'resolution': {'root': str(root), 'production_lock': str(production_lock),
                       'proof_manifest': str(manifest), 'production_manifest': str(root / 'Cargo.toml'),
                       'environment_overrides': overrides, 'configuration_sources': configuration},
        'scope': 'Actual Sign and BigInt/BigUint metadata methods are verified through a fresh Cargo compilation and bound export. Numerical metadata requires normalized representation and u64 count ranges; constructors, arithmetic, equality, approximation truth and caller premises remain unproved.',
    }
    check_dependency_identity(evidence)
    (reports / 'dependencies.json').write_text(json.dumps(evidence, indent=2) + '\n')
    return arguments, evidence


def check_dependency_identity(evidence):
    if any(package['name'] == 'num-bigint' for package in evidence['packages']):
        if 'verified_metadata' not in evidence:
            raise RuntimeError('Verified dependency metadata binding is missing')
        check_export(evidence['verified_metadata'])
    if evidence['sources'] != dependency_sources(evidence['packages']):
        raise RuntimeError('Proof dependency sources changed during verification')
    if any(not Path(name).is_file() or sha256(Path(name)) != expected for name, expected in evidence['artifacts'].items()):
        raise RuntimeError('Proof dependency artifacts changed during verification')
    if 'resolution' not in evidence:
        raise RuntimeError('Proof dependency resolution binding is missing')
    context = evidence['resolution']
    root = Path(context['root'])
    production_lock = Path(context['production_lock'])
    environment = os.environ | context['environment_overrides']
    if context['configuration_sources'] != configuration_sources(root, production_lock, environment):
        raise RuntimeError('Proof dependency configuration changed during verification')
    lock = tomllib.loads(Path(context['proof_manifest']).with_name('Cargo.lock').read_text())
    _, packages, bindings = read_resolution(evidence['metadata_commands'], root, environment,
        Path(context['proof_manifest']), Path(context['production_manifest']), lock)
    if packages != evidence['packages'] or bindings != evidence['bindings']:
        raise RuntimeError('Proof dependency resolution changed during verification')
    if evidence['sources'] != dependency_sources(packages):
        raise RuntimeError('Proof dependency sources changed while rechecking resolution')
    if any(not Path(name).is_file() or sha256(Path(name)) != expected for name, expected in evidence['artifacts'].items()):
        raise RuntimeError('Proof dependency artifacts changed while rechecking resolution')
    if context['configuration_sources'] != configuration_sources(root, production_lock, environment):
        raise RuntimeError('Proof dependency configuration changed while rechecking resolution')
    if 'verified_metadata' in evidence:
        check_export(evidence['verified_metadata'])
