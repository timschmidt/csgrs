"""Freeze parent and candidate with process-owned allocation and shared controls."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tomllib

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-13-native-tower-sign-retention'
WORK = ROOT / 'target/drc-process-allocator-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    scope = json.loads((E / 'scope-v1.json').read_text())
    parent = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    assert sha(PARENT / 'preparation-v1/protocol.json') == scope['parent_preparation_sha256']
    original = Path(parent['source'])
    expected = json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    guard = sources()
    assert len(expected) == 10131 and len(guard) == 1134
    assert guard == parent['main_source_sha256'] and len(parent['pending_native_sha256']) == 35
    assert all(sha(original / name) == digest for name, digest in expected.items())
    assert all(sha(ROOT / name) == digest for name, digest in parent['pending_native_sha256'].items())
    for row in scope['source_review']:
        assert sha(Path(row['path'])) == row['sha256']
    cache = Path('/tmp/csgrs-properties-cargo/registry/src/index.crates.io-1949cf8c6b5b557f')
    dependencies = {}
    packages = tomllib.loads((original / 'Cargo.lock').read_text())['package']
    for package in ['mimalloc-0.1.52', 'libmimalloc-sys-0.1.49']:
        directory = cache / package
        checksum = directory / '.cargo-checksum.json'
        manifest = json.loads(checksum.read_text())
        package_name, version = package.rsplit('-', 1)
        locked = next(p for p in packages if p['name'] == package_name and p['version'] == version)
        assert manifest['package'] == locked['checksum']
        dependencies[str(checksum)] = sha(checksum)
        for name, digest in manifest['files'].items():
            path = directory / name
            assert path.resolve().is_relative_to(directory.resolve()) and sha(path) == digest
            dependencies[str(path)] = digest
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    edits = json.loads((E / 'runtime-edits-v1.json').read_text())
    variants = {}
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    for variant in ['baseline', 'candidate']:
        source = WORK / variant / 'source'
        shutil.copytree(original, source)
        added = ['crates/hyperdrc/src/allocator_contract_tests.rs']
        if variant == 'candidate':
            added.append('crates/hyperdrc/src/process_allocator.rs')
        for name in added:
            shutil.copyfile(E / Path(name).name, source / name)
        for name, replacements in edits[variant].items():
            text = (source / name).read_text()
            for row in replacements:
                assert text.count(row['before']) == 1, name
                text = text.replace(row['before'], row['after'])
            (source / name).write_text(text)
            for row in reversed(replacements):
                assert text.count(row['after']) == 1, name
                text = text.replace(row['after'], row['before'])
            assert text == (original / name).read_text(), name
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source, env=env,
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        with (out / (variant + '-metadata.json')).open('w') as stdout, (out / (variant + '-metadata.stderr')).open('w') as stderr:
            subprocess.run(['cargo', 'metadata', '--locked', '--offline', '--no-deps', '--format-version=1'],
                           cwd=source, env=env, stdout=stdout, stderr=stderr, check=True)
        current = {name: sha(source / name) for name in sorted(set(expected) | set(added))}
        assert len(current) == (10132 if variant == 'baseline' else 10133)
        assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(current)
        changes = {name: digest for name, digest in current.items() if expected.get(name) != digest}
        assert set(changes) == set(edits[variant]) | set(added)
        for name, replacements in edits[variant].items():
            text = (source / name).read_text()
            for row in reversed(replacements):
                text = text.replace(row['after'], row['before'])
            assert text == (original / name).read_text(), name
        current_packages = tomllib.loads((source / 'Cargo.lock').read_text())['package']
        if variant == 'candidate':
            old_packages = {p['name'] + '@' + p['version']: p for p in packages}
            new_packages = {p['name'] + '@' + p['version']: p for p in current_packages}
            assert old_packages.keys() == new_packages.keys()
            changed = [key for key in old_packages if old_packages[key] != new_packages[key]]
            assert changed == ['hyperdrc@0.3.0']
            row = new_packages[changed[0]].copy()
            row['dependencies'] = [d for d in row['dependencies'] if d != 'mimalloc']
            assert row == old_packages[changed[0]]
        else:
            assert current_packages == packages
        hashes = out / (variant + '-source-hashes.json.gz')
        hashes.write_bytes(gzip.compress((json.dumps(current, sort_keys=True) + '\n').encode(), mtime=0))
        patch = ''.join(''.join(difflib.unified_diff(
            (original / name).read_text().splitlines(True) if name in expected else [],
            (source / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changes)
        (out / (variant + '.patch.gz')).write_bytes(gzip.compress(patch.encode(), mtime=0))
        with tarfile.open(out / (variant + '-source-delta.tar.gz'), 'w:gz') as archive:
            for name in changes:
                archive.add(source / name, arcname=name)
        variants[variant] = {'source': str(source), 'target': str(WORK / variant / 'target'),
                             'files': len(current), 'source_hashes_sha256': sha(hashes),
                             'changes_sha256': changes, 'original_bytes_recovered': True}
    reference = 'crates/hyperdrc/src/allocator_contract_tests.rs'
    assert (Path(variants['baseline']['source']) / reference).read_bytes() == (Path(variants['candidate']['source']) / reference).read_bytes()
    for name, entries in [('parent-source', expected), ('dependency-source', dependencies)]:
        (out / (name + '-hashes.json.gz')).write_bytes(gzip.compress((json.dumps(entries, sort_keys=True) + '\n').encode(), mtime=0))
    inputs = ['scope-v1.json', 'runtime-edits-v1.json', 'process_allocator.rs', 'allocator_contract_tests.rs',
              'common_v1.py', 'qualify_v1.py', 'assess_qualification_v1.py']
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'inputs_sha256': {name: sha(E / name) for name in inputs},
                'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'),
                'parent_source': str(original), 'variants': variants,
                'main_source_sha256': guard, 'pending_native_sha256': parent['pending_native_sha256'],
                'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
                'dependency_hashes_sha256': sha(out / 'dependency-source-hashes.json.gz'),
                'guarded_dependency_files': len(dependencies),
                'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
                'runtime_admitted': False, 'plan_complete': False}
    assert sources() == guard
    assert all(sha(original / name) == digest for name, digest in expected.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'variants': {k: {'files': v['files'], 'changes': len(v['changes_sha256'])} for k, v in variants.items()},
                      'guarded_dependency_files': len(dependencies), 'main_and_parent_unchanged': True}))


if __name__ == '__main__':
    main()
