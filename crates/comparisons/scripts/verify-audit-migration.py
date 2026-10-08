#!/usr/bin/env python3
"""Validate archive bytes, staged blobs, entry-point syntax and source paths.

This verifies the relocation; it does not execute historical defect probes or
overwrite their evidence. Run after `consolidate-audits.py stage`.
"""
import argparse
import ast
import collections
import concurrent.futures
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

REPO = Path(__file__).resolve().parents[1]
ARCHIVE = REPO / 'audits'


def index_check(manifest):
    result = subprocess.run(['git', 'ls-files', '--stage', '-z', 'audits'],
                            cwd=REPO, check=True, stdout=subprocess.PIPE)
    index = {}
    for record in result.stdout.split(b'\0'):
        if record:
            info, path = record.split(b'\t', 1)
            mode, oid, stage = info.split()
            assert stage == b'0', 'unmerged audit path'
            index[path.decode()] = (mode, oid)
    expected = collections.defaultdict(set)
    for f in manifest['files']:
        key = 'audits/' + f['destination']
        assert key in index, 'missing staged artifact: ' + key
        mode, oid = index[key]
        assert mode == (b'100755' if f['mode'] & 0o111 else b'100644'), key
        expected[oid].add(f['sha256'])
    proc = subprocess.Popen(['git', 'cat-file', '--batch'], cwd=REPO,
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    try:
        for oid, hashes in expected.items():
            proc.stdin.write(oid + b'\n')
            proc.stdin.flush()
            actual_oid, kind, size = proc.stdout.readline().split()
            assert actual_oid == oid and kind == b'blob'
            remaining = int(size)
            digest = hashlib.sha256()
            while remaining:
                data = proc.stdout.read(min(remaining, 1024 * 1024))
                assert data, 'truncated git blob'
                digest.update(data)
                remaining -= len(data)
            assert proc.stdout.read(1) == b'\n'
            assert hashes == {digest.hexdigest()}, 'staged byte transformation: ' + oid.decode()
    finally:
        proc.stdin.close()
        proc.stdout.close()
        assert proc.wait() == 0
    return {'artifacts': len(manifest['files']), 'distinct_blobs': len(expected),
            'staged_bytes_match_original': True}


def syntax_check(manifest):
    files = []
    for row in manifest['files']:
        p = Path(row['destination'])
        if p.suffix not in {'.mjs', '.py', '.sh'}:
            continue
        if not (len(p.parts) == 2 and p.parts[0] == 'tmp' or '-qualification/' in str(p)):
            continue
        if any(x in p.parts for x in ('snapshot', 'source', 'before-source',
                                     'hyperreal', 'hypersolve', 'hypercurve')):
            continue
        files.append(p)

    def check(p):
        file = ARCHIVE / p
        if p.suffix == '.py':
            ast.parse(file.read_bytes(), str(file))
        else:
            command = ['node', '--check', str(file)] if p.suffix == '.mjs' else ['bash', '-n', str(file)]
            subprocess.run(command, capture_output=True, check=True)

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(check, files))
    return {'checked': len(files), 'by_extension': dict(collections.Counter(p.suffix for p in files))}


def rust_targets():
    paths = ['tmp/realistic-audit', 'tmp/boehm-reals-audit',
             'tmp/published-reals-audit', 'tmp/interval-computable-audit',
             'workspace/exact-real-references/escardo-qualification']
    results = []
    for path in paths:
        result = subprocess.run(['cargo', 'metadata', '--manifest-path',
                                 str(ARCHIVE / path / 'Cargo.toml'), '--no-deps',
                                 '--format-version', '1'], capture_output=True, text=True, check=True)
        package = json.loads(result.stdout)['packages'][0]
        missing = [t['src_path'] for t in package['targets'] if not Path(t['src_path']).is_file()]
        assert not missing, str(missing)
        results.append({'package': package['name'], 'targets': len(package['targets'])})
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    manifest = json.loads((ARCHIVE / 'migration.json').read_text())
    spec = importlib.util.spec_from_file_location('migration', REPO / 'scripts/consolidate-audits.py')
    migration = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(migration)
    migration.verify(manifest)
    report = {'scope': 'migration integrity, staged files and representative entry points',
              'archive': migration.summary(manifest), 'index': index_check(manifest),
              'syntax': syntax_check(manifest), 'rust_packages': rust_targets()}
    result = subprocess.run(['node', str(ARCHIVE / 'workspace/exact-real-references/constructible-qualification/verify-dependencies.mjs')],
                            capture_output=True, text=True, check=True)
    report['dependency_archives'] = json.loads(result.stdout)
    print(json.dumps(report, indent=2))
    if args.output:
        args.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
