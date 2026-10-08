#!/usr/bin/env python3
"""Inventory, relocate, and verify the historical exact-real audit files.

Archived bytes are immutable. Old paths become relative compatibility symlinks.
Build products and installed dependencies stay in their original locations.
Run `plan` before `apply`; `verify` is read-only and checks every archived byte.
"""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess

REPO = Path(__file__).resolve().parents[1]
WORKSPACE = REPO.parent
ARCHIVE = REPO / 'audits'
MANIFEST = ARCHIVE / 'migration.json'
LEDGERS = ('EXACTCORELIB_AUDIT_PROGRESS.md',
           'CONSTRUCTIVE_REAL_AUDIT_PROGRESS.md',
           'EXACT_REAL_ECOSYSTEM_AUDIT_PROGRESS.md', 'exact_reals_references.txt')
PREFIX = re.compile(r'^(?:aern|ariadne|boehm|boost-real|br21_|cdar|computable|'
                    r'constructible|escardo|edalat|fast-reals|fewdigits|flatsurf|'
                    r'haskell-creal|hera|hyper(?:-|real|curve|lattice|limit|solve|tri)|'
                    r'ic-reals|interval-computable|ireal|irram|numbers|plume|'
                    r'published-reals|realistic|reals-|reallib|ruffini|spigot|xrc|exactcore)', re.I)
SKIP_DIRS = {'.git', 'node_modules', '__pycache__', '.stack-work', 'dist-newstyle',
             '.pixi', '.cargo', '.rustup', 'CMakeFiles', 'autom4te.cache', '.deps',
             '.libs', 'incremental', '.fingerprint', 'site-packages', '.venv'}
SKIP_EXT = {'.o', '.a', '.so', '.dylib', '.dll', '.exe', '.rlib', '.rmeta',
            '.hi', '.dyn_hi', '.dyn_o', '.class', '.pyc', '.lo', '.la', '.lai',
            '.profraw', '.profdata', '.gcno', '.gcda'}
CODE_EXT = {'.rs', '.py', '.mjs', '.cjs', '.js', '.ts', '.sh', '.bash', '.c',
            '.cc', '.cpp', '.h', '.hpp', '.hs', '.lhs', '.java', '.jl', '.ml',
            '.mli', '.lisp', '.cl', '.tcl', '.v', '.agda', '.lean', '.patch'}


def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def directory_reason(p):
    if p.name.startswith(('aern2-stack.', 'flatsurf-pixi.')):
        return 'downloaded compiler/package installation'
    if p.name in SKIP_DIRS:
        return 'generated output or installed dependency'
    if p.name == 'target' or p.name.startswith('target-') or p.name == '.audit-targets':
        return 'Cargo build output'
    if (p / '.rustc_info.json').exists() or (p / 'CACHEDIR.TAG').exists():
        return 'compiler cache'
    if any(s in p.name for s in ('browser-profile', 'compiler-cache', '-coverage.')):
        return 'browser/compiler/profiling cache'
    return None


def file_reason(p):
    if p.suffix in {'.perf', '.data', '.zst'}:
        return 'raw binary profiler capture (reports and analysis code archived)'
    if p.suffix in SKIP_EXT or '.so.' in p.name:
        return 'compiled output'
    with p.open('rb') as f:
        start = f.read(4096)
    if start.startswith((b'\x7fELF', b'MZ', b'!<arch>\n', b'\x00asm')):
        return 'compiled output (magic)'
    if p.name in {'.git', 'CMakeCache.txt', 'cmake_install.cmake', 'compile_commands.json'}:
        return 'local build metadata'
    return None


def origins():
    for name in LEDGERS:
        yield WORKSPACE / name, 'workspace/' + name
    refs = WORKSPACE / 'exact-real-references'
    for p in sorted(refs.iterdir()):
        if p.is_file() and p.suffix in {'.md', '.tsv', '.json', '.txt'}:
            yield p, 'workspace/exact-real-references/' + p.name
        elif p.name.endswith('-qualification'):
            yield p, 'workspace/exact-real-references/' + p.name
    for p in sorted(WORKSPACE.iterdir()):
        if p.name.startswith('.audit-') or p.name in {'audit', 'audit-experiments'}:
            yield p, 'workspace/' + p.name
    for p in sorted(Path('/tmp').iterdir()):
        if PREFIX.match(p.name) or p.name == 'cpp_minimal.cpp':
            yield p, 'tmp/' + p.name


def scan():
    files, skipped, links, roots = [], [], [], []

    def visit(p, destination):
        if p.is_symlink():
            links.append({'source': str(p), 'target': os.readlink(p),
                          'destination': destination})
            return
        if p.is_dir():
            reason = directory_reason(p)
            if reason:
                skipped.append({'source': str(p), 'reason': reason})
                return
            for child in sorted(p.iterdir()):
                visit(child, destination + '/' + child.name)
            return
        if not p.is_file():
            skipped.append({'source': str(p), 'reason': 'missing or non-regular'})
            return
        reason = file_reason(p)
        if reason:
            skipped.append({'source': str(p), 'reason': reason})
            return
        size = p.stat().st_size
        files.append({'source': str(p), 'destination': destination,
                      'bytes': size, 'sha256': digest(p),
                      'mode': stat.S_IMODE(p.stat().st_mode),
                      'kind': 'code' if p.suffix in CODE_EXT else 'support'})

    for p, destination in origins():
        roots.append(str(p))
        visit(p, destination)
    return {'schema': 1, 'original_workspace': str(WORKSPACE),
            'roots': roots, 'files': files, 'existing_links': links,
            'excluded': skipped}


def summary(m):
    return {'files': len(m['files']), 'bytes': sum(f['bytes'] for f in m['files']),
            'kinds': dict(collections.Counter(f['kind'] for f in m['files'])),
            'existing_links': len(m['existing_links']),
            'excluded': len(m['excluded'])}


def verify(m, require_links=True, git_modes=False):
    failures = []
    for f in m['files']:
        p, old = ARCHIVE / f['destination'], Path(f['source'])
        if not p.is_file() or p.is_symlink() or digest(p) != f['sha256']:
            failures.append('archive content mismatch: ' + str(p))
        elif (bool(p.stat().st_mode & 0o111) != bool(f['mode'] & 0o111) if git_modes
              else stat.S_IMODE(p.stat().st_mode) != f['mode']):
            failures.append('archive mode mismatch: ' + str(p))
        if require_links and (not old.is_symlink() or old.resolve() != p):
            failures.append('compatibility link mismatch: ' + str(old))
    if failures:
        raise RuntimeError('\n'.join(failures[:40]) + f'\n{len(failures)} failures')
    print(json.dumps({'verified': summary(m), 'compatibility_links': require_links,
                      'mode_contract': 'git-executable-bit' if git_modes else 'original-permissions'}))


def apply(m):
    # Validate the entire selection before changing any source location.
    for f in m['files']:
        old, new = checked_paths(f)
        if new.exists() and digest(new) != f['sha256']:
            raise RuntimeError('destination conflict: ' + str(new))
        if not old.is_file() or digest(old) != f['sha256']:
            raise RuntimeError('source changed: ' + str(old))
    for f in m['files']:
        old, new = checked_paths(f)
        if old.is_symlink() and old.resolve() == new:
            continue  # Resume an interrupted apply.
        new.parent.mkdir(parents=True, exist_ok=True)
        if not new.exists():
            shutil.copy2(old, new)
        if digest(new) != f['sha256']:
            raise RuntimeError('copy verification failed: ' + str(new))
        # Replace the original only after the complete copy has been verified.
        alias = old.with_name(old.name + '.audit-migration-link')
        target = os.path.relpath(new, old.parent)
        if alias.is_symlink():
            if os.readlink(alias) != target:
                raise RuntimeError('temporary link conflict: ' + str(alias))
        elif alias.exists():
            raise RuntimeError('temporary path conflict: ' + str(alias))
        else:
            alias.symlink_to(target)
        alias.replace(old)
    verify(m)


def checked_paths(f):
    old, new = Path(f['source']), ARCHIVE / f['destination']
    relative = Path(f['destination'])
    if relative.is_absolute() or '..' in relative.parts or ARCHIVE.resolve() not in new.resolve().parents:
        raise RuntimeError('destination outside archive: ' + str(new))
    if not old.is_absolute() or not any(root in old.parents for root in (WORKSPACE, Path('/tmp'))):
        raise RuntimeError('source outside audit locations: ' + str(old))
    if '.git' in old.parts or ARCHIVE in old.parents:
        raise RuntimeError('invalid audit source: ' + str(old))
    return old, new


def setup(m):
    """Recreate dependency links in the archive without changing saved sources."""
    local_links = []
    destinations = {f['source']: ARCHIVE / f['destination'] for f in m['files']}
    original_workspace = Path(m['original_workspace'])

    def map_target(target):
        text = str(target)
        if text in destinations:
            return destinations[text], True
        for root in sorted(m['roots'], key=len, reverse=True):
            p = Path(root)
            if target == p or p in target.parents:
                if original_workspace == p or original_workspace in p.parents:
                    mapped = ARCHIVE / 'workspace' / target.relative_to(original_workspace)
                elif Path('/tmp') in p.parents:
                    mapped = ARCHIVE / 'tmp' / target.relative_to('/tmp')
                else:
                    continue
                if mapped.exists():
                    return mapped, True
        if target == original_workspace or original_workspace in target.parents:
            return WORKSPACE / target.relative_to(original_workspace), False
        return target, False

    def link(p, target, internal=False):
        if p.is_symlink():
            if p.resolve() != target.resolve():
                raise RuntimeError('dependency link conflict: ' + str(p))
        elif p.exists():
            raise RuntimeError('dependency path conflict: ' + str(p))
        else:
            p.parent.mkdir(parents=True, exist_ok=True)
            p.symlink_to(os.path.relpath(target, p.parent), target_is_directory=target.is_dir())
        if not internal:
            local_links.append('/' + str(p.relative_to(ARCHIVE)))

    for d in json.loads((ARCHIVE / 'dependencies.json').read_text()):
        link(ARCHIVE / 'workspace' / d['path'], WORKSPACE / d['path'])
    for d in m['existing_links']:
        target = Path(d['target'])
        if not target.is_absolute():
            target = Path(os.path.normpath(str(Path(d['source']).parent / target)))
        mapped, internal = map_target(target)
        link(ARCHIVE / d['destination'], mapped, internal)
    (ARCHIVE / '.gitignore').write_text('# Local dependency links; recreate with setup.\n' +
                                       '\n'.join(sorted(set(local_links))) + '\n')
    print(json.dumps({'local_dependency_links': len(local_links),
                      'preserved_links': len(m['existing_links'])}))


def dependencies():
    rows = []
    for base in (WORKSPACE, WORKSPACE / 'exact-real-references'):
        for p in sorted(base.iterdir()):
            if not p.is_dir() or p == REPO:
                continue
            if base == WORKSPACE and not (p.name.startswith('hyper') or p.name in
                                          {'ExactCalculator', 'crcalc', 'crcalc-boehm', 'exactCorelib-main'}):
                continue
            if base.name == 'exact-real-references' and p.name.endswith('-qualification'):
                continue
            row = {'path': str(p.relative_to(WORKSPACE)), 'role': 'external dependency'}
            if (p / '.git').exists():
                for key, args in [('commit', ['rev-parse', 'HEAD']),
                                  ('origin', ['remote', 'get-url', 'origin'])]:
                    r = subprocess.run(['git', '-C', str(p), *args], capture_output=True, text=True)
                    if r.returncode == 0:
                        row[key] = r.stdout.strip()
            row['licenses'] = [str(q.relative_to(p)) for q in p.iterdir()
                               if q.is_file() and re.search('license|copyright|copying', q.name, re.I)]
            rows.append(row)
    return rows


def stage(m):
    """Stage the explicit archive, including fixtures hidden by donor ignores."""
    verify(m, require_links=False)
    paths = ['audits/' + f['destination'] for f in m['files']]
    for f in m['existing_links']:
        p = ARCHIVE / f['destination']
        if p.is_symlink() and ARCHIVE in p.resolve().parents:
            paths.append('audits/' + f['destination'])
    paths.extend(['audits/migration.json', 'audits/dependencies.json',
                  'audits/README.md', 'audits/.gitignore', 'README.md',
                  'THIRD_PARTY.md', 'scripts/consolidate-audits.py',
                  'scripts/test-consolidate-audits.py',
                  'scripts/verify-audit-migration.py', '.gitignore'])
    if (ARCHIVE / 'migration-verification.json').exists():
        paths.append('audits/migration-verification.json')
    subprocess.run(['git', '--literal-pathspecs', 'add', '-f', '--pathspec-from-file=-',
                    '--pathspec-file-nul'], input=('\0'.join(paths) + '\0').encode(),
                   cwd=REPO, check=True)
    print(json.dumps({'staged_explicit_paths': len(paths)}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['plan', 'replan', 'apply', 'setup', 'stage', 'verify'])
    parser.add_argument('--archive-only', action='store_true')
    parser.add_argument('--git-modes', action='store_true',
                        help='verify the executable bit Git preserves, not original host permissions')
    args = parser.parse_args()
    if args.command in {'plan', 'replan'}:
        if MANIFEST.exists():
            if args.command != 'replan':
                raise RuntimeError('A plan already exists; preserve its original-source evidence.')
            previous = json.loads(MANIFEST.read_text())
            if any((ARCHIVE / f['destination']).exists() for f in previous['files']):
                raise RuntimeError('Cannot replan after apply has started.')
        m = scan()
        ARCHIVE.mkdir(exist_ok=True)
        MANIFEST.write_text(json.dumps(m, indent=2) + '\n')
        (ARCHIVE / 'dependencies.json').write_text(json.dumps(dependencies(), indent=2) + '\n')
        print(json.dumps(summary(m)))
        groups = collections.Counter()
        for f in m['files']:
            groups['/'.join(f['destination'].split('/')[:2])] += 1
        print(json.dumps(dict(groups.most_common(25)), indent=2))
    else:
        m = json.loads(MANIFEST.read_text())
        if args.command == 'apply':
            apply(m)
        elif args.command == 'setup':
            setup(m)
        elif args.command == 'stage':
            stage(m)
        else:
            verify(m, not args.archive_only, args.git_modes)


if __name__ == '__main__':
    main()
