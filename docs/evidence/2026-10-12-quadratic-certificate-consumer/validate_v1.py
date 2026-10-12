"""Freeze the direct quadratic consumer and run its initial correctness gates.

The eight-node consumer retains exact rational coefficients, then uses the
existing bounded recognizer on every decline. No original fallback, fixture,
term limit, or runtime assertion is removed. This is an isolated prototype,
not an admission of either the scalar cache or the pending native geometry.
"""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/quadratic-certificate-consumer/source'
BASE = ROOT / 'target/persistent-laurent-cache/candidate/source'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'validation-v1'
    out.mkdir(exist_ok=False)
    preparation = json.loads((E / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    changed = ['crates/hyperreal/src/' + name for name in [
        'computable/node/algebra.rs', 'computable/node/structural_analysis.rs',
        'computable/node/cached_quadratic.rs', 'real/arithmetic/facts.rs',
        'real/arithmetic/pi_laurent_compaction.rs']]
    current = {name: sha(SOURCE / name) for name in sorted(set(original) | set(changed))}
    assert {name for name, value in current.items() if original.get(name) != value} == set(changed)
    assert sources() == preparation['main_source_sha256']
    assert all(sha(BASE / name) == expected for name, expected in original.items())
    with tarfile.open(out / 'candidate-delta.tar.gz', 'w:gz') as archive:
        for name in changed:
            archive.add(SOURCE / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (BASE / name).read_text().splitlines(True) if name in original else [],
        (SOURCE / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changed)
    (out / 'candidate.patch').write_text(patch)
    commands = [
        ('format', ['cargo', 'fmt', '--all', '--', '--check']),
        ('consumer-format', ['rustfmt', '--edition', '2024', '--check',
                             'crates/hyperreal/src/computable/node/cached_quadratic.rs']),
        ('quadratic-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', 'cached_quadratic::tests', '--', '--test-threads=1']),
        ('cache-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', 'persistent_laurent_cache_tests', '--', '--test-threads=1']),
        ('strict-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--all-targets', '--', '-D', 'warnings']),
        ('scalar-library', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--', '--test-threads=2']),
        ('scalar-doctests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--doc']),
        ('scalar-docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '--all-features', '--no-deps']),
    ]
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
           'CARGO_TARGET_DIR': str(ROOT / 'target/quadratic-certificate-consumer/target'),
           'RUSTDOCFLAGS': '-D warnings'}
    protocol = {'runner_sha256': sha(Path(__file__)), 'preparation_sha256': sha(E / 'preparation.json'),
                'source_changes_sha256': {name: current[name] for name in changed},
                'candidate_archive_sha256': sha(out / 'candidate-delta.tar.gz'),
                'candidate_patch_sha256': sha(out / 'candidate.patch'),
                'commands': commands, 'environment': env, 'scope': __doc__,
                'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results = []
    for name, command in commands:
        assert sources() == preparation['main_source_sha256']
        assert all(sha(SOURCE / p) == expected for p, expected in current.items())
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | env,
                                  stdout=stdout, stderr=stderr).returncode
        unchanged = all(sha(SOURCE / p) == expected for p, expected in current.items())
        main_unchanged = sources() == preparation['main_source_sha256']
        result = {'name': name, 'exit_code': code, 'seconds': time.monotonic() - started,
                  'candidate_unchanged': unchanged, 'main_unchanged': main_unchanged,
                  'stdout_sha256': sha(out / (name + '.stdout')),
                  'stderr_sha256': sha(out / (name + '.stderr'))}
        results.append(result)
        complete = len(results) == len(commands) and all(r['exit_code'] == 0 for r in results)
        (out / 'result.json').write_text(json.dumps({'phases': results, 'complete': complete}, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        assert unchanged and main_unchanged
        if code:
            raise SystemExit(code)
    assert all(sha(BASE / name) == expected for name, expected in original.items())


if __name__ == '__main__':
    main()
