"""Validate explicitly in-scope existing worktree changes without changing inputs.

Usage: python3 validate.py NEW_PHASE fmt|scalar|geometry
Commands run sequentially; each phase preserves its original source identities
and every outcome. A failure stops the phase and requires a fresh output name.
This focused validation is not the repository's complete pre-push matrix.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def sources():
    crates = ['hyperreal', 'hyperlimit', 'hyperlattice', 'hypersolve', 'hypercurve', 'hyperdrc']
    paths = subprocess.check_output(
        ['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard', '--',
         *['crates/' + name for name in crates]], cwd=ROOT).decode().split('\0')
    selected = {name for name in paths if name and (
        Path(name).suffix in {'.rs', '.py', '.json', '.snap'}
        or Path(name).name in {'Cargo.toml', 'Cargo.lock'})}
    selected.update(['Cargo.toml', 'Cargo.lock', '.cargo/config.toml', '.rustfmt.toml'])
    return {name: sha(ROOT / name) for name in sorted(selected)}


def main():
    phase, group = sys.argv[1:]
    assert Path(phase).name == phase
    out = E / phase
    out.mkdir(exist_ok=False)
    groups = {
        'fmt': [('workspace-fmt', ['cargo', 'fmt', '--all', '--', '--check'], {})],
        'scalar': [
            ('default-tests', ['cargo', 'test', '--locked', '-p', 'hyperreal', '--lib', '--tests'], {}),
            ('all-feature-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--tests'], {}),
            ('all-feature-doctests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--doc'], {}),
            ('default-clippy', ['cargo', 'clippy', '--locked', '-p', 'hyperreal', '--all-targets', '--', '-D', 'warnings'], {}),
            ('all-feature-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-targets', '--all-features', '--', '-D', 'warnings'], {}),
            ('docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '--all-features', '--no-deps'], {'RUSTDOCFLAGS': '-D warnings'}),
        ],
        'geometry': [
            ('clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-targets', '--all-features', '--', '-D', 'warnings'], {}),
            ('curve-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', '--tests', '--', '--test-threads=2'], {}),
            ('drc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--tests', '--', '--test-threads=2'], {}),
        ],
    }
    commands = groups[group]
    initial = sources()
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    protocol = {'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'runner_sha256': sha(Path(__file__)), 'group': group, 'commands': commands,
                'environment': env, 'sources_sha256': initial,
                'scope': 'Existing worktree changes explicitly authorized by the user; focused validation, not full repository CI.'}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results = []
    for name, command, extra in commands:
        assert sources() == initial
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=ROOT, env=os.environ | env | extra,
                                  stdout=stdout, stderr=stderr).returncode
        unchanged = sources() == initial
        result = {'name': name, 'exit_code': code, 'seconds': time.monotonic() - started,
                  'source_unchanged': unchanged, 'stdout_sha256': sha(out / (name + '.stdout')),
                  'stderr_sha256': sha(out / (name + '.stderr'))}
        results.append(result)
        complete = len(results) == len(commands) and all(r['exit_code'] == 0 and r['source_unchanged'] for r in results)
        (out / 'result.json').write_text(json.dumps({'phases': results, 'complete': complete}, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        assert unchanged
        if code:
            raise SystemExit(code)


if __name__ == '__main__':
    main()
