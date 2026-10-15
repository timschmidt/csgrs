"""Preserve each primitive-failures acceptance trial and its complete result.

Run once per phase. A changed source or command requires a fresh phase;
completed or failed trials are never resumed, filtered, or overwritten.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tarfile
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def sources():
    names = subprocess.check_output([
        'git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard',
        '--', 'src', 'tests', 'crates', 'examples', 'benchmarks/rust', 'fuzz', 'bindings', 'Cargo.toml', 'Cargo.lock',
        '.cargo/config.toml', '.rustfmt.toml', 'AGENTS.md',
    ], cwd=ROOT).decode().split('\0')
    return {name: sha(ROOT / name) for name in sorted(set(names)) if name and (
        Path(name).suffix in {'.rs', '.c', '.cpp', '.h', '.hpp', '.py', '.js', '.cjs', '.ts'}
        or Path(name).name in {'Cargo.toml', 'Cargo.lock', 'AGENTS.md'}
        or name in {'.cargo/config.toml', '.rustfmt.toml'}
    )}


def main():
    phase = sys.argv[1]
    commands = json.loads((E / (phase + '.json')).read_text())
    out = E / phase
    out.mkdir(exist_ok=False)
    target = ROOT / 'target' / 'primitive-failures' / phase
    assert not target.exists()
    initial = sources()
    runner = sha(Path(__file__))
    with gzip.open(out / 'source-hashes.json.gz', 'wt') as f:
        json.dump(initial, f, sort_keys=True)
    (out / 'tracked.patch').write_bytes(subprocess.check_output([
        'git', 'diff', '--binary', 'HEAD', '--', 'src', 'tests', 'crates', 'examples', 'benchmarks/rust', 'fuzz', 'bindings', 'Cargo.toml',
        'Cargo.lock', '.cargo/config.toml', '.rustfmt.toml', 'AGENTS.md',
    ], cwd=ROOT))
    untracked = subprocess.check_output([
        'git', 'ls-files', '-z', '--others', '--exclude-standard', '--', 'src', 'tests', 'crates', 'examples', 'benchmarks/rust', 'fuzz', 'bindings',
    ], cwd=ROOT).decode().split('\0')
    with tarfile.open(out / 'untracked-sources.tar.gz', 'w:gz') as archive:
        for name in sorted(set(untracked) & initial.keys()):
            archive.add(ROOT / name, arcname=name)
    env = {
        'CARGO_HOME': '/tmp/csgrs-properties-cargo',
        'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
        'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
        'CARGO_BUILD_JOBS': '2', 'CARGO_TARGET_DIR': str(target),
    }
    dump(out / 'protocol.json', {
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'runner_sha256': runner, 'phase': phase, 'commands': commands, 'environment': env,
        'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'properties': ['P6', 'P8', 'P9', 'P17', 'P18', 'P25', 'P35', 'P37'],
        'owning_layers': ['CSGRS primitive constructors and successful-result cache', 'scalar and language adapters'],
        'domain': 'Strict native primitives: nonnegative dimensions/radii, directed signed frustum height, valid sample minima, representable buffer capacities. Proven degeneracies are canonical empty only after all inputs are validated. Ring tori require major > minor unless minor = 0.',
        'consumers': ['SolidExt', 'scalar adapters', 'C ABI', 'WASM'],
        'missing_acceptance_test': 'tests/primitive_failures.rs',
        'semantic_oracle': 'Typed failures remain failures through adapters; exact box coordinates/volume, canonical empty composition, native topology, and preservation of existing full-resolution fixtures.',
        'wall_limit_seconds_per_command': 1800,
        'limitations': 'External wall containment only; no internal total-work or heap-byte bound claimed. Focused qualification, not full CI, release validation, or performance comparison.',
        'previous_goal_turn': 'Progress: implemented and committed empty composition with native and real binding qualification.',
    })
    rows = []
    for name, command in commands:
        assert sources() == initial and sha(Path(__file__)) == runner
        print(json.dumps({'started': name, 'phase': phase}), flush=True)
        start = time.monotonic()
        with (out / (name + '.stdout')).open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            child = subprocess.Popen(command, cwd=ROOT, env=os.environ | env,
                                     stdout=stdout, stderr=stderr, start_new_session=True)
            try:
                code = child.wait(timeout=1800)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
                code = 124
        unchanged = sources() == initial and sha(Path(__file__)) == runner
        errors = (out / (name + '.stderr')).read_text()
        binaries = {path: sha(ROOT / path) for path in re.findall(r'^\s*Running .* \(([^\n]+)\)$', errors, re.M)}
        row = {
            'name': name, 'exit_code': code, 'seconds': time.monotonic() - start,
            'sources_unchanged': unchanged, 'binaries_sha256': binaries,
            'stdout_sha256': sha(out / (name + '.stdout')),
            'stderr_sha256': sha(out / (name + '.stderr')),
        }
        rows.append(row)
        dump(out / 'result.json', {'commands': rows, 'passed': len(rows) == len(commands) and all(r['exit_code'] == 0 and r['sources_unchanged'] for r in rows)})
        print(json.dumps(row), flush=True)
        assert unchanged
        if code:
            raise SystemExit(code)


if __name__ == '__main__':
    main()
