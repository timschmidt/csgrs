"""Preserve each curve-metaball-failures acceptance trial and its complete result.

Run once per phase. A changed source or command requires a fresh phase;
completed or failed trials are never resumed, filtered, or overwritten.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import resource
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
        Path(name).suffix in {'.rs', '.c', '.cpp', '.h', '.hpp', '.py', '.js', '.cjs', '.ts', '.go'}
        or Path(name).name in {'Cargo.toml', 'Cargo.lock', 'AGENTS.md', 'go.mod', 'go.sum'}
        or name in {'.cargo/config.toml', '.rustfmt.toml'}
    )}


def copy_seed(command, out):
    with (out / 'seed-copy.stdout').open('wb') as stdout, (out / 'seed-copy.stderr').open('wb') as stderr:
        result = subprocess.run(command, stdout=stdout, stderr=stderr)
    dump(out / 'preparation.json', {
        'command': command, 'exit_code': result.returncode,
        'runner_sha256': sha(Path(__file__)),
        'stdout_sha256': sha(out / 'seed-copy.stdout'),
        'stderr_sha256': sha(out / 'seed-copy.stderr'),
    })
    if result.returncode:
        raise SystemExit(result.returncode)


def main():
    phase = sys.argv[1]
    commands = json.loads((E / (phase + '.json')).read_text())
    out = E / phase
    out.mkdir(exist_ok=False)
    target = Path('/tmp/csgrs-curve-metaball-failures') / phase
    target.parent.mkdir(parents=True, exist_ok=True)
    assert not target.exists()
    seed = None
    preserved = {}
    seed_phase = json.loads((E / (phase + '-seed.json')).read_text())
    if seed_phase:
        seed = ROOT / seed_phase['target']
        assert not any(path.is_symlink() for path in seed.rglob('*'))
        records = json.loads((ROOT / seed_phase['record']).read_text())
        for row in records['commands']:
            preserved.update(row['binaries_sha256'])
        if seed_phase == 'bindings-v1':
            artifact_names = ['release/libcsgrs_ffi.so', 'wasm32-unknown-unknown/wasm/csgrs.wasm',
                              'wasm-bindings-node/csgrs.js', 'wasm-bindings-node/csgrs.d.ts',
                              'wasm-bindings-node/csgrs_bg.wasm']
            artifact_names += [name + '-' + language for name in ['empty_composition', 'primitive_failures'] for language in ['c', 'cpp']]
            preserved.update({str((seed / name).relative_to(ROOT)): sha(seed / name) for name in artifact_names})
        assert all(sha(ROOT / path) == digest for path, digest in preserved.items())
        if seed_phase.get('release_only'):
            target.mkdir()
            copy_seed(['cp', '-a', '--reflink=auto', str(seed / 'release'), str(target / 'release')], out)
            preserved = {name: digest for name, digest in preserved.items() if '/release/' in name}
        else:
            copy_seed(['cp', '-a', '--reflink=auto', str(seed), str(target)], out)
        for path in preserved:
            old = ROOT / path
            copy = target / old.relative_to(seed)
            assert old.stat().st_ino != copy.stat().st_ino
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
        'seeded_build_artifacts': str(seed) if seed else None,
        'preserved_prior_executables_sha256': preserved,
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'runner_sha256': runner, 'phase': phase, 'commands': commands, 'environment': env,
        'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'properties': ['P1', 'P6', 'P8', 'P9', 'P13', 'P17', 'P18', 'P20', 'P25', 'P35', 'P37'],
        'owning_layers': ['Hypersdf inverse-square field evaluation and conservative bounds', 'CSGRS planar field sampling', 'Hypercurve contour extraction and existing region arrangement'],
        'domain': 'Positive-radius planar inverse-square metaballs, positive iso, nonnegative padding and a declared grid. Empty succeeds after validation. Exact normalized F=(iso-I)/(iso+I), extended to -1 at centers, replaces cutoff and sentinel through an explicit geometry migration. Native marching-squares chords and its documented center-sign diagonal rule define the sampled proposal, with separate exact region topology and no continuous-field fidelity bound.',
        'consumers': ['curve::metaballs', 'native Hypercurve sampled-region consumers', 'shared Hypersdf field', '24x24 README fixture and 48x48 benchmark fixture'],
        'missing_acceptance_test': 'tests/curve_metaball_failures.rs',
        'semantic_oracle': 'Independent sampled diamond area 2*(20201/20200)^2, native grid topology cases and ring of sources with an excluded center, retained typed errors and downstream region/extrusion use.',
        'wall_limit_seconds_per_command': 1800,
        'address_space_limit_bytes_per_process': 8 * 1024**3,
        'limitations': 'External wall and per-process address-space containment; no internal total-work or heap-byte bound claimed. Focused qualification, not full CI, release validation, or performance comparison.',
        'previous_goal_turn': 'Progress: ab91bd456 and f82329124 committed exact 3D metaball fields, typed failures and passing 19-command qualification; the full properties goal remains active.',
    })
    rows = []
    for name, command in commands:
        assert sources() == initial and sha(Path(__file__)) == runner
        assert all(sha(ROOT / path) == digest for path, digest in preserved.items())
        print(json.dumps({'started': name, 'phase': phase}), flush=True)
        start = time.monotonic()
        with (out / (name + '.stdout')).open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            child = subprocess.Popen(command, cwd=ROOT, env=os.environ | env,
                                     stdout=stdout, stderr=stderr, start_new_session=True,
                                     preexec_fn=lambda: resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3)))
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
        assert all(sha(ROOT / path) == digest for path, digest in preserved.items())
        if code:
            raise SystemExit(code)


if __name__ == '__main__':
    main()
