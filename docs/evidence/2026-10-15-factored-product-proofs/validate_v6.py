"""Preserve each factored-product-proofs acceptance trial and its complete result.

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
        '--', 'src', 'tests', 'crates', 'examples', 'benchmarks/rust', 'fuzz', 'bindings', 'Cargo.toml', 'Cargo.lock', 'readme.md', 'crates/hypercurve/README.md', 'crates/hypersdf/README.md', 'scripts/test-wasm-bindings.sh',
        '.cargo/config.toml', '.rustfmt.toml', 'AGENTS.md', 'docs/rounded_rectangle.png',
    ], cwd=ROOT).decode().split('\0')
    return {name: sha(ROOT / name) for name in sorted(set(names)) if name and (
        Path(name).suffix in {'.rs', '.c', '.cpp', '.h', '.hpp', '.py', '.js', '.cjs', '.ts', '.go', '.md', '.sh', '.png'}
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
    target_spec = E / (phase + '-target.json')
    target = ROOT / json.loads(target_spec.read_text()) if target_spec.exists() else Path('/tmp/csgrs-factored-product-proofs') / phase
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
        if False:  # No special implicit seed behavior; all new phases use explicit targets.
            artifact_names = ['release/libcsgrs_ffi.so', 'wasm32-unknown-unknown/wasm/csgrs.wasm',
                              'wasm-bindings-node/csgrs.js', 'wasm-bindings-node/csgrs.d.ts',
                              'wasm-bindings-node/csgrs_bg.wasm']
            artifact_names += [name + '-' + language for name in ['empty_composition', 'primitive_failures'] for language in ['c', 'cpp']]
            preserved.update({str((seed / name).relative_to(ROOT)): sha(seed / name) for name in artifact_names})
        assert all(sha(ROOT / path) == digest for path, digest in preserved.items())
        if seed_phase.get('release_only'):
            target.mkdir()
            copy_seed(['cp', '-a', '--no-preserve=links', '--reflink=auto', str(seed / 'release'), str(target / 'release')], out)
            preserved = {name: digest for name, digest in preserved.items() if '/release/' in name}
        else:
            copy_seed(['cp', '-a', '--no-preserve=links', '--reflink=auto', str(seed), str(target)], out)
        for path in preserved:
            old = ROOT / path
            copy = target / old.relative_to(seed)
            assert old.stat().st_ino != copy.stat().st_ino
    if seed:
        seed_root = seed / 'release' if seed_phase.get('release_only') else seed
        copy_root = target / 'release' if seed_phase.get('release_only') else target
        identities = set()
        count = 0
        for original in seed_root.rglob('*'):
            if original.is_file():
                copy = copy_root / original.relative_to(seed_root)
                before, after = original.stat(), copy.stat()
                identity = (after.st_dev, after.st_ino)
                assert (before.st_dev, before.st_ino) != identity and identity not in identities
                identities.add(identity)
                count += 1
        dump(out / 'seed-inode-independence.json', {'independent_files': count, 'shared_inodes': 0})
    external = E / (phase + "-preserve.json")
    if external.exists():
        preserved.update(json.loads(external.read_text()))
        assert all(sha(ROOT / name) == digest for name, digest in preserved.items())
    initial = sources()
    runner = sha(Path(__file__))
    with gzip.open(out / 'source-hashes.json.gz', 'wt') as f:
        json.dump(initial, f, sort_keys=True)
    (out / 'tracked.patch').write_bytes(subprocess.check_output([
        'git', 'diff', '--binary', 'HEAD', '--', 'src', 'tests', 'crates', 'examples', 'benchmarks/rust', 'fuzz', 'bindings', 'Cargo.toml',
        'Cargo.lock', '.cargo/config.toml', '.rustfmt.toml', 'AGENTS.md', 'docs/rounded_rectangle.png',
    ], cwd=ROOT))
    untracked = subprocess.check_output([
        'git', 'ls-files', '-z', '--others', '--exclude-standard', '--', 'src', 'tests', 'crates', 'examples', 'benchmarks/rust', 'fuzz', 'bindings',
    ], cwd=ROOT).decode().split('\0')
    with tarfile.open(out / 'untracked-sources.tar.gz', 'w:gz') as archive:
        for name in sorted(set(untracked) & initial.keys()):
            archive.add(ROOT / name, arcname=name)
    env = {
        'CARGO_HOME': '/home/tim/.cargo',
        'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
        'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
        'CARGO_BUILD_JOBS': '2', 'CARGO_TARGET_DIR': str(target), 'TMPDIR': str(target),
    }
    dump(out / 'protocol.json', {
        'seeded_build_artifacts': str(seed) if seed else None,
        'preserved_prior_executables_sha256': preserved,
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'runner_sha256': runner, 'phase': phase, 'commands': commands, 'environment': env,
        'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'properties': ['P2', 'P3', 'P4', 'P5', 'P7', 'P8', 'P9', 'P17', 'P18', 'P20', 'P21', 'P24', 'P25', 'P27', 'P35', 'P36', 'P37', 'P39'],
        'owning_layers': ['Hyperreal rational polynomial identities over opaque atoms', 'Hypermesh plane predicates and Boolean topology', 'CSGRS modeling composition and certainty', 'Hypersolve residual-report certainty'],
        'domain': 'After existing quotient expansion and factor reduction decline, a sum of at most 64 products can retain individually admitted polynomial factors, distributing only addition, negation, offsets and literal rational scale wrappers, match common factors and prove its residual zero. Every operand domain, total product degree and original depth/node/coefficient/work/cancellation constraint remains enforced within the same attempt. No claim is published for a nonzero residual.',
        'consumers': ['Real::exact_rational_normal_form', 'Hyperlimit shared sign resolvers', 'Hypermesh planes and Boolean arrangement', 'CSGRS strict drilled-body workflow', 'Hypersolve dense residual replay'],
        'missing_acceptance_test': ['candidate-v2 passes every original 8/16/24/32 full strict composition test and all independent initial-drill semantics/topology checks. Broad downstream qualification on the final source remains required.', 'Retain the new zero, signed displacement, inner nonzero residual, scaled nested sum, invalid domain, degree, work exhaustion and cancellation regressions.', 'qualification-v1 exposed an obsolete Hypersolve Unknown expectation. residual-policy-v1 proves exact acceptance on both committed 045621eba and E12. Keep the original system as exact success under both policies and add existing terminal_zero mixed-row coverage for strict/approximate/strict certainty; see residual-policy-contract-v1.md.', 'qualification-v2 exposed a second obsolete Unknown-count assertion. representation-policy-v1 confirms all 22 original sparse and pattern-preserving cases now certify on both 045621eba and E12. Require 22/22 strict successes and exact receipts; see representation-policy-contract-v1.md.'],
        'semantic_oracle': 'Independent quotient and distributive identities with nonzero-denominator obligations, exact nonzero displacements and typed unresolved-denominator controls. Downstream acceptance additionally needs independent point membership, polygonal volume and a separate topology check; program exit zero alone is not certified completion.',
        'wall_limit_seconds_per_command': 1800,
        'address_space_exceptions': {'typescript': 'Node/V8 virtual reservation; wall containment remains, no heap bound claimed.', 'wasm': 'Node/V8 and WebAssembly reserve large virtual address ranges. This command retains wall containment but has no RLIMIT_AS; no heap bound is claimed.'},
        'address_space_limit_bytes_per_process': 8 * 1024**3,
        'limitations': 'External wall and per-process address-space containment; no internal total-work or heap-byte bound claimed. Focused qualification, not full CI, release validation, or performance comparison. Historical /tmp artifact loss is recorded in ../2026-10-15-polynomial-quotient-reduction/artifact-audit-v1.json.gz; this phase guards its available pinned inputs but cannot claim complete historical artifact preservation.',
        'previous_goal_turn': 'Progress: 045621eba certifies full original 8/16/24 chains; the 32-segment subtraction reaches a new 64-term failure. The captured degree-eight identity expands to 145 terms and contains common factors. All original limits, tests and fixtures remain.',
    })
    rows = []
    for name, command in commands:
        assert sources() == initial and sha(Path(__file__)) == runner
        assert all(sha(ROOT / path) == digest for path, digest in preserved.items())
        print(json.dumps({'started': name, 'phase': phase}), flush=True)
        start = time.monotonic()
        with (out / (name + '.stdout')).open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            cap = None if name in {'wasm', 'typescript'} else lambda: resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
            child = subprocess.Popen(['/usr/bin/time', '-v', '-o', str(out / (name + '.resources'))] + command, cwd=ROOT, env=os.environ | env,
                                     stdout=stdout, stderr=stderr, start_new_session=True,
                                     preexec_fn=cap)
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
            'resources_sha256': sha(out / (name + '.resources')),
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
