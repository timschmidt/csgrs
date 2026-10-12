"""Freeze affected sources and run scoped validation serially; never overlap Cargo feature builds.

Usage: python3 reproduce_validation.py NEW_OUTPUT_DIRECTORY [--resume]
Resume requires an all-passed command prefix and identical production sources.
After a reviewed snapshot-only correction, preserve the failed run separately,
copy only its successful prefix, and rerun the affected consumer tests.
Includes scalar, predicate, solver and lattice gates because interval cache
access changed, plus the native curve kernel, demo and PCB consumers.
A failing stage stops this run without discarding its status or log.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

root = Path.cwd()
out = Path(sys.argv[1]).resolve()
resume = sys.argv[2:] == ['--resume']
out.mkdir(parents=True, exist_ok=resume)
def sources():
    paths = [p for name in ['hyperreal', 'hyperlattice', 'hyperlimit', 'hypersolve', 'hypercurve', 'hyperdrc', 'hypercircuit']
             for p in Path('crates', name).rglob('*')
             if p.is_file() and 'target' not in p.parts
             and (p.suffix in ['.rs', '.snap'] or p.name in ['Cargo.toml', 'Cargo.lock'])]
    paths += [Path('Cargo.toml'), Path('Cargo.lock'), Path('.cargo/config.toml'), Path('.rustfmt.toml')]
    paths += [p for folder in ['crates/hyperreal/scripts', 'crates/hyperreal/verification']
              for p in Path(folder).rglob('*') if p.is_file() and 'target' not in p.parts
              and p.suffix in ['.py', '.json']]
    paths += [Path('.github/workflows/hyperreal.yml')]
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}
initial = sources()
commands = [
    ('fmt', ['cargo', 'fmt', '--all', '--', '--check'], {}),
    ('real-check', ['cargo', 'check', '--locked', '-p', 'hyperreal', '--all-targets'], {}),
    ('real-default-tests', ['cargo', 'test', '--locked', '-p', 'hyperreal', '--all-targets'], {}),
    ('real-default-clippy', ['cargo', 'clippy', '--locked', '-p', 'hyperreal', '--all-targets', '--', '-D', 'warnings'], {}),
    ('real-all-tests', ['cargo', 'test', '--locked', '-p', 'hyperreal', '--release', '--all-features', '--no-fail-fast'], {}),
    ('real-all-clippy', ['cargo', 'clippy', '--locked', '-p', 'hyperreal', '--all-features', '--all-targets', '--', '-D', 'warnings'], {}),
    ('real-docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '--all-features', '--no-deps'], {'RUSTDOCFLAGS': '-D warnings'}),
    ('real-fuzz', ['cargo', 'check', '--locked', '--manifest-path', 'crates/hyperreal/fuzz/Cargo.toml', '--bins'], {}),
    ('real-verus-install', ['python3', 'crates/hyperreal/scripts/verify_verus.py', 'install'], {}),
    ('real-verus-proofs', ['python3', 'crates/hyperreal/scripts/verify_verus.py', 'verify'], {}),
    ('real-verus-rejections', ['python3', 'crates/hyperreal/verification/test_rejections.py'], {}),
    ('real-verus-acceptance', ['python3', 'crates/hyperreal/verification/test_acceptance.py'], {}),
    ('limit-clippy', ['cargo', 'clippy', '--locked', '-p', 'hyperlimit', '--all-features', '--all-targets', '--', '-D', 'warnings'], {}),
    ('limit-tests', ['cargo', 'test', '--locked', '-p', 'hyperlimit', '--release', '--all-features', '--no-fail-fast'], {}),
    ('solve-clippy', ['cargo', 'clippy', '--locked', '-p', 'hypersolve', '--all-features', '--all-targets', '--', '-D', 'warnings'], {}),
    ('solve-tests', ['cargo', 'test', '--locked', '-p', 'hypersolve', '--release', '--all-features', '--no-fail-fast'], {}),
    ('lattice-clippy', ['cargo', 'clippy', '--locked', '-p', 'hyperlattice', '--all-features', '--all-targets', '--', '-D', 'warnings'], {}),
    ('lattice-tests', ['cargo', 'test', '--locked', '-p', 'hyperlattice', '--release', '--all-features', '--no-fail-fast'], {}),
    ('drc-clippy', ['cargo', 'clippy', '--locked', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'], {}),
    ('drc-tests', ['cargo', 'test', '--locked', '-p', 'hyperdrc', '--release', '--no-fail-fast'], {}),
    ('drc-docs', ['cargo', 'doc', '--locked', '-p', 'hyperdrc', '--all-features', '--no-deps'], {'RUSTDOCFLAGS': '-D warnings'}),
    ('curve-clippy', ['cargo', 'clippy', '--locked', '-p', 'hypercurve', '--all-features', '--all-targets', '--', '-D', 'warnings'], {}),
    ('curve-tests', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--all-features', '--no-fail-fast', '--', '--test-threads=2'], {}),
    ('curve-docs', ['cargo', 'doc', '--locked', '-p', 'hypercurve', '--all-features', '--no-deps'], {'RUSTDOCFLAGS': '-D warnings'}),
    ('demo-tests', ['cargo', 'test', '--locked', '--manifest-path', 'crates/hypercurve/examples/hypercurve_ui/Cargo.toml', '--no-fail-fast'], {}),
    ('demo-clippy', ['cargo', 'clippy', '--locked', '--manifest-path', 'crates/hypercurve/examples/hypercurve_ui/Cargo.toml', '--all-targets', '--', '-D', 'warnings'], {}),
    ('demo-wasm', ['cargo', 'build', '--locked', '--manifest-path', 'crates/hypercurve/examples/hypercurve_ui/Cargo.toml', '--release', '--target', 'wasm32-unknown-unknown'], {}),
    ('consumer-clippy', ['cargo', 'clippy', '--locked', '-p', 'hypercircuit', '--all-features', '--all-targets', '--', '-D', 'warnings'], {}),
    ('consumer-drc', ['cargo', 'test', '--locked', '-p', 'hypercircuit', '--all-features', '--release', 'hyperdrc'], {}),
    ('consumer-curves', ['cargo', 'test', '--locked', '-p', 'hypercircuit', '--all-features', '--release', '--test', 'curved_outline'], {}),
    ('fixture-build', ['cargo', 'bench', '--locked', '-p', 'hyperdrc', '--bench', 'fixture_smoke', '--no-run'], {}),
]
status = json.loads((out / 'status.json').read_text()) if resume else []
if resume:
    previous = json.loads((out / 'sources.json').read_text())
    assert previous['unchanged'] and previous['source_hashes'] == initial
    assert all(step['exit_code'] == 0 for step in status)
    assert [step['name'] for step in status] == [step[0] for step in commands[:len(status)]]
fixtures = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(Path('crates/hypercircuit/tests/fixtures/easyduino/fast-drc').glob('*.json'))}
for name, command, env in commands[len(status):]:
    assert sources() == initial
    started = time.monotonic()
    with (out / (name + '.log')).open('w') as log:
        code = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, env={**os.environ, **env}).returncode
    status.append(dict(name=name, command=command, environment=env, exit_code=code, elapsed_seconds=time.monotonic() - started))
    (out / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    (out / 'sources.json').write_text(json.dumps(dict(source_hashes=initial, unchanged=sources() == initial), indent=2) + '\n')
    current_fixtures = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in fixtures}
    (out / 'fixtures.json').write_text(json.dumps(dict(source_hashes=fixtures, unchanged=current_fixtures == fixtures), indent=2) + '\n')
    assert current_fixtures == fixtures
    print(name, code, flush=True)
    assert sources() == initial
    if code:
        break
raise SystemExit(status[-1]['exit_code'])
