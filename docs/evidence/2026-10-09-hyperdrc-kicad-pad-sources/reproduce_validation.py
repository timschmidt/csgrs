"""Freeze affected sources and run scoped validation serially; never overlap Cargo feature builds.

Usage: python3 reproduce_validation.py NEW_OUTPUT_DIRECTORY [--resume]
Resume requires an all-passed command prefix and identical production sources.
After a reviewed snapshot-only correction, preserve the failed run separately,
copy only its successful prefix, and rerun the affected consumer tests.
Unchanged curve/demo validation is accounted for separately, with source hashes.
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
    paths = [p for name in ['hypercurve', 'hyperdrc', 'hypercircuit']
             for p in Path('crates', name).rglob('*')
             if p.is_file() and 'target' not in p.parts
             and (p.suffix in ['.rs', '.snap'] or p.name in ['Cargo.toml', 'Cargo.lock'])]
    paths += [Path('Cargo.toml'), Path('Cargo.lock')]
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}
initial = sources()
commands = [
    ('fmt', ['cargo', 'fmt', '--all', '--', '--check'], {}),
    ('drc-clippy', ['cargo', 'clippy', '--locked', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'], {}),
    ('drc-tests', ['cargo', 'test', '--locked', '-p', 'hyperdrc', '--release', '--no-fail-fast'], {}),
    ('drc-docs', ['cargo', 'doc', '--locked', '-p', 'hyperdrc', '--all-features', '--no-deps'], {'RUSTDOCFLAGS': '-D warnings'}),
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
