"""Freeze affected sources and run curve workflow plus downstream gates sequentially.

Usage: python3 reproduce_validation.py OUTPUT_DIRECTORY
Pass --resume to continue a recorded partial run with identical source hashes.
Demo failures are recorded without suppressing the remaining validation phases.
Never overlap feature builds: the workspace emits an un-hashed csgrs rlib.
"""
import hashlib, json, os, subprocess, sys, time
from pathlib import Path

root = Path.cwd()
out = Path(sys.argv[1]).resolve()
resume = sys.argv[2:] == ['--resume']
out.mkdir(parents=True, exist_ok=resume)
crates = ['hypercurve', 'hyperdrc', 'hypercircuit']
def sources():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for crate in crates for p in sorted(Path('crates', crate).rglob('*.rs'))
            if 'target' not in p.parts}
initial = sources()
demo = root / 'crates/hypercurve/examples/hypercurve_ui'
commands = [
    ('fmt', root, ['cargo','fmt','--all','--','--check'], {}),
    ('curve-clippy', root, ['cargo','clippy','--locked','-p','hypercurve','--all-targets','--all-features','--','-D','warnings'], {}),
    ('curve-tests', root, ['cargo','test','--release','--locked','-p','hypercurve','--all-features','--no-fail-fast','--','--test-threads=2'], {}),
    ('curve-docs', root, ['cargo','doc','--locked','-p','hypercurve','--all-features','--no-deps'], {'RUSTDOCFLAGS':'-D warnings'}),
    ('demo-tests', demo, ['cargo','test','--locked','--no-fail-fast'], {}),
    ('demo-clippy', demo, ['cargo','clippy','--locked','--all-targets','--','-D','warnings'], {}),
    ('demo-wasm', demo, ['cargo','build','--locked','--release','--target','wasm32-unknown-unknown'], {}),
    ('consumer-clippy', root, ['cargo','clippy','--locked','-p','hypercircuit','--all-features','--all-targets','--','-D','warnings'], {}),
    ('consumer-drc', root, ['cargo','test','--locked','-p','hypercircuit','--all-features','--release','hyperdrc'], {}),
    ('consumer-curves', root, ['cargo','test','--locked','-p','hypercircuit','--all-features','--release','--test','curved_outline'], {}),
    ('drc-clippy', root, ['cargo','clippy','--locked','-p','hyperdrc','--all-features','--all-targets','--','-D','warnings'], {}),
    ('drc-tests', root, ['cargo','test','--locked','-p','hyperdrc','--release'], {}),
    ('drc-docs', root, ['cargo','doc','--locked','-p','hyperdrc','--all-features','--no-deps'], {'RUSTDOCFLAGS':'-D warnings'}),
    ('fixture-build', root, ['cargo','bench','--locked','-p','hyperdrc','--bench','fixture_smoke','--no-run'], {}),
]
status = json.loads((out/'status.json').read_text()) if resume else []
if resume:
    prior = json.loads((out/'sources.json').read_text())
    assert prior['unchanged'] and prior['source_hashes'] == initial
    assert [s['name'] for s in status] == [c[0] for c in commands[:len(status)]]
for name, cwd, command, env in commands[len(status):]:
    assert sources() == initial
    started = time.monotonic()
    with (out / (name+'.log')).open('w') as log:
        code = subprocess.run(command,cwd=cwd,stdout=log,stderr=subprocess.STDOUT,env={**os.environ,**env}).returncode
    status.append(dict(name=name,command=command,cwd=str(cwd),environment=env,exit_code=code,elapsed_seconds=time.monotonic()-started))
    (out/'status.json').write_text(json.dumps(status,indent=2)+'\n')
    (out/'sources.json').write_text(json.dumps(dict(source_hashes=initial,unchanged=sources()==initial),indent=2)+'\n')
    print(name,code,flush=True)
    assert sources() == initial
    if code and name != 'demo-tests': break
raise SystemExit(next((step['exit_code'] for step in status if step['exit_code']), 0))
