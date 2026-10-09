"""Run the pad regressions and existing geometry/parser tests on frozen sources.

Run Cargo phases sequentially: the workspace emits an un-hashed csgrs rlib.
Usage: python3 reproduce_focused.py OUTPUT
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

out = Path(sys.argv[1]); out.mkdir(parents=True,exist_ok=False)
sources = lambda: {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('crates/hyperdrc').rglob('*.rs') if 'target' not in p.parts}
before = sources()
commands = [('fmt',['cargo','fmt','--all','--','--check']),
            ('clippy',['cargo','clippy','--locked','-p','hyperdrc','--all-features','--all-targets','--','-D','warnings'])]
for label, selector in [('pads','geometry::pad_regressions'),('geometry','geometry::tests'),('rectangles','geometry::rectangle_regressions'),('kicad','kicad::'),('duplicate-layer','checks::layer::tests::duplicate_'),('bounds','bounds_regressions')]:
    commands.append((label,['cargo','test','--locked','-p','hyperdrc','--release',selector,'--','--test-threads=1']))
status = []
for name, command in commands:
    started = time.monotonic()
    with (out / f'{name}.log').open('w') as log:
        result = subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,env=os.environ)
    status.append(dict(name=name,command=command,exit_code=result.returncode,elapsed_seconds=time.monotonic()-started))
    (out / 'status.json').write_text(json.dumps(status,indent=2)+'\n')
    print(name,result.returncode,flush=True)
    if result.returncode: break
(out / 'sources.json').write_text(json.dumps(dict(source_hashes=before,unchanged=before == sources()),indent=2)+'\n')
assert before == sources()
raise SystemExit(status[-1]['exit_code'])
