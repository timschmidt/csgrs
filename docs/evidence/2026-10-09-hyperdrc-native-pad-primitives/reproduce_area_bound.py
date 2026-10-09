"""Replay the isolated original pad pair against saved pre-fix/current libraries.

Usage: python3 reproduce_area_bound.py COST_OUTPUT OUTPUT
The pre-fix binary is an explicitly retained local artifact, not rebuilt by this
script. Its source/hash, failing regression and full reports are retained here.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

record = Path(__file__).resolve().parent
cost, out = map(Path, sys.argv[1:3]); out.mkdir(parents=True,exist_ok=False)
load = lambda p: json.loads(p.read_bytes())
info = lambda p: dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
artifacts = [json.loads(line) for line in (cost/'library-build.jsonl').read_text().splitlines()]
def rlib(name):
    return next(Path(f) for a in artifacts if a.get('reason') == 'compiler-artifact' and a['target']['name'] == name for f in a['filenames'] if f.endswith('.rlib'))
before = load(record/'pre-area-fix/preserved-library.json')
libraries = dict(before=Path(before['path']),after=rlib('hyperdrc'))
assert info(libraries['before']) == {k:before[k] for k in ['bytes','sha256']}
dependencies = {name:rlib(name) for name in ['hyperlimit','serde_json']}
board = Path('/tmp/drc-native-rectangles-report-inputs/cparti-fpga/CPArti FPGA dev board.kicad_pcb')
source = record/'area_bound_probe.rs'
results = {}
for label, lib in libraries.items():
    binary = out/label
    command = ['rustc','--edition=2024','-O',str(source),'-L',f'dependency={Path.cwd()}/target/release/deps']
    for name,p in dict(hyperdrc=lib,**dependencies).items(): command += ['--extern',f'{name}={p}']
    command += ['-o',str(binary)]
    with (out/f'{label}-build.log').open('w') as log:
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    with (out/f'{label}.json').open('w') as log:
        subprocess.run([str(binary),str(board)],stdout=log,check=True)
    results[label] = dict(command=command,build_exit_code=0,exit_code=0,library=info(lib),output=info(out/f'{label}.json'))
old,new = [load(out/f'{label}.json') for label in libraries]
assert len(old['duplicate_findings']) == 1 and old['duplicate_findings'][0]['check'] == 'geometry-uncertainty'
assert new['duplicate_findings'] == []
assert old['overlap_area_error'] == new['overlap_area_error'] == 'region-area for KiCad pad could not be certified: area integral unavailable'
assert not old['overlap_native_view_available'] and not new['overlap_native_view_available']
assert {k:v for k,v in old.items() if k != 'duplicate_findings'} == {k:v for k,v in new.items() if k != 'duplicate_findings'}
(out/'manifest.json').write_text(json.dumps(dict(status='passed',source=info(source),board=info(board),results=results,
    scope='Original full CPArti board, source features 1723/1724. Independent native rounded area and exact bounding-box cap prove non-duplication. The underlying retained circular-fragment area integral remains unavailable.'),indent=2)+'\n')
print('Original pad pair: uncertainty before, certified non-duplicate after; native geometry unchanged.')
