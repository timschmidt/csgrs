"""Replay the original complete board and unchanged copper indices 1723/1724.
Usage: python3 reproduce_original_pair.py COST_OUTPUT OUTPUT
"""
import hashlib,json,subprocess,sys
from pathlib import Path
record=Path(__file__).resolve().parent
cost,out=map(Path,sys.argv[1:3]);out.mkdir(parents=True,exist_ok=False)
info=lambda p:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
artifacts=[json.loads(line) for line in (cost/'library-build.jsonl').read_text().splitlines()]
def rlib(name):
    return next(Path(f) for a in artifacts if a.get('reason')=='compiler-artifact' and a['target']['name']==name for f in a['filenames'] if f.endswith('.rlib'))
source=record/'original_pair_probe.rs';binary=out/'original_pair'
command=['rustc','--edition=2024','-O',str(source),'-L',f'dependency={Path.cwd()}/target/release/deps']
linked={name:rlib(name) for name in ['hyperdrc','hyperlimit','serde_json']}
for name,path in linked.items():command+=['--extern',f'{name}={path}']
command+=['-o',str(binary)]
with (out/'build.log').open('w') as log: subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
board=Path('/tmp/drc-native-rectangles-report-inputs/cparti-fpga/CPArti FPGA dev board.kicad_pcb')
with (out/'after.json').open('w') as log: subprocess.run([str(binary),str(board)],stdout=log,check=True)
before=json.loads((record/'baseline/area-bound/after.json').read_bytes())
after=json.loads((out/'after.json').read_bytes())
assert before['overlap_area_error'] and not before['overlap_native_view_available']
assert after['overlap_area_error'] is None and after['overlap_native_view_available']
assert before['duplicate_findings']==after['duplicate_findings']==[]
allowed={'overlap_area_error','overlap_native_view_available','overlap_area'}
assert {k:v for k,v in before.items() if k not in allowed}=={k:v for k,v in after.items() if k not in allowed}
(out/'manifest.json').write_text(json.dumps(dict(status='passed',command=command,source=info(source),board=info(board),binary=info(binary),linked_libraries={str(p):info(p) for p in linked.values()},output=info(out/'after.json'),baseline=info(record/'baseline/area-bound/after.json'),scope='Original full CPArti board and indices. Exact area and native view become available; all other retained pair fields, including certified non-duplicate findings, are unchanged.'),indent=2)+'\n')
print('Original pair: exact integral and native contour available; independent area bounds certified.',flush=True)
