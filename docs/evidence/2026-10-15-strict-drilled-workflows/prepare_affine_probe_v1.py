"""Prepare the next frontier observation against the qualified scalar build."""
from pathlib import Path
import json,hashlib
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
phase='affine-baseline-v1';target=Path('/tmp/csgrs-strict-drilled-workflows')/phase
assert json.loads((E/'cost-v2/result.json').read_text())['passed']
commands=json.loads((E/'cost-v2.json').read_text())
command=next(command for name,command in commands if name=='candidate-probe').copy()
command=[str(E/'affine_plane_probe_v1.rs') if arg==str(E/'scalar_cost_v1.rs') else arg for arg in command]
command[command.index('-o')+1]=str(target/'probe')
commands=[['prepare-target',['mkdir',str(target)]],['probe-build',command],['frontier',[str(target/'probe')]]]
keep=json.loads((E/'cost-v2-preserve.json').read_text())
for p in [*Path('/tmp/csgrs-strict-drilled-workflows/cost-v2').iterdir(),E/'affine_plane_probe_v1.rs',Path(__file__)]:
    if p.is_file():keep[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
for suffix,value in [('.json',commands),('-seed.json',None),('-preserve.json',keep)]:
    p=E/(phase+suffix);assert not p.exists();p.write_text(json.dumps(value,indent=2)+'\n')
