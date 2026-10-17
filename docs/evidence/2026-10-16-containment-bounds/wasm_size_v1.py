"""Compare final WASM artifacts and declarations with the qualified baseline."""
import hashlib,json
from pathlib import Path
W=Path(__file__).resolve().parent
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
out=W/'wasm-size-v1.json';assert not out.exists()
paths={'baseline':W.parent/'containment-winding/wasm-validation-v3/result.json',
    'candidate':W/'wasm-validation-v2/result.json'}
variants={}
for name,path in paths.items():
    result=read(path);assert result['complete'] and not result['unexpected_exits']
    variants[name]={Path(p).name:{'path':p,**r} for p,r in result['artifacts'].items()}
    assert all(sha(r['path'])==r['sha256'] and Path(r['path']).stat().st_size==r['bytes'] for r in variants[name].values())
rows=[]
for name in ['csgrs.wasm','csgrs_bg.wasm','csgrs.js','csgrs.d.ts','csgrs_bg.wasm.d.ts']:
    a,b=variants['baseline'][name],variants['candidate'][name]
    rows.append({'artifact':name,'baseline':a,'candidate':b,'byte_delta':b['bytes']-a['bytes'],'identical':a['sha256']==b['sha256']})
assert all(r['identical'] for r in rows if r['artifact'].endswith(('.js','.ts')))
record={'complete':True,'runner_sha256':sha(__file__),'qualified_results_sha256':{str(p):sha(p) for p in paths.values()},'rows':rows,
    'claim':'Artifact byte sizes and exact declaration/glue identity, not serialized geometry size or a complete distribution measurement.'}
out.write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({r['artifact']:r['byte_delta'] for r in rows}),flush=True)
