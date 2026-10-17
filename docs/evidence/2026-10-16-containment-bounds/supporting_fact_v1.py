"""Observe new proof sufficiency without reopening either WASM build target."""
import hashlib,json,resource,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'supporting-fact-v1'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
sources={'baseline':W.parent/'containment-winding/wasm-validation-v3/result.json',
    'candidate':W/'wasm-validation-v1/result.json'}
pins={str(p):sha(p) for p in [Path(__file__),W/'supporting_fact_v1.cjs',*sources.values()]}
for p in sources.values():
    result=read(p);assert result['complete']
    pins.update({p:r['sha256'] for p,r in result['artifacts'].items()})
assert all(sha(p)==h for p,h in pins.items())
O.mkdir(exist_ok=False)
dump(O/'protocol.json',{'read_only_inputs_sha256':pins,'new_proof_scope':'The represented inverse-rotated source, not fidelity to the original unit cube.',
    'completed_targets_reopened':False,'elapsed_limit_seconds':180,'address_space_limit_bytes':64*1024**3})
rows=[]
def limits():
    resource.setrlimit(resource.RLIMIT_AS,(64*1024**3,64*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
for variant,p in sources.items():
    js=next(p for p in read(p)['artifacts'] if p.endswith('/csgrs.js'))
    command=['node',str(W/'supporting_fact_v1.cjs'),js]
    start=time.monotonic()
    with (O/(variant+'.stdout')).open('wb') as out,(O/(variant+'.stderr')).open('wb') as err:
        result=subprocess.run(['/usr/bin/time','-v','-o',str(O/(variant+'.resources')),'timeout','--kill-after=10s','180s',*command],
            stdout=out,stderr=err,preexec_fn=limits,start_new_session=True)
    rows.append({'variant':variant,'command':command,'exit_code':result.returncode,'elapsed_seconds':time.monotonic()-start,
        **{s+'_sha256':sha(O/(variant+'.'+s)) for s in ['stdout','stderr','resources']}})
    dump(O/'commands.json',rows)
    assert result.returncode==0
assert all(sha(p)==h for p,h in pins.items())
dump(O/'result.json',{'complete':True,'commands':rows,'read_only_inputs_unchanged':True,
    'observations':{v:read(O/(v+'.stdout')) for v in sources}})
print(json.dumps(read(O/'result.json')['observations'],indent=2),flush=True)
