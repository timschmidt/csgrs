"""Run the new invalid-input assertions against the currently qualified binding."""
import hashlib,json,resource,subprocess
from pathlib import Path
WORK=Path(__file__).resolve().parent
OUT=WORK/'wasm-reproduction-v1'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
prior=json.loads((WORK.parent/'wasm-mesh-failure-review/wasm-validation-v1/result.json').read_text())
assert prior['complete'] and not prior['unexpected_exits']
artifacts=prior['artifacts'];assert all(sha(p)==r['sha256'] for p,r in artifacts.items())
probe=WORK/'baseline-source-v1/tests/wasm/mesh_errors.cjs'
OUT.mkdir(exist_ok=False)
dump(OUT/'protocol.json',{'runner_sha256':sha(__file__),'test_sha256':sha(probe),'artifacts':artifacts,'virtual_address_space_bytes':64*1024**3,'external_limit_seconds':60})
def limits():
 resource.setrlimit(resource.RLIMIT_AS,(64*1024**3,64*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
js=next(p for p in artifacts if p.endswith('/csgrs.js'))
with (OUT/'stdout').open('wb') as out,(OUT/'stderr').open('wb') as err:
 p=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/'resources'),'timeout','--kill-after=5s','60s','node',str(probe),js],stdout=out,stderr=err,preexec_fn=limits,start_new_session=True)
text=(OUT/'stdout').read_text()
assert all(sha(p)==r['sha256'] for p,r in artifacts.items())
matched=p.returncode==1 and all(s in text for s in ['# tests 13','# pass 10','# fail 3','failed operation returned a successful value'])
dump(OUT/'result.json',{'exit_code':p.returncode,'expected_failure_reproduced':matched,'passed':10 if matched else None,'failed':3 if matched else None,**{s+'_sha256':sha(OUT/s) for s in ['stdout','stderr','resources']}})
print((OUT/'result.json').read_text(),flush=True)
assert matched
