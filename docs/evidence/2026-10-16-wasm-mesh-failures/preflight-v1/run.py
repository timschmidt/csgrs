import hashlib,json,os,resource,subprocess,time
from pathlib import Path
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
QUAL=ROOT/'target/properties/native-change-disposition/wasm-validation-v1/result.json'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
record=json.loads(QUAL.read_text())
assert record['complete'] and not record['unexpected_exits']
artifacts={p:r['sha256'] for p,r in record['artifacts'].items()}
assert all(sha(p)==h for p,h in artifacts.items())
protocol={'runner_sha256':sha(__file__),'probe_sha256':sha(OUT/'probe.cjs'),'contract_sha256':sha(OUT/'contract.json'),'qualification_sha256':sha(QUAL),'read_only_artifacts_sha256':artifacts}
assert not (OUT/'protocol.json').exists()
(OUT/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
js=next(p for p in artifacts if p.endswith('/csgrs.js'))
def limits():
 resource.setrlimit(resource.RLIMIT_AS,(64*1024**3,64*1024**3))
 resource.setrlimit(resource.RLIMIT_CORE,(0,0))
command=['node',str(OUT/'probe.cjs'),js]
start=time.monotonic()
with (OUT/'stdout').open('wb') as stdout,(OUT/'stderr').open('wb') as stderr:
 result=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/'resources'),'timeout','--signal=TERM','--kill-after=5s','60s',*command],cwd=OUT,stdout=stdout,stderr=stderr,preexec_fn=limits,start_new_session=True)
assert all(sha(p)==h for p,h in artifacts.items())
assert sha(__file__)==protocol['runner_sha256'] and sha(OUT/'probe.cjs')==protocol['probe_sha256']
rows=json.loads((OUT/'stdout').read_text()) if result.returncode==0 else []
summary={'exit_code':result.returncode,'command':command,'elapsed_seconds':time.monotonic()-start,'observations':len(rows),'string_exceptions':sum(r['outcome']=='threw' and r['type']=='string' for r in rows),'returned':sum(r['outcome']=='returned' for r in rows),'artifacts_unchanged':True,'admission_claimed':False,**{name+'_sha256':sha(OUT/name) for name in ['stdout','stderr','resources']}}
(OUT/'result.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary),flush=True)
raise SystemExit(result.returncode)
