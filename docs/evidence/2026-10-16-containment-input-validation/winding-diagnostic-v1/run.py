"""Read-only current binding diagnostic; it does not admit a winding fix."""
import hashlib,json,resource,subprocess,time
from pathlib import Path
OUT=Path(__file__).resolve().parent
WORK=OUT.parent
ROOT=WORK.parents[2]
QUAL=WORK.parent/'wasm-mesh-failure-review/wasm-validation-v1/result.json'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
record=json.loads(QUAL.read_text());assert record['complete'] and not record['unexpected_exits']
artifacts={p:r['sha256'] for p,r in record['artifacts'].items()}
assert all(sha(p)==h for p,h in artifacts.items())
protocol={'runner_sha256':sha(__file__),'probe_sha256':sha(OUT/'probe.cjs'),'qualification_sha256':sha(QUAL),'artifacts_read_only_sha256':artifacts,'scope':'Compare independently derived nonzero winding membership with current documented parity; preserve the mismatch as a separate open semantic issue. No geometry normalization or policy change.'}
assert not (OUT/'protocol.json').exists();dump(OUT/'protocol.json',protocol)
def limits():
 resource.setrlimit(resource.RLIMIT_AS,(64*1024**3,64*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
js=next(p for p in artifacts if p.endswith('/csgrs.js'));command=['node',str(OUT/'probe.cjs'),js]
start=time.monotonic()
with (OUT/'stdout').open('wb') as out,(OUT/'stderr').open('wb') as err:
 p=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/'resources'),'timeout','--signal=TERM','--kill-after=5s','60s',*command],stdout=out,stderr=err,preexec_fn=limits,start_new_session=True)
assert all(sha(p)==h for p,h in artifacts.items())
rows=json.loads((OUT/'stdout').read_text()) if p.returncode==0 else []
dump(OUT/'result.json',{'exit_code':p.returncode,'command':command,'elapsed_seconds':time.monotonic()-start,'observations':len(rows),'nonzero_winding_mismatches':sum(r['observed']!=r['expected_nonzero_winding_membership'] for r in rows),'artifacts_unchanged':True,**{n+'_sha256':sha(OUT/n) for n in ['stdout','stderr','resources']}})
print((OUT/'result.json').read_text(),flush=True)
raise SystemExit(p.returncode)
