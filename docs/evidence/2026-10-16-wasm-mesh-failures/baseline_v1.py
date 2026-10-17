"""Run the proposed binding regressions against preserved committed-source WASM artifacts."""
import hashlib,json,resource,shutil,subprocess,time
from pathlib import Path
WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[2]
OUT=WORK/'baseline-v1'
PROTO=WORK/'prototype-v1'
QUAL=ROOT/'target/properties/native-change-disposition/wasm-validation-v1/result.json'
def sha(path):
 with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def read(path):return json.loads(Path(path).read_text())
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')
r=read(QUAL)
assert r['complete'] and not r['unexpected_exits']
artifacts={path:row['sha256'] for path,row in r['artifacts'].items()}
assert all(sha(path)==digest for path,digest in artifacts.items())
tools=read(QUAL.parent/'protocol.json')['tools_read_only_sha256']
tsc=next(path for path in tools if path.endswith('/typescript/lib/tsc.js'))
assert sha(tsc)==tools[tsc]
OUT.mkdir(exist_ok=False)
consumer=OUT/'consumer';consumer.mkdir()
for path in artifacts:
 source=Path(path)
 if source.name in ['csgrs.js','csgrs.d.ts','csgrs_bg.wasm','csgrs_bg.wasm.d.ts']:
  shutil.copy2(source,consumer/source.name)
shutil.copy2(PROTO/'tests/wasm/mesh_errors.ts',consumer/'mesh_errors.ts')
pins={str(path):sha(path) for path in PROTO.rglob('*') if path.is_file()}
protocol={'runner_sha256':sha(__file__),'prototype_sha256':pins,'read_only_artifacts_sha256':artifacts,
 'qualification_sha256':sha(QUAL),'tsc':tsc,'tsc_sha256':tools[tsc],
 'copied_consumer_inputs_sha256':{str(path):sha(path) for path in consumer.iterdir()},
 'timeout_seconds':60,'per_process_address_space_bytes':64*1024**3,
 'purpose':'Failing acceptance tests on exact committed libraries before implementation; preserve failure output.'}
dump(OUT/'protocol.json',protocol)
rows=[]
def limits():
 resource.setrlimit(resource.RLIMIT_AS,(64*1024**3,64*1024**3))
 resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
 start=time.monotonic()
 with (OUT/(name+'.stdout')).open('wb') as stdout,(OUT/(name+'.stderr')).open('wb') as stderr:
  p=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/(name+'.resources')),'timeout','--signal=TERM','--kill-after=5s','60s',*command],cwd=OUT,stdout=stdout,stderr=stderr,preexec_fn=limits,start_new_session=True)
 row={'name':name,'command':command,'exit_code':p.returncode,'elapsed_seconds':time.monotonic()-start,**{suffix+'_sha256':sha(OUT/(name+'.'+suffix)) for suffix in ['stdout','stderr','resources']}}
 rows.append(row);dump(OUT/'commands.json',rows);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
 return p.returncode
js=run('real-js',['node',str(PROTO/'tests/wasm/mesh_errors.cjs'),str(consumer/'csgrs.js')])
ts=run('compile-typescript',['node',tsc,'--strict','--target','ES2020','--lib','ES2020,DOM,ESNext.Disposable','--module','Node16','--moduleResolution','Node16','--outDir',str(consumer),str(consumer/'mesh_errors.ts')])
executed=run('real-typescript',['node',str(consumer/'mesh_errors.js')]) if ts==0 else None
assert all(sha(path)==digest for path,digest in artifacts.items())
assert all(sha(path)==digest for path,digest in pins.items())
assert sha(tsc)==tools[tsc] and sha(__file__)==protocol['runner_sha256']
result={'complete':ts==0 and executed is not None,'javascript_exit_code':js,'typescript_compile_exit_code':ts,'typescript_consumer_exit_code':executed,'prototype_and_baseline_artifacts_unchanged':True,'candidate_admitted':False,'native_disposition_changed':False}
dump(OUT/'result.json',result);print(json.dumps(result),flush=True)
raise SystemExit(0 if result['complete'] else 101)
