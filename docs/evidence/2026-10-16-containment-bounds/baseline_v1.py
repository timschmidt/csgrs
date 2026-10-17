"""Verify current source identity and rerun missing controls before changing it."""
import gzip,hashlib,json,resource,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];OLD=W.parent/'containment-winding';E=ROOT/'docs/evidence/2026-10-16-containment-winding';O=W/'baseline-v1'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
BASE='311d04fbbe5635a29fe7e754b838cccc4b8add53'
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True)
with gzip.open(E/'final-input-hashes-v1.json.gz','rt') as f:current=json.load(f)
current['docs/properties-status.json']=read(E/'checkpoint-v1.json')['ledger_after_sha256']
assert all(sha(ROOT/p)==h for p,h in current.items())
for folder in ['candidate-validation-v3','wasm-validation-v3','ffi-validation-v3']:
 r=read(OLD/folder/'result.json');assert r['complete'] and not r['unexpected_exits']
rows=[json.loads(line) for line in (OLD/'candidate-validation-v3/cost-libraries.stdout').read_text().splitlines() if line.startswith('{')]
libs={name:next(Path(p) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']==name for p in row['filenames'] if p.endswith('.rlib')) for name in ['csgrs','hypermesh']}
deps=libs['hypermesh'].parent;pins={str(p):sha(p) for p in deps.glob('*.rlib')}
wasm=read(OLD/'wasm-validation-v3/result.json')['artifacts'];pins.update({p:r['sha256'] for p,r in wasm.items()})
inputs={str(p):sha(p) for p in (W/'prototype-v1').rglob('*') if p.is_file()}
inputs.update({str(p):sha(p) for p in [Path(__file__),W/'control_cost_v1.rs',W/'matrix_cost_v1.rs',W/'cost-contract-v1.json']})
O.mkdir(exist_ok=False)
with gzip.open(O/'current-input-hashes.json.gz','wt') as f:json.dump(current,f,sort_keys=True)
dump(O/'protocol.json',{'base_commit':BASE,'current_inputs_sha256':sha(O/'current-input-hashes.json.gz'),'runner_probes_contract_sha256':inputs,'read_only_artifacts_sha256':pins,'native_libraries':{k:str(p) for k,p in libs.items()},'source_difference':'Only admission current_work metadata differs from E32 qualified sources. Every current runtime input matches.','completed_targets_reopened':False,'rustc':subprocess.check_output(['rustc','-vV'],text=True),'external_seconds_per_command':180,'native_address_space_bytes':8*1024**3,'wasm_address_space_bytes':64*1024**3})
commands=[]
def run(name,command,wasm=False):
 def limits():
  n=(64 if wasm else 8)*1024**3;resource.setrlimit(resource.RLIMIT_AS,(n,n));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
 start=time.monotonic()
 with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
  p=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=10s','180s',*command],cwd=O,stdout=out,stderr=err,preexec_fn=limits,start_new_session=True)
 row={'name':name,'command':command,'exit_code':p.returncode,'elapsed_seconds':time.monotonic()-start,**{x+'_sha256':sha(O/(name+'.'+x)) for x in ['stdout','stderr','resources']}}
 commands.append(row);dump(O/'commands.json',commands);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
 return p.returncode
binary=O/'native-bounds-tests'
assert run('native-compile',['rustc','--edition=2024','-D','warnings','--test','-C','opt-level=3','-L','dependency='+str(deps),'--extern','csgrs='+str(libs['csgrs']),'--extern','hypermesh='+str(libs['hypermesh']),str(W/'prototype-v1/tests/containment_bounds.rs'),'-o',str(binary)])==0
native=run('native-tests',[str(binary),'--nocapture','--test-threads=1'])
js=next(p for p in wasm if p.endswith('/csgrs.js'))
wasm_exit=run('wasm-tests',['node',str(W/'prototype-v1/tests/wasm/containment_winding.cjs'),js],wasm=True)
for scope in ['control','matrix']:
 binary=O/(scope+'-probe')
 assert run(scope+'-compile',['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+str(deps),'--extern','hypermesh='+str(libs['hypermesh']),str(W/(scope+'_cost_v1.rs')),'-o',str(binary)])==0
 assert run(scope+'-cost',[str(binary)])==0
assert all(sha(p)==h for p,h in pins.items()) and all(sha(p)==h for p,h in inputs.items())
assert all(sha(ROOT/p)==h for p,h in current.items())
dump(O/'result.json',{'complete':True,'native_test_exit_code':native,'wasm_test_exit_code':wasm_exit,'current_inputs_unchanged':True,'read_only_artifacts_unchanged':True,'control_and_matrix_observations':'One fresh process each before implementation; paired cost evidence follows only after candidate qualification.','candidate_admitted':False})
assert native==0 and wasm_exit==0
