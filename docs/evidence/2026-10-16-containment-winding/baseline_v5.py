"""Reproduce set-meaning failures on exact current native and WASM artifacts."""
import gzip,hashlib,json,resource,subprocess,time
from pathlib import Path
WORK=Path(__file__).resolve().parent;ROOT=WORK.parents[2]
OLD=WORK.parent/'containment-input-validation'
PARENT=ROOT/'docs/evidence/2026-10-16-containment-input-validation'
OUT=WORK/'baseline-v5'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
BASE='d4eee9a675fa51d84eb0eb29503dd802bdcdbd19'
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True)
with gzip.open(PARENT/'final-input-hashes-v1.json.gz','rt') as f:current=json.load(f)
current['docs/properties-status.json']=read(PARENT/'checkpoint-v1.json')['ledger_after_sha256']
assert all(sha(ROOT/p)==h for p,h in current.items())
for folder in ['candidate-validation-v2','wasm-validation-v2']:
 r=read(OLD/folder/'result.json');assert r['complete'] and not r['unexpected_exits']
artifacts=[json.loads(line) for line in (OLD/'candidate-validation-v2/cost-libraries.stdout').read_text().splitlines() if line.startswith('{')]
libs={name:next(Path(p) for row in artifacts if row.get('reason')=='compiler-artifact' and row['target']['name']==name for p in row['filenames'] if p.endswith('.rlib')) for name in ['csgrs','hypermesh']}
deps=libs['hypermesh'].parent
pins={str(p):sha(p) for p in deps.glob('*.rlib')}
wasm=read(OLD/'wasm-validation-v2/result.json')['artifacts']
pins.update({p:r['sha256'] for p,r in wasm.items()})
probes={str(p):sha(p) for p in (WORK/'prototype-v3').rglob('*') if p.is_file()};probes[str(Path(__file__))]=sha(__file__)
OUT.mkdir(exist_ok=False)
with gzip.open(OUT/'current-input-hashes.json.gz','wt') as f:json.dump(current,f,sort_keys=True)
dump(OUT/'protocol.json',{'base_commit':BASE,'runner_and_probes_sha256':probes,'current_inputs_sha256':sha(OUT/'current-input-hashes.json.gz'),'read_only_artifacts_sha256':pins,'qualified_source_manifest_sha256':sha(PARENT/'final-input-hashes-v1.json.gz'),'native_libraries':{k:str(p) for k,p in libs.items()},'rustc':subprocess.check_output(['rustc','-vV'],text=True),'source_difference':'Only admission current_work metadata differs from the prior qualified snapshot; all current runtime inputs match.','completed_targets_reopened':False,'external_seconds_per_command':180,'native_address_space_bytes':8*1024**3,'wasm_address_space_bytes':64*1024**3})
commands=[]
def run(name,command,wasm=False):
 def limits():
  n=(64 if wasm else 8)*1024**3;resource.setrlimit(resource.RLIMIT_AS,(n,n));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
 start=time.monotonic()
 with (OUT/(name+'.stdout')).open('wb') as out,(OUT/(name+'.stderr')).open('wb') as err:
  p=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/(name+'.resources')),'timeout','--kill-after=10s','180s',*command],cwd=OUT,stdout=out,stderr=err,preexec_fn=limits,start_new_session=True)
 row={'name':name,'command':command,'exit_code':p.returncode,'elapsed_seconds':time.monotonic()-start,**{s+'_sha256':sha(OUT/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
 commands.append(row);dump(OUT/'commands.json',commands);print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
 return p.returncode
binary=OUT/'native-winding-tests'
assert run('native-compile',['rustc','--edition=2024','--test','-C','opt-level=3','-L','dependency='+str(deps),'--extern','csgrs='+str(libs['csgrs']),'--extern','hypermesh='+str(libs['hypermesh']),str(WORK/'prototype-v3/tests/containment_winding.rs'),'-o',str(binary)])==0
native_exit=run('native-tests',[str(binary),'--nocapture','--test-threads=1'])
js=next(p for p in wasm if p.endswith('/csgrs.js'))
wasm_exit=run('wasm-tests',['node',str(WORK/'prototype-v3/tests/wasm/containment_winding.cjs'),js],wasm=True)
assert all(sha(p)==h for p,h in pins.items()) and all(sha(p)==h for p,h in probes.items())
assert all(sha(ROOT/p)==h for p,h in current.items())
assert native_exit==101 and 'running 15 tests' in (OUT/'native-tests.stdout').read_text()
assert wasm_exit==1 and '# tests 9' in (OUT/'wasm-tests.stdout').read_text()
dump(OUT/'result.json',{'complete':True,'native_test_exit_code':native_exit,'wasm_test_exit_code':wasm_exit,'native_test_binary_sha256':sha(binary),'current_inputs_unchanged':True,'read_only_artifacts_unchanged':True,'candidate_admitted':False})
