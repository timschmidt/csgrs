"""Compare all cold/warm completion and allocation rows; never reopen Cargo targets."""
import hashlib,json,os,resource,statistics,subprocess,time
from pathlib import Path
WORK=Path(__file__).resolve().parent;OUT=WORK/'cost-v1'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
qualified=[WORK/'baseline-validation-v1/result.json',WORK/'candidate-validation-v2/result.json',WORK/'wasm-validation-v2/result.json']
for p in qualified:
 r=read(p);assert r['complete'] and not r['unexpected_exits']
assert read(WORK/'wasm-reproduction-v1/result.json')['expected_failure_reproduced']
assert '4 passed; 6 failed' in (WORK/'baseline-validation-v1/containment-inputs.stdout').read_text()
assert '# pass 13' in (WORK/'wasm-validation-v2/real-consumers.stdout').read_text()
assert '10 passed; 0 failed' in (WORK/'candidate-validation-v2/containment-inputs.stdout').read_text()
OUT.mkdir(exist_ok=False)
variants={}
baseline_matches=list((WORK/'baseline-target-v1/release/deps').glob('libhypermesh-*.rlib'))
assert len(baseline_matches)==1
library_name=baseline_matches[0].name
fingerprint_name=baseline_matches[0].stem.removeprefix('lib')
fingerprints={}
for variant in ['baseline','candidate']:
 target=WORK/('baseline-target-v1' if variant=='baseline' else 'candidate-target-v2')
 deps=target/'release/deps';library=deps/library_name
 fingerprint=target/'release/.fingerprint'/fingerprint_name/'lib-hypermesh.json'
 assert library.is_file() and fingerprint.is_file()
 fingerprints[variant]={'path':str(fingerprint),'sha256':sha(fingerprint),'value':read(fingerprint)}
 variants[variant]={'dependencies':str(deps),'libraries':{'hypermesh':str(library)},'binary':str(OUT/(variant+'-probe'))}
assert fingerprints['baseline']['value']==fingerprints['candidate']['value']
read_only={str(p):sha(p) for variant in variants.values() for p in Path(variant['dependencies']).glob('*.rlib')}
pins={str(p):sha(p) for p in [Path(__file__),WORK/'cost_v1.rs',WORK/'cost-contract-v1.json',*qualified]}
protocol={'runner_probe_contract_qualification_sha256':pins,'read_only_library_sha256':read_only,'variants':variants,'pairs':5,'rustc':subprocess.check_output(['rustc','-vV'],text=True),'hardware':subprocess.check_output(['lscpu'],text=True),'external_seconds_per_command':120,'address_space_bytes_per_process':8*1024**3,'source_manifests':{v:sha(WORK/(v+'-preparation-'+('v1' if v=='baseline' else 'v2'))/'source-hashes.json.gz') for v in variants},'optimization_claimed':False,'matched_native_fingerprints':fingerprints}
dump(OUT/'protocol.json',protocol);commands=[];reports={}
def limits():
 resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
 start=time.monotonic()
 with (OUT/(name+'.stdout')).open('wb') as out,(OUT/(name+'.stderr')).open('wb') as err:
  p=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/(name+'.resources')),'timeout','--kill-after=10s','120s',*command],stdout=out,stderr=err,cwd=OUT,preexec_fn=limits,start_new_session=True)
 row={'name':name,'command':command,'exit_code':p.returncode,'elapsed_seconds':time.monotonic()-start,**{s+'_sha256':sha(OUT/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
 commands.append(row);dump(OUT/'commands.json',commands);assert p.returncode==0,name
for name,v in variants.items():
 run('compile-'+name,['rustc','--edition=2024','-C','opt-level=3','-L','dependency='+v['dependencies'],'--extern','hypermesh='+v['libraries']['hypermesh'],str(WORK/'cost_v1.rs'),'-o',v['binary']])
 v['binary_sha256']=sha(v['binary']);v['binary_bytes']=Path(v['binary']).stat().st_size
for pair in range(5):
 for variant in (['baseline','candidate'] if pair%2==0 else ['candidate','baseline']):
  name=f'{pair:02d}-{variant}';run(name,[variants[variant]['binary']]);rows=read(OUT/(name+'.stdout'))
  assert len(rows)==6
  for r in rows:
   if 'completed' in r:assert r['completed']==r['certified']==64 and r['values']==[True,False,False,False]*16
  reports[name]=rows;print(name+' passed',flush=True)
assert all(sha(p)==h for p,h in read_only.items()) and all(sha(p)==h for p,h in pins.items())
summary=[]
for policy in ['strict','approximate']:
 for phase in ['cold_construct_query','warm_query']:
  paired=[]
  for pair in range(5):
   rows={v:next(r for r in reports[f'{pair:02d}-{v}'] if r['policy']==policy and r['phase']==phase) for v in variants}
   paired.append({'pair':pair,'elapsed_ratio':rows['candidate']['elapsed_ns']/rows['baseline']['elapsed_ns'],**{key+'_delta':rows['candidate'][key]-rows['baseline'][key] for key in ['allocation_calls','allocated_bytes','retained_requested_bytes_delta','peak_extra_requested_bytes']}})
  summary.append({'policy':policy,'phase':phase,'all_pairs':paired,'median_elapsed_ratio':statistics.median(r['elapsed_ratio'] for r in paired)})
dump(OUT/'result.json',{'complete':True,'processes':10,'certified_query_completions':2560,'variants':variants,'all_rows':reports,'paired_summary':summary,'all_expected_values_and_certainties_match':True,'read_only_libraries_and_probes_unchanged':True,'optimization_claimed':False})
print('All focused cost and allocation observations retained',flush=True)
