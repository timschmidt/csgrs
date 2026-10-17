"""Retain every completed-contract cost observation from immutable native libraries."""
import hashlib,json,resource,statistics,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent; O=W/'cost-v1'; OLD=W.parent/'containment-winding'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
qualified=[OLD/'candidate-validation-v3/result.json',W/'candidate-validation-v1/result.json',W/'wasm-validation-v1/result.json']
for p in qualified:
 r=read(p);assert r['complete'] and not r['unexpected_exits']
assert read(W/'wasm-validation-v1/result.json')['real_js_and_compiled_typescript_consumers_passed']
contract=read(W/'cost-contract-v1.json')
assert contract['pairs']==5
assert all(sha(p)==h for p,h in contract['probe_sha256'].items())
O.mkdir(exist_ok=False)
variants={};fingerprints={}
for variant,work,version in [('baseline',OLD,'v3'),('candidate',W,'v1')]:
 rows=[json.loads(line) for line in (work/('candidate-validation-'+version)/'cost-libraries.stdout').read_text().splitlines() if line.startswith('{')]
 library=next(Path(p) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='hypermesh' for p in row['filenames'] if p.endswith('.rlib'))
 deps=library.parent; fingerprint=deps.parent/'.fingerprint'/library.stem.removeprefix('lib')/'lib-hypermesh.json'
 fingerprints[variant]={'path':str(fingerprint),'sha256':sha(fingerprint),'value':read(fingerprint)}
 variants[variant]={'library':str(library),'deps':str(deps),'source_manifest_sha256':sha(work/('candidate-preparation-'+version)/'source-hashes.json.gz'),'binaries':{}}
assert fingerprints['baseline']['value']==fingerprints['candidate']['value']
readonly={str(p):sha(p) for v in variants.values() for p in Path(v['deps']).glob('*.rlib')}
pins={str(p):sha(p) for p in [Path(__file__),W/'control_cost_v1.rs',W/'matrix_cost_v1.rs',W/'cost-contract-v1.json',*qualified]}
dump(O/'protocol.json',{'input_sha256':pins,'read_only_library_sha256':readonly,'variants':variants,'matching_native_fingerprints':fingerprints,'rustc':subprocess.check_output(['rustc','-vV'],text=True),'hardware':subprocess.check_output(['lscpu'],text=True),'external_seconds_per_command':180,'address_space_bytes':8*1024**3,'pairs_per_scope':5,'scopes':['control','matrix'],'completed_cargo_targets_reopened':False,'production_latency_claimed':False})
commands=[];reports={}
def limits():
 resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
def run(name,command):
 start=time.monotonic()
 with (O/(name+'.stdout')).open('wb') as out,(O/(name+'.stderr')).open('wb') as err:
  p=subprocess.run(['/usr/bin/time','-v','-o',str(O/(name+'.resources')),'timeout','--kill-after=10s','180s',*command],cwd=O,stdout=out,stderr=err,preexec_fn=limits,start_new_session=True)
 row={'name':name,'command':command,'exit_code':p.returncode,'elapsed_seconds':time.monotonic()-start,**{s+'_sha256':sha(O/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
 commands.append(row);dump(O/'commands.json',commands)
 print(json.dumps({k:row[k] for k in ['name','exit_code','elapsed_seconds']}),flush=True)
 assert p.returncode==0,name
for scope in ['control','matrix']:
 for variant,v in variants.items():
  binary=O/(variant+'-'+scope)
  run('compile-'+variant+'-'+scope,['rustc','--edition=2024','-D','warnings','-C','opt-level=3','-L','dependency='+v['deps'],'--extern','hypermesh='+v['library'],str(W/(scope+'_cost_v1.rs')),'-o',str(binary)])
  v['binaries'][scope]={'path':str(binary),'sha256':sha(binary),'bytes':binary.stat().st_size}
for scope in ['control','matrix']:
 for pair in range(5):
  for variant in (['baseline','candidate'] if pair%2==0 else ['candidate','baseline']):
   name=f'{scope}-{pair:02d}-{variant}';run(name,[variants[variant]['binaries'][scope]['path']]);rows=read(O/(name+'.stdout'))
   assert len(rows)==(6 if scope=='control' else 126)
   for r in rows:
    if scope=='control' and 'completed' in r:assert r['completed']==r['certified']==64 and r['values']==[True,False,False,False]*16
    if 'observations' in r:assert r['attempted']==len(r['observations'])==8
   reports[name]=rows
assert all(sha(p)==h for p,h in readonly.items()) and all(sha(p)==h for p,h in pins.items())
paired=[];counts={v:{} for v in variants}
for scope in ['control','matrix']:
 examples=reports[f'{scope}-00-baseline']
 for example in examples:
  if 'elapsed_ns' not in example:continue
  key={k:example[k] for k in ['case','policy','phase'] if k in example}
  pairs=[]
  for pair in range(5):
   row={v:next(r for r in reports[f'{scope}-{pair:02d}-{v}'] if all(r.get(k)==x for k,x in key.items())) for v in variants}
   comparable=scope=='control' or all(all(o['status']=='certified_completion' for o in r['observations']) for r in row.values())
   pairs.append({'pair':pair,'same_certified_contract_completed':comparable,'elapsed_ratio':row['candidate']['elapsed_ns']/row['baseline']['elapsed_ns'] if comparable else None,**{k+'_delta':row['candidate'][k]-row['baseline'][k] for k in ['allocation_calls','allocated_bytes','retained_requested_bytes_delta','peak_extra_requested_bytes']}})
  values=[p['elapsed_ratio'] for p in pairs if p['elapsed_ratio'] is not None]
  paired.append({'scope':scope,**key,'all_pairs':pairs,'median_elapsed_ratio':statistics.median(values) if len(values)==5 else None})
for variant in variants:
 for pair in range(5):
  for r in reports[f'matrix-{pair:02d}-{variant}']:
   for o in r.get('observations',[]):counts[variant][o['status']]=counts[variant].get(o['status'],0)+1
result={'complete':True,'processes':20,'control_certified_query_completions':2560,'matrix_query_attempts':6720,'matrix_completion_categories':counts,'candidate_matrix_all_certified':counts['candidate']=={'certified_completion':3360},'variants':variants,'all_rows':reports,'paired_summary':paired,'read_only_artifacts_unchanged':True,'production_latency_claimed':False}

assert counts['baseline']==counts['candidate']=={'certified_completion':3360}
gate_rows=[]
for row in paired:
 assert all(p['same_certified_contract_completed'] for p in row['all_pairs'])
 if row['scope']=='control':
  passed=all(p['allocation_calls_delta']<0 and p['allocated_bytes_delta']<0 for p in row['all_pairs'])
  gate_rows.append({'scope':'control','policy':row['policy'],'phase':row['phase'],'passed':passed})
 elif row['case'] in contract['targeted_matrix_cases'] and row['phase']=='warm_query':
  reductions=[]
  for pair in range(5):
   key={k:row[k] for k in ['case','policy','phase']}
   r={v:next(x for x in reports[f'matrix-{pair:02d}-{v}'] if all(x.get(k)==value for k,value in key.items())) for v in variants}
   reductions.append(1-r['candidate']['allocation_calls']/r['baseline']['allocation_calls'])
  gate_rows.append({'scope':'matrix',**key,'allocation_call_reductions':reductions,'passed':all(r>=contract['required_allocation_call_reduction_on_each_warm_targeted_row'] for r in reductions)})
assert len(gate_rows)==16
result['preregistered_allocation_gates']=gate_rows
result['all_preregistered_allocation_gates_passed']=all(row['passed'] for row in gate_rows)
result['baseline_and_candidate_all_correct_certified']=True
result['claim_scope']='Five matched allocator-instrumented fresh-process pairs; no uninstrumented production speedup claim.'

dump(O/'result.json',result)
print(json.dumps({k:result[k] for k in ['complete','matrix_completion_categories','candidate_matrix_all_certified']}),flush=True)

assert result["all_preregistered_allocation_gates_passed"], "Preregistered allocation gates failed; retain all results and reject this optimization."
