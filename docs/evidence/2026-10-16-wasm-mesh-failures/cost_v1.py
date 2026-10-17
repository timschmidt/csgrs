"""Retain every paired completed-output/resource observation without a speedup claim."""
import hashlib,json,os,resource,subprocess,time
from pathlib import Path
WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[2]
OLD=ROOT/'target/properties/native-change-disposition'
OUT=WORK/'cost-v1'
def sha(path):
 with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def read(path):return json.loads(Path(path).read_text())
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')
qualification_paths=[OLD/'committed-validation-v1/result.json',WORK/'native-validation-v1/result.json',WORK/'wasm-validation-v1/result.json']
for path in qualification_paths:
 q=read(path);assert q['complete'] and not q.get('failed_commands',q.get('unexpected_exits'))
variants={'baseline':read(OLD/'wasm-validation-v1/result.json'),'candidate':read(WORK/'wasm-validation-v1/result.json')}
artifacts={path:row for result in variants.values() for path,row in result['artifacts'].items()}
assert all(sha(path)==row['sha256'] for path,row in artifacts.items())
probes={str(path):sha(path) for path in [Path(__file__),WORK/'cost_v1.cjs',WORK/'cost-contract-v1.json',*qualification_paths]}
OUT.mkdir(exist_ok=False)
protocol={'runner_probe_contract_qualification_sha256':probes,'artifacts_read_only':artifacts,
 'node':subprocess.check_output(['node','--version'],text=True).strip(),
 'hardware':subprocess.check_output(['lscpu'],text=True),
 'source_manifests':{'baseline':sha(OLD/'preparation-v1/committed-source-hashes.json.gz'),'candidate':sha(WORK/'preparation-v1/source-hashes.json.gz')},
 'pairs':5,'process_limit_seconds':60,'address_space_limit_bytes':64*1024**3,
 'independent_mathematical_oracle_claimed':False,'optimization_claimed':False,
 'timing_boundary':'Within-process timers cover construction, native operations, arrays/STL materialization and forced error-stack access; final JSON report encoding is outside those timers and inside whole-process GNU time.'}
dump(OUT/'protocol.json',protocol)
rows=[];reports={};reference_success=None;reference_diagnostics=None

def limits():
 resource.setrlimit(resource.RLIMIT_AS,(64*1024**3,64*1024**3))
 resource.setrlimit(resource.RLIMIT_CORE,(0,0))

for pair in range(5):
 for variant in (['baseline','candidate'] if pair%2==0 else ['candidate','baseline']):
  result=variants[variant]
  module=next(path for path in result['artifacts'] if path.endswith('/csgrs.js'))
  name=f'{pair:02d}-{variant}'
  before=time.monotonic()
  command=['node',str(WORK/'cost_v1.cjs'),module]
  env=os.environ.copy();env.pop('NODE_OPTIONS',None)
  with (OUT/(name+'.stdout')).open('wb') as stdout,(OUT/(name+'.stderr')).open('wb') as stderr:
   p=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/(name+'.resources')),'timeout','--signal=TERM','--kill-after=5s','60s',*command],cwd=OUT,env=env,stdout=stdout,stderr=stderr,preexec_fn=limits,start_new_session=True)
  row={'name':name,'variant':variant,'pair':pair,'command':command,'exit_code':p.returncode,'elapsed_seconds':time.monotonic()-before,**{suffix+'_sha256':sha(OUT/(name+'.'+suffix)) for suffix in ['stdout','stderr','resources']}}
  rows.append(row);dump(OUT/'commands.json',rows)
  assert p.returncode==0,name
  report=read(OUT/(name+'.stdout'))
  assert report['completed']==5 and report['diagnosed']==25
  assert all(r['certainty']==0 and r['emptyCertainty']==0 and r['empty'] and r['final']=={'positions':[],'normals':[],'indices':[]} for r in report['successes'])
  if reference_success is None:reference_success=report['successes']
  assert report['successes']==reference_success
  diagnostics=[{'name':r['name'],'diagnostic':r['diagnostic']} for r in report['failures']]
  if reference_diagnostics is None:reference_diagnostics=diagnostics
  assert diagnostics==reference_diagnostics
  if variant=='baseline':assert all(r['type']=='string' and not r['isError'] and r['stack'] is None for r in report['failures'])
  else:assert all(r['type']=='object' and r['isError'] and r['errorName']=='GeometryError' and r['code']=='invalid_input' and isinstance(r['stack'],str) and r['stack'] for r in report['failures'])
  reports[name]={'completed':report['completed'],'diagnosed':report['diagnosed'],'elapsed_ns':report['elapsedNs'],'cpu_microseconds':report['cpuMicroseconds'],'successful_phase':report['successfulPhase'],'diagnostic_phase':report['diagnosticPhase']}
  print(json.dumps({'sample':name,'complete':True}),flush=True)
assert all(sha(path)==row['sha256'] for path,row in artifacts.items())
assert all(sha(path)==digest for path,digest in probes.items())
result={'complete':True,'processes':len(rows),'completed_geometry_pipelines':50,'typed_or_legacy_diagnostics':250,
 'complete_success_values_and_stl_bytes_equal':True,'native_diagnostic_messages_equal':True,
 'intentional_error_contract_migration_verified':True,'all_samples':reports,
 'baseline_processed_wasm_bytes':next(row['bytes'] for path,row in variants['baseline']['artifacts'].items() if path.endswith('/csgrs_bg.wasm')),
 'candidate_processed_wasm_bytes':next(row['bytes'] for path,row in variants['candidate']['artifacts'].items() if path.endswith('/csgrs_bg.wasm')),
 'artifacts_and_probes_unchanged':True,'optimization_claimed':False,'candidate_admitted':False}
dump(OUT/'result.json',result)
print(json.dumps({key:result[key] for key in ['complete','processes','completed_geometry_pipelines','typed_or_legacy_diagnostics','complete_success_values_and_stl_bytes_equal','native_diagnostic_messages_equal']}),flush=True)
