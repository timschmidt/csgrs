"""Pin the narrow containment-input migration and preserve all inherited limitations."""
import ast,concurrent.futures,gzip,hashlib,json,os,re,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
WORK=Path(__file__).resolve().parent;ROOT=WORK.parents[2]
E=ROOT/'docs/evidence/2026-10-16-containment-input-validation'
PARENT=ROOT/'docs/evidence/2026-10-16-wasm-mesh-failures'
PREFIX=str(E.relative_to(ROOT))+'/'
sys.path.insert(0,str(E.parent/'2026-10-16-physical-build-requalification'))
from load_expectations_v1 import load

def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def gz(p):
 with gzip.open(p,'rt') as f:return json.load(f)
def span(raw):
 start=re.search(rb'"current_work"\s*:\s*',raw).end();value,n=json.JSONDecoder().raw_decode(raw[start:].decode())
 return start,start+len(raw[start:].decode()[:n].encode()),value
assert not (E/'checkpoint-v1.json').exists() and not (E/'artifact-expectations-v1.json.gz').exists()
previous_path=PARENT/'checkpoint-v1.json';previous=read(previous_path)
assert previous['pending_native_sha256']=={} and previous['native_disposition_preserved']
assert all(sha(ROOT/p)==h for p,h in previous['evidence_files_sha256'].items())
assert sha(ROOT/previous['artifact_expectations'])==previous['artifact_expectations_sha256']
prior,missing=load(ROOT/previous['artifact_expectations'],allow_unqualified=True)
assert len(prior)==978660 and len(missing)==27093
application=read(E/'application-v1.json');decision=read(E/'decision-v1.json')
BASE=application['main_parent_commit'];assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
implementation=application['implementation_sha256'];assert implementation==decision['implementation_sha256']
assert decision['containment_input_validation_admitted'] and not decision['full_set_membership_correctness_claimed']
inherited=previous['inherited_implementation_sha256']|previous['implementation_sha256']
extra=previous['extra_unchanged_inputs_sha256'];dependencies=previous['live_source_dependency_pins_sha256']
assert all(sha(ROOT/p)==h for pins in [inherited|implementation,extra,dependencies,previous['restored_native_sha256']] for p,h in pins.items())
assert all(not (ROOT/p).exists() for p in previous['removed_added_paths'])
assert all(sha(ROOT/previous['candidate_snapshot']/p)==h for p,h in previous['rejected_native_sha256'].items())
assert application['format_exit_code']==0 and application['all_final_inputs_match_qualified_sources']
qualified=gz(E/'candidate-preparation-v2/source-hashes.json.gz');final=gz(E/'final-input-hashes-v1.json.gz')
assert qualified==final and len(final)==2827
assert all(sha(WORK/'candidate-source-v2'/p)==h and sha(ROOT/p)==h for p,h in final.items())
assert final['docs/properties-status.json']==previous['ledger_after_sha256']
assert all(sha(WORK/'baseline-source-v1'/p)==h for p,h in gz(E/'baseline-preparation-v1/source-hashes.json.gz').items())
assert sha(E/'final-input-hashes-v1.json.gz')==application['final_input_hashes_sha256']
for n,r in application['retained_completed_evidence'].items():assert sha(E/n)==sha(r['original'])==r['sha256']
for folder,runner,common in [('baseline-validation-v1','baseline_v1.py','validation_common_v1.py'),('candidate-validation-v2','candidate_v2.py','validation_common_v2.py'),('wasm-validation-v2','wasm_v2.py','validation_common_v2.py')]:
 protocol=read(E/folder/'protocol.json');result=read(E/folder/'result.json');assert result['complete'] and not result['unexpected_exits']
 assert sha(E/runner)==protocol['runner_sha256'] and sha(E/common)==protocol['common_sha256']
 for row in read(E/folder/'commands.json'):
  assert row['exit_code']==row['expected_exit_code']
  for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
  assert all(sha(r['retained'])==r['sha256'] for r in row['binaries'].values())
 assert all(sha(p)==h for p,h in protocol.get('tools_read_only_sha256',{}).items())
assert read(E/'wasm-reproduction-v1/result.json')['expected_failure_reproduced']
assert read(E/'baseline-wasm-limit-note-v1.json')['classification']=='external-address-space-exhaustion-before-tests'
assert all(sha(p)==r['sha256'] for p,r in read(E/'wasm-validation-v2/result.json')['artifacts'].items())
assert read(E/'cost-v1/result.json')['complete'] and read(E/'cost-v1/result.json')['all_expected_values_and_certainties_match']
assert (E/'winding-diagnostic-v1/stdout').read_bytes()==(E/'winding-followup-v1/stdout').read_bytes()
for p in E.rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
ledger=ROOT/'docs/properties-status.json';raw=ledger.read_bytes();before=sha(ledger)
assert before==previous['ledger_after_sha256']
reference=subprocess.check_output(['git','show','9025de62:docs/properties-status.json'],cwd=ROOT)
start,end,current=span(raw);rs,re_,_=span(reference)
assert (raw[:start],raw[end:])==(reference[:rs],reference[re_:])
paths=[];links={}
for p in WORK.rglob('*'):
 assert not p.is_mount(),p
 if p.is_symlink():
  target=p.resolve(strict=True);assert target.is_file() and target.is_relative_to(WORK),p
  links[str(p)]={'link_text':os.readlink(p),'resolved_target':str(target)}
 if p.is_file():paths.append(p)
def inspect(p):
 before,link_before=p.stat(),p.lstat();digest=sha(p);after,link_after=p.stat(),p.lstat()
 for a,b in [(before,after),(link_before,link_after)]:assert all(getattr(a,k)==getattr(b,k) for k in ['st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns']),p
 if str(p) in links:assert os.readlink(p)==links[str(p)]['link_text'] and str(p.resolve(strict=True))==links[str(p)]['resolved_target']
 return str(p),digest
print('Hashing completed physical containment evidence:',len(paths),flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:physical=dict(pool.map(inspect,paths,buffersize=128))
assert prior.keys().isdisjoint(physical)
expectations={'schema_version':3,'kind':'new physical containment-input evidence; inherited audit remains failed/incomplete','parent_expectations':previous['artifact_expectations'],'parent_expectations_sha256':previous['artifact_expectations_sha256'],'inherited_expected_artifact_paths':len(prior),'inherited_missing_artifact_paths':len(missing),'added_physical_artifacts_sha256':physical,'new_physical_artifacts_verified':len(physical),'local_tool_symlink_targets':links,'expected_artifact_paths':len(prior)+len(physical),'inherited_artifacts_reverified':False,'complete_artifact_audit_passed':False,'historical_preservation_complete':False,'scope':'Every fresh physical file independently hashed with stable stat/lstat, including failed preparation, source copies, native/WASM targets, consumers, probes and raw observations. Parent missing artifacts remain unresolved; no whole-history verification or zero-loss claim.'}
with gzip.open(E/'artifact-expectations-v1.json.gz','wt') as f:json.dump(expectations,f,sort_keys=True)
current['updated_on']='2026-10-16'
current['containment_input_validation']={'status':decision['status'],'main_parent_commit':BASE,'contract':PREFIX+'contract-v2.md','progress':PREFIX+'progress-v1.md','decision':PREFIX+'decision-v1.json','checkpoint':PREFIX+'checkpoint-v1.json','artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz','implementation_sha256':implementation,'cost':PREFIX+'cost-v1/result.json','cost_contract':PREFIX+'cost-contract-v1.json','baseline_native':decision['baseline_native'],'baseline_wasm':decision['baseline_wasm'],'new_native_tests':10,'mesh_wasm_tests':13,'containment_input_validation_admitted':True,'full_set_membership_correctness_claimed':False,'known_winding_gap':decision['known_winding_gap'],'intentional_migration':decision['intentional_migration'],'pending_native_paths_remaining':0,'native_disposition_preserved':True,'published_package_pipeline_executed':False,'full_plan_complete':False,'next':decision['next']}
updated=raw[:start]+json.dumps(current,indent=2).encode()+raw[end:];us,ue,_=span(updated)
assert (updated[:us],updated[ue:])==(reference[:rs],reference[re_:])
assert {k:v for k,v in json.loads(updated).items() if k!='current_work'}=={k:v for k,v in json.loads(reference).items() if k!='current_work'}
assert sha(ledger)==before;ledger.write_bytes(updated)
files={str(p.relative_to(ROOT)):sha(p) for p in E.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
record={'status':decision['status'],'checkpoint_kind':'narrow containment input/evidence migration; unresolved winding and inherited history preserved','previous_closure':str(previous_path.relative_to(ROOT)),'previous_closure_sha256':sha(previous_path),'evidence_files_sha256':files,'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz','artifact_expectations_sha256':sha(E/'artifact-expectations-v1.json.gz'),'artifact_expectations_schema':3,'new_physical_artifacts_verified':len(physical),'inherited_expected_artifact_paths':len(prior),'inherited_missing_artifact_paths':len(missing),'complete_artifact_audit_passed':False,'historical_preservation_complete':False,'implementation_sha256':implementation,'inherited_implementation_sha256':inherited,'extra_unchanged_inputs_sha256':extra,'pending_native_sha256':{},'rejected_native_sha256':previous['rejected_native_sha256'],'restored_native_sha256':previous['restored_native_sha256'],'removed_added_paths':previous['removed_added_paths'],'candidate_snapshot':previous['candidate_snapshot'],'live_source_dependency_pins_sha256':dependencies,'ledger_before_sha256':before,'ledger_after_sha256':sha(ledger),'only_current_work_changed':True,'python_sources_parse':True,'containment_input_validation_admitted':True,'full_set_membership_correctness_claimed':False,'native_disposition_preserved':True,'native_candidate_admitted':False,'native_runtime_admitted':False,'full_plan_complete':False}
(E/'checkpoint-v1.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
print(json.dumps({'new_physical_artifacts_verified':len(physical),'only_current_work_changed':True,'containment_input_validation_admitted':True}),flush=True)
