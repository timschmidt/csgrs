"""Pin the narrow WASM mesh-error admission without rewriting native disposition or history."""
import ast
import concurrent.futures
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode=True
ROOT=Path('/home/tim/Documents/GitHub/workspace/csgrs')
E=ROOT/'docs/evidence/2026-10-16-wasm-mesh-failures'
PARENT=E.parent/'2026-10-16-native-change-disposition'
WORK=ROOT/'target/properties/wasm-mesh-failure-review'
OLD=ROOT/'target/properties/native-change-disposition'
PREFIX=str(E.relative_to(ROOT))+'/'
sys.path.insert(0,str(E.parent/'2026-10-16-physical-build-requalification'))
from load_expectations_v1 import load


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def read(path):return json.loads(Path(path).read_text())
def gz(path):
    with gzip.open(path,'rt') as stream:return json.load(stream)
def span(raw):
    start=re.search(rb'"current_work"\s*:\s*',raw).end()
    value,length=json.JSONDecoder().raw_decode(raw[start:].decode())
    return start,start+len(raw[start:].decode()[:length].encode()),value


assert not (E/'checkpoint-v1.json').exists() and not (E/'artifact-expectations-v1.json.gz').exists()
previous_path=PARENT/'checkpoint-v1.json'
previous=read(previous_path)
assert previous['disposition_complete'] and previous['pending_native_sha256']=={}
assert all(sha(ROOT/path)==digest for path,digest in previous['evidence_files_sha256'].items())
assert sha(ROOT/previous['artifact_expectations'])==previous['artifact_expectations_sha256']
prior,inherited_missing=load(ROOT/previous['artifact_expectations'],allow_unqualified=True)
assert len(prior)==previous['inherited_expected_artifact_paths']+previous['new_physical_artifacts_verified']
assert len(inherited_missing)==27093
application,decision=read(E/'application-v1.json'),read(E/'decision-v1.json')
BASE=application['main_parent_commit']
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
implementation=application['implementation_sha256']
assert implementation==decision['implementation_sha256'] and decision['wasm_mesh_failures_admitted']
assert set(implementation)=={'src/wasm/mesh_js.rs','scripts/test-wasm-bindings.sh','tests/wasm/mesh_errors.cjs','tests/wasm/mesh_errors.ts'}
inherited=previous['inherited_implementation_sha256']|previous['implementation_sha256']
extra,dependencies=previous['extra_unchanged_inputs_sha256'],previous['live_source_dependency_pins_sha256']
assert all(sha(ROOT/path)==digest for pins in [inherited|implementation,extra,dependencies] for path,digest in pins.items())
assert all(sha(ROOT/path)==digest for path,digest in previous['restored_native_sha256'].items())
assert all(not (ROOT/path).exists() for path in previous['removed_added_paths'])
assert all(sha(ROOT/previous['candidate_snapshot']/path)==digest for path,digest in previous['rejected_native_sha256'].items())
prep=read(E/'preparation-v1/result.json')
assert prep['complete'] and sha(E/'prepare_wasm_v1.py')==prep['runner_sha256']
assert sha(E/'preparation-v1/source-hashes.json.gz')==prep['source_hashes_sha256']
qualified=gz(E/'preparation-v1/source-hashes.json.gz')
inputs=gz(E/'final-input-hashes-v1.json.gz')
assert len(qualified)==len(inputs)==2826
assert all(sha(WORK/'candidate-source-v1'/path)==digest for path,digest in qualified.items())
assert all(sha(ROOT/path)==digest for path,digest in inputs.items())
assert {path for path in inputs if inputs[path]!=qualified[path]}=={'docs/properties-status.json'}
assert inputs['docs/properties-status.json']==previous['ledger_after_sha256']
metadata=application['admission_metadata_only_difference']
assert metadata['qualified_sha256']==qualified['docs/properties-status.json']
assert metadata['current_sha256']==inputs['docs/properties-status.json']
assert metadata['raw_bytes_outside_current_work_equal'] and metadata['other_decoded_keys_equal']
assert sha(E/'final-input-hashes-v1.json.gz')==application['final_input_hashes_sha256']
assert application['format_exit_code']==0 and application['pending_native_paths_remaining']==0
for suffix in ['stdout','stderr']:assert sha(E/('main-format.'+suffix))==application['format_'+suffix+'_sha256']
for name,row in application['retained_completed_evidence'].items():assert sha(E/name)==row['sha256']==sha(row['original'])
for path in E.rglob('*.py'):ast.parse(path.read_text(),filename=str(path))
source_baseline=read(E/'prototype-v1/baseline-source.json')
baseline_binding={'logical_path':source_baseline['path'],'retained_path':str(OLD/'committed-source-v1/src/wasm/mesh_js.rs'),'sha256':source_baseline['sha256']}
assert sha(baseline_binding['retained_path'])==baseline_binding['sha256']
for folder,runner in [('wasm-validation-v1','wasm_v1.py'),('native-validation-v1','native_v1.py')]:
    protocol=read(E/folder/'protocol.json');result=read(E/folder/'result.json')
    assert result['complete'] and result['commands']==3 and not result['unexpected_exits']
    assert sha(E/runner)==protocol['runner_sha256'] and sha(E/'wasm_validation_common_v1.py')==protocol['common_sha256']
    assert sha(E/'preparation-v1/source-hashes.json.gz')==protocol['source_hashes_sha256']
    assert all(sha(path)==digest for path,digest in protocol.get('tools_read_only_sha256',{}).items())
    for row in read(E/folder/'commands.json'):
        assert row['exit_code']==0
        for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        assert all(sha(record['retained'])==record['sha256'] for record in row['binaries'].values())
wasm=read(E/'wasm-validation-v1/result.json')
assert wasm['real_js_and_compiled_typescript_consumers_passed']
assert all(sha(path)==row['sha256'] for path,row in wasm['artifacts'].items())
baseline=read(E/'baseline-v1/protocol.json')
assert sha(E/'baseline_v1.py')==baseline['runner_sha256']
assert all(sha(path)==digest for path,digest in baseline['prototype_sha256'].items())
assert all(sha(path)==digest for path,digest in baseline['read_only_artifacts_sha256'].items())
assert all(sha(path)==digest for path,digest in baseline['copied_consumer_inputs_sha256'].items())
assert sha(baseline['tsc'])==baseline['tsc_sha256']
preflight=read(E/'preflight-v1/protocol.json')
assert sha(E/'preflight-v1/run.py')==preflight['runner_sha256']
assert sha(E/'preflight-v1/probe.cjs')==preflight['probe_sha256']
assert sha(E/'preflight-v1/contract.json')==preflight['contract_sha256']
assert all(sha(path)==digest for path,digest in preflight['read_only_artifacts_sha256'].items())
cost=read(E/'cost-v1/result.json');protocol=read(E/'cost-v1/protocol.json')
assert cost['complete'] and cost['processes']==10 and cost['complete_success_values_and_stl_bytes_equal'] and cost['native_diagnostic_messages_equal']
assert all(sha(path)==digest for path,digest in protocol['runner_probe_contract_qualification_sha256'].items())
assert all(sha(path)==row['sha256'] for path,row in protocol['artifacts_read_only'].items())
for folder in ['baseline-v1','cost-v1']:
    for row in read(E/folder/'commands.json'):
        for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']

# Validate the protected ledger boundary before producing checkpoint outputs.
ledger=ROOT/'docs/properties-status.json'
raw,before=ledger.read_bytes(),sha(ledger)
assert before==previous['ledger_after_sha256']
reference=subprocess.check_output(['git','show','9025de62:docs/properties-status.json'],cwd=ROOT)
start,end,work=span(raw)
rstart,rend,_=span(reference)
assert (raw[:start],raw[end:])==(reference[:rstart],reference[rend:])

# Every physical path is read separately. Never infer identity from inode reuse.
paths=[]
symlinks={}
for path in WORK.rglob('*'):
    assert not path.is_mount(),path
    if path.is_symlink():
        target=path.resolve(strict=True)
        assert target.is_file() and target.is_relative_to(WORK),path
        symlinks[str(path)]={'link_text':os.readlink(path),'resolved_target':str(target)}
    if path.is_file():paths.append(path)


def inspect(path):
    stat_before,link_before=path.stat(),path.lstat()
    digest=sha(path)
    stat_after,link_after=path.stat(),path.lstat()
    for left,right in [(stat_before,stat_after),(link_before,link_after)]:
        assert all(getattr(left,key)==getattr(right,key) for key in ['st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns']),path
    if str(path) in symlinks:
        assert os.readlink(path)==symlinks[str(path)]['link_text']
        assert str(path.resolve(strict=True))==symlinks[str(path)]['resolved_target']
    return str(path),digest


print('Hashing completed physical WASM mesh-failure files:',len(paths),flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical=dict(pool.map(inspect,paths,buffersize=128))
assert prior.keys().isdisjoint(physical)
expectations={'schema_version':3,'kind':'new physical WASM mesh-failure evidence; inherited historical audit remains failed/incomplete',
 'parent_expectations':previous['artifact_expectations'],'parent_expectations_sha256':previous['artifact_expectations_sha256'],
 'inherited_expected_artifact_paths':len(prior),'inherited_missing_artifact_paths':len(inherited_missing),
 'added_physical_artifacts_sha256':physical,'new_physical_artifacts_verified':len(physical),
 'local_tool_symlink_targets':symlinks,'expected_artifact_paths':len(prior)+len(physical),
 'inherited_artifacts_reverified':False,'complete_artifact_audit_passed':False,'historical_preservation_complete':False,
 'scope':'Completed independent candidate sources, native/WASM targets, generated consumers, metadata, logs and probes are hashed independently per path with stable stat/lstat checks. Parent expectations, known nested-view failures and inherited missing paths remain unchanged. No whole-history verification or zero-loss claim.'}
with gzip.open(E/'artifact-expectations-v1.json.gz','wt') as stream:json.dump(expectations,stream,sort_keys=True)
assert gz(E/'artifact-expectations-v1.json.gz')==expectations
work['updated_on']='2026-10-16'
work['wasm_mesh_failures']={
 'status':decision['status'],'main_parent_commit':BASE,'contract':PREFIX+'prototype-v1/contract.json',
 'progress':PREFIX+'progress-v1.md','decision':PREFIX+'decision-v1.json','checkpoint':PREFIX+'checkpoint-v1.json',
 'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz','implementation_sha256':implementation,
 'cost':PREFIX+'cost-v1/result.json','cost_contract':PREFIX+'cost-contract-v1.json',
 'wasm_mesh_failures_admitted':True,'pending_native_paths_remaining':0,'native_disposition_preserved':True,
 'source_metadata_difference':metadata,'published_package_pipeline_executed':False,'full_plan_complete':False,'next':decision['next']}
updated=raw[:start]+json.dumps(work,indent=2).encode()+raw[end:]
ustart,uend,_=span(updated)
assert (updated[:ustart],updated[uend:])==(reference[:rstart],reference[rend:])
assert {key:value for key,value in json.loads(updated).items() if key!='current_work'}=={
    key:value for key,value in json.loads(reference).items() if key!='current_work'}
assert sha(ledger)==before
ledger.write_bytes(updated)
files={str(path.relative_to(ROOT)):sha(path) for path in E.rglob('*') if path.is_file() and '__pycache__' not in path.parts}
record={'status':decision['status'],'checkpoint_kind':'narrow WASM mesh-error admission; native disposition and historical limitations retained',
 'previous_closure':str(previous_path.relative_to(ROOT)),'previous_closure_sha256':sha(previous_path),
 'evidence_files_sha256':files,'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz',
 'artifact_expectations_sha256':sha(E/'artifact-expectations-v1.json.gz'),'artifact_expectations_schema':3,
 'new_physical_artifacts_verified':len(physical),'inherited_expected_artifact_paths':len(prior),
 'inherited_missing_artifact_paths':len(inherited_missing),'complete_artifact_audit_passed':False,
 'historical_preservation_complete':False,'implementation_sha256':implementation,
 'inherited_implementation_sha256':inherited,'extra_unchanged_inputs_sha256':extra,
 'pending_native_sha256':{},'rejected_native_sha256':previous['rejected_native_sha256'],
 'restored_native_sha256':previous['restored_native_sha256'],'removed_added_paths':previous['removed_added_paths'],
 'candidate_snapshot':previous['candidate_snapshot'],'live_source_dependency_pins_sha256':dependencies,
 'ledger_before_sha256':before,'ledger_after_sha256':sha(ledger),'only_current_work_changed':True,
 'baseline_source_binding':baseline_binding,'admission_metadata_only_difference':metadata,
 'python_sources_parse':True,'wasm_mesh_failures_admitted':True,'native_disposition_preserved':True,
 'native_candidate_admitted':False,'native_runtime_admitted':False,'full_plan_complete':False}
(E/'checkpoint-v1.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
print(json.dumps({key:record[key] for key in ['new_physical_artifacts_verified','only_current_work_changed','wasm_mesh_failures_admitted']}),flush=True)
