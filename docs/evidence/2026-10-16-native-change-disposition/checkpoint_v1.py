"""Close the native disposition while preserving rejected sources and historical evidence limits."""
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
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
PARENT=E.parent/'2026-10-16-wasm-format-failures'
WORK=ROOT/'target/properties/native-change-disposition'
BASE='99b88f707b04053de7f40a86d6d029e3c07460d9'
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
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
previous_path=PARENT/'checkpoint-v1.json'
previous=read(previous_path)
assert all(sha(ROOT/path)==digest for path,digest in previous['evidence_files_sha256'].items())
assert sha(ROOT/previous['artifact_expectations'])==previous['artifact_expectations_sha256']
prior,inherited_missing=load(ROOT/previous['artifact_expectations'],allow_unqualified=True)
assert len(prior)==947270 and len(inherited_missing)==27093
rejected=previous['pending_native_sha256']
assert len(rejected)==35 and all(sha(E/'pending-source-v1'/path)==digest for path,digest in rejected.items())
inherited=previous['inherited_implementation_sha256']|previous['implementation_sha256']
extra,dependencies=previous['extra_unchanged_inputs_sha256'],previous['live_source_dependency_pins_sha256']
decision,application=read(E/'decision-v1.json'),read(E/'application-v1.json')
implementation=application['implementation_sha256']
assert implementation==decision['implementation_sha256']=={}
assert decision['disposition_complete'] and not decision['native_candidate_admitted']
isolation=read(E/'followup-timing-isolation-v1.json')
assert sha(isolation['followup_result'])==isolation['followup_result_sha256']
assert sha(isolation['followup_native_result'])==isolation['followup_native_result_sha256']
assert all(row['followup_completion_precedes_gate_start'] and row['conservative_gap_seconds']>0 for row in isolation['gates'].values())
assert application['pending_native_paths_remaining']==0
assert application['rejected_native_sha256']==decision['rejected_native_sha256']==rejected
restored=application['restored_native_sha256']
removed=application['removed_added_paths']
assert len(restored)==29 and len(removed)==6 and set(restored)|set(removed)==set(rejected)
assert all(sha(ROOT/path)==digest for path,digest in restored.items())
assert all(not (ROOT/path).exists() for path in removed)
assert all(sha(ROOT/path)==digest for pins in [inherited,extra,dependencies] for path,digest in pins.items())
subprocess.run(['git','diff','--quiet','--',*sorted(rejected)],cwd=ROOT,check=True)
prep=read(E/'preparation-v1/result.json')
assert prep['complete'] and prep['pending_native_sha256']==rejected
assert sha(E/'prepare_v1.py')==prep['runner_sha256']
assert sha(E/'preparation-v1/tracked.patch')==prep['tracked_patch_sha256']
assert sha(E/'preparation-v1/untracked.patch')==prep['untracked_patch_sha256']
for variant,count in [('candidate',2830),('committed',2824)]:
    manifest=E/'preparation-v1'/(variant+'-source-hashes.json.gz')
    assert sha(manifest)==prep['variants'][variant]['source_hashes_sha256']
    pins=gz(manifest)
    assert len(pins)==count==prep['variants'][variant]['inputs']
    assert all(sha(WORK/(variant+'-source-v1')/path)==digest for path,digest in pins.items())
inputs=gz(E/'final-input-hashes-v1.json.gz')
assert len(inputs)==2824 and inputs==gz(E/'preparation-v1/committed-source-hashes.json.gz')
assert sha(E/'final-input-hashes-v1.json.gz')==application['final_input_hashes_sha256']
assert all(sha(ROOT/path)==digest for path,digest in inputs.items())
assert application['format_exit_code']==0
for suffix in ['stdout','stderr']:
    assert sha(E/('main-format.'+suffix))==application['format_'+suffix+'_sha256']
for path in E.rglob('*.py'):ast.parse(path.read_text(),filename=str(path))
collection=read(E/'collection-v1.json')
assert collection['complete'] and sha(E/'collect_v1.py')==collection['runner_sha256']
assert all(sha(E/name)==row['sha256']==sha(row['original']) for name,row in collection['retained_completed_evidence'].items())
fixtures=read(E/'fixture-preflight-v1/result.json')
assert fixtures['complete'] and fixtures['all_original_archives_and_entries_present'] and len(fixtures['rows'])==4
assert sha(E/'fixtures_v1.py')==fixtures['runner_sha256']
assert sha(fixtures['unzip'])==fixtures['unzip_sha256']
for fixture in prep['fixtures']:assert sha(fixture['archive'])==fixture['sha256']
for folder,runner,common,count,variant in [
 ('candidate-validation-v1','candidate_v1.py','validation_common_v1.py',7,'candidate'),
 ('committed-validation-v1','committed_v1.py','validation_common_v1.py',13,'committed'),
 ('wasm-validation-v1','wasm_v1.py','wasm_validation_common_v1.py',3,'committed')]:
    protocol=read(E/folder/'protocol.json')
    result=read(E/folder/'result.json')
    assert result['complete'] and result['commands']==count
    if variant=='candidate':assert set(result['failed_commands'])=={'full-board','copper-neck'}
    else:assert not result.get('failed_commands',result.get('unexpected_exits'))
    assert sha(E/runner)==protocol['runner_sha256'] and sha(E/common)==protocol['common_sha256']
    assert sha(E/'preparation-v1'/(variant+'-source-hashes.json.gz'))==protocol['source_hashes_sha256']
    assert all(sha(path)==digest for path,digest in protocol.get('tools_read_only_sha256',{}).items())
    rows=read(E/folder/'commands.json')
    assert len(rows)==count
    for row in rows:
        expected=101 if variant=='candidate' and row['name'] in ['full-board','copper-neck'] else 0
        assert row['exit_code']==expected
        for suffix in ['stdout','stderr','resources']:
            assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        assert all(sha(record['retained'])==record['sha256'] for record in row['binaries'].values())
wasm=read(E/'wasm-validation-v1/result.json')
assert wasm['real_js_and_compiled_typescript_consumers_passed']
assert all(sha(path)==record['sha256'] for path,record in wasm['artifacts'].items())
probe=read(E/'committed-proposed-regressions-v1/result.json')
protocol=read(E/'committed-proposed-regressions-v1/protocol.json')
assert probe['complete'] and probe['source_and_libraries_unchanged'] and not probe['regressions_weakened']
assert sha(E/'committed_regressions_v1.py')==protocol['runner_sha256']
assert sha(protocol['source'])==protocol['source_sha256']
assert all(sha(row['path'])==row['sha256'] for row in protocol['qualified_libraries'].values())
assert sha(WORK/'committed-proposed-regressions-v1/native_linear_strokes')==probe['binary_sha256']
for row in read(E/'committed-proposed-regressions-v1/commands.json'):
    for suffix in ['stdout','stderr','resources']:
        assert sha(E/'committed-proposed-regressions-v1'/(row['name']+'.'+suffix))==row[suffix+'_sha256']

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


print('Hashing completed physical native disposition files:',len(paths),flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical=dict(pool.map(inspect,paths,buffersize=128))
assert prior.keys().isdisjoint(physical)
expectations={'schema_version':3,'kind':'new physical native disposition evidence; inherited historical audit remains failed/incomplete',
 'parent_expectations':previous['artifact_expectations'],'parent_expectations_sha256':previous['artifact_expectations_sha256'],
 'inherited_expected_artifact_paths':len(prior),'inherited_missing_artifact_paths':len(inherited_missing),
 'added_physical_artifacts_sha256':physical,'new_physical_artifacts_verified':len(physical),
 'local_tool_symlink_targets':symlinks,'expected_artifact_paths':len(prior)+len(physical),
 'inherited_artifacts_reverified':False,'complete_artifact_audit_passed':False,'historical_preservation_complete':False,
 'scope':'Completed candidate and committed source snapshots, native/WASM targets, extracted full fixtures, test binaries, metadata, logs and probes are hashed independently per path with stable stat/lstat checks. Parent expectations, known nested-view failures and inherited missing paths remain unchanged. No whole-history verification or zero-loss claim.'}
with gzip.open(E/'artifact-expectations-v1.json.gz','wt') as stream:json.dump(expectations,stream,sort_keys=True)
assert gz(E/'artifact-expectations-v1.json.gz')==expectations
work['updated_on']='2026-10-16'
work['pending_native_geometry']={
 'status':'rejected; disposition complete; no pending native edits',
 'scope':'The submitted 35-file native integration was evaluated against the current revision and rejected after both unchanged original workload gates failed.',
 'previous_pending_entry':work['pending_native_geometry'],
 'evidence':[PREFIX+'decision-v1.json',PREFIX+'progress-v1.md',PREFIX+'application-v1.json'],
 'runtime_admitted':False,'pending_native_paths_remaining':0,
 'candidate_files_preserved':35,'candidate_snapshot':PREFIX+'pending-source-v1/',
 'next_step':decision['next'][0]}
work['native_change_disposition']={
 'status':decision['status'],'main_parent_commit':BASE,'contract':PREFIX+'contract-v1.md',
 'progress':PREFIX+'progress-v1.md','decision':PREFIX+'decision-v1.json','checkpoint':PREFIX+'checkpoint-v1.json',
 'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz','implementation_sha256':{},
 'candidate_snapshot':PREFIX+'pending-source-v1/','candidate_files_preserved':35,
 'candidate_gates':decision['candidate']['gates'],
 'proposed_regressions_on_committed_libraries':probe['counts'],
 'known_gaps':decision['known_gap_interpretation'],
 'disposition_complete':True,'native_candidate_admitted':False,'pending_native_paths_remaining':0,
 'published_package_pipeline_executed':False,'full_plan_complete':False,'next':decision['next']}
updated=raw[:start]+json.dumps(work,indent=2).encode()+raw[end:]
ustart,uend,_=span(updated)
assert (updated[:ustart],updated[uend:])==(reference[:rstart],reference[rend:])
assert {key:value for key,value in json.loads(updated).items() if key!='current_work'}=={
    key:value for key,value in json.loads(reference).items() if key!='current_work'}
assert sha(ledger)==before
ledger.write_bytes(updated)
files={str(path.relative_to(ROOT)):sha(path) for path in E.rglob('*') if path.is_file() and '__pycache__' not in path.parts}
record={'status':decision['status'],'checkpoint_kind':'native candidate rejection; qualified committed sources restored; historical limitations retained',
 'previous_closure':str(previous_path.relative_to(ROOT)),'previous_closure_sha256':sha(previous_path),
 'evidence_files_sha256':files,'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz',
 'artifact_expectations_sha256':sha(E/'artifact-expectations-v1.json.gz'),'artifact_expectations_schema':3,
 'new_physical_artifacts_verified':len(physical),'inherited_expected_artifact_paths':len(prior),
 'inherited_missing_artifact_paths':len(inherited_missing),'complete_artifact_audit_passed':False,
 'historical_preservation_complete':False,'implementation_sha256':{},
 'inherited_implementation_sha256':inherited,'extra_unchanged_inputs_sha256':extra,
 'pending_native_sha256':{},'rejected_native_sha256':rejected,
 'restored_native_sha256':restored,'removed_added_paths':removed,
 'candidate_snapshot':PREFIX+'pending-source-v1/','live_source_dependency_pins_sha256':dependencies,
 'ledger_before_sha256':before,'ledger_after_sha256':sha(ledger),'only_current_work_changed':True,
 'python_sources_parse':True,'disposition_complete':True,
 'native_candidate_admitted':False,'native_runtime_admitted':False,'full_plan_complete':False}
(E/'checkpoint-v1.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
print(json.dumps({key:record[key] for key in ['new_physical_artifacts_verified','only_current_work_changed','disposition_complete']}),flush=True)
