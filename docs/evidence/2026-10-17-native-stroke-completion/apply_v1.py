"""Apply the independently qualified single-line component and retain attempts."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2]
E=ROOT/'docs/evidence/2026-10-17-native-stroke-completion'
PARENT=ROOT/'docs/evidence/2026-10-17-kicad-track-failures'
BASE='b137381247a03342ce5b6184710475fcc22c9d63'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def gz(p):
    with gzip.open(p,'rt') as f:return json.load(f)
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True);subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
parent=read(PARENT/'checkpoint-v1.json')
assert all(sha(ROOT/p)==h for p,h in parent['evidence_files_sha256'].items())
assert sha(ROOT/'docs/properties-status.json')==parent['ledger_after_sha256']
for version,expected in [(1,['original-curve-regressions']),(2,[]),(4,['curve-all','circuit-all','clippy'])]:
    result=read(W/f'candidate-validation-v{version}/result.json')
    assert result['complete'] and result['unexpected_exits']==expected
assert read(W/'contracts-observation-v2/result.json')['contract_exit_code']==0
assert read(W/'baseline-contracts-v1/result.json')['complete']
assert read(W/'ui-validation-v1/result.json')['unexpected_exits']==['ui-tests','ui-clippy','ui-wasm']
assert read(W/'ui-validation-v2/result.json')['unexpected_exits']==[]
assert read(W/'ui-isolation-v1/result.json')['all_final_v6_inputs_byte_identical']
assert read(W/'qualification-tail-v5/result.json')['complete']
assert read(W/'consumer-validation-v1/result.json')['unexpected_exits']==['circuit-all']
assert read(W/'snapshot-requalification-v1/result.json')['complete']
assert read(W/'metadata-migration-v2/result.json')['complete']
assert read(W/'circuit-baseline-v1/result.json')['baseline_compile_failure_reproduced']
assert read(W/'v4-qualification-reuse-v2/result.json')['complete']
assert read(W/'structure-metrics-v1/result.json')['complete']
assert read(W/'cost-review-v1.json')['complete']
assert read(W/'project-reports-v1/result.json')['all_report_values_identical']
cost=read(W/'comparison-v1/result.json')
assert cost['complete'] and not cost['candidate_incomplete_cases'] and cost['exact_scalar_serialization_errors']==0
assert all(not rows for rows in cost['raw_full_board_report_differences'].values())
assert len(cost['all_samples'])==240 and len(cost['paired_summary'])==24
before=gz(W/'candidate-preparation-v1/baseline-source-hashes.json.gz')
final=gz(W/'candidate-preparation-v6/source-hashes.json.gz')
changes=read(W/'candidate-preparation-v6/result.json')['changes']
assert len(before)==2832 and len(final)==2835 and len(changes)==14
assert all(sha(ROOT/p)==h for p,h in before.items())
assert all(sha(W/'candidate-source-v6'/p)==h for p,h in final.items())
added=set(final)-set(before)
assert added=={'crates/hypercurve/src/bezier_region/linear_stroke.rs','crates/hypercurve/tests/native_linear_stroke.rs','crates/hypercurve/tests/native_linear_stroke_contracts.rs'}
assert all(not (ROOT/p).exists() for p in added)
reused={p:changes[p] for p in added if p in parent['rejected_native_sha256']}
assert len(reused)==2 and all(h==parent['rejected_native_sha256'][p] for p,h in reused.items())
E.mkdir(exist_ok=False);retained={};excluded=[];physical_project_inputs=[];physical_libraries=[]
folders=['reproduction-v1','reproduction-v2','probe-source-v1','probe-source-v2','direct-observation-v2',
    'candidate-preparation-v1','candidate-preparation-v2','candidate-preparation-v3','candidate-preparation-v4','candidate-preparation-v5','candidate-preparation-v6',
    'candidate-validation-v1','candidate-validation-v2','candidate-validation-v4',
    'contracts-observation-v1','contracts-observation-v2','baseline-contracts-v1',
    'cost-preflight-v1','cost-preflight-v2','ui-validation-v1','ui-validation-v2','ui-isolation-v1','comparison-v1','qualification-tail-v1','qualification-tail-v2','qualification-tail-v3','qualification-tail-v4','qualification-tail-v5','consumer-validation-v1','circuit-baseline-v1','v4-qualification-reuse-v1','v4-qualification-reuse-v2','artifact-mismatches-v1','metadata-migration-v1','metadata-migration-v2','snapshot-replay-libraries-v1','snapshot-requalification-v1','structure-metrics-v1','structure-preflight-v1','format-source-licenses-v1','project-reports-v1']
for folder in folders:
    assert (W/folder).is_dir(),folder
    for source in sorted((W/folder).rglob('*')):
        assert not source.is_symlink()
        if not source.is_file():continue
        if folder=='snapshot-replay-libraries-v1' and source.is_relative_to(W/folder/'libraries'):
            physical_libraries.append({'path':str(source),'sha256':sha(source),'reason':'Frozen completed circuit-build library retained physically for read-only snapshot replay'});continue
        if folder=='project-reports-v1' and source.is_relative_to(W/folder/'inputs'):
            physical_project_inputs.append({'path':str(source),'sha256':sha(source),'reason':'Complete original project extraction retained physically; original archive and full file inventory are pinned'});continue
        with source.open('rb') as f:magic=f.read(4)
        if magic==b'\x7fELF':
            excluded.append({'path':str(source),'sha256':sha(source),'reason':'Physical executable retained and independently audited in the work directory'});continue
        out=E/folder/source.relative_to(W/folder);out.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,out)
        assert sha(out)==sha(source);retained[str(out.relative_to(E))]={'original':str(source),'sha256':sha(source)}
for source in sorted(W.iterdir()):
    if source.is_file():
        out=E/source.name;shutil.copy2(source,out);assert sha(out)==sha(source)
        retained[source.name]={'original':str(source),'sha256':sha(source)}
for path,h in changes.items():
    dest=ROOT/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/'candidate-source-v6'/path,dest);assert sha(dest)==h
assert all(sha(ROOT/p)==h for p,h in final.items())
with gzip.open(E/'final-input-hashes-v1.json.gz','wt') as f:json.dump(final,f,sort_keys=True)
command=['cargo','fmt','--all','--','--check'];result=subprocess.run(command,cwd=ROOT,capture_output=True)
(E/'main-format.stdout').write_bytes(result.stdout);(E/'main-format.stderr').write_bytes(result.stderr)
patch=subprocess.check_output(['git','diff','--binary','--',*changes],cwd=ROOT)
for path in sorted(added):
    diff=subprocess.run(['git','diff','--no-index','--binary','--','/dev/null',path],cwd=ROOT,capture_output=True)
    assert diff.returncode==1;patch+=diff.stdout
(E/'applied.patch').write_bytes(patch)
dump(E/'application-v1.json',{'main_parent_commit':BASE,'implementation_sha256':changes,'final_source_inputs':len(final),
    'all_final_inputs_match_qualified_sources':True,'final_input_hashes_sha256':sha(E/'final-input-hashes-v1.json.gz'),
    'retained_completed_evidence':retained,'physical_executables_not_duplicated_in_git':excluded,
    'physical_project_inputs_not_duplicated_in_git':physical_project_inputs,
    'physical_libraries_not_duplicated_in_git':physical_libraries,
    'physical_isolated_demo_source_root':read(W/'ui-isolation-v1/result.json')['source'],
    'format_command':command,'format_exit_code':result.returncode,
    'format_stdout_sha256':sha(E/'main-format.stdout'),'format_stderr_sha256':sha(E/'main-format.stderr'),
    'independently_admitted_archived_components_sha256':reused,
    'unchanged_dependency_qualification':'v4-qualification-reuse-v2/result.json',
    'final_consumer_qualification':{'native_attempt':'consumer-validation-v1/result.json','artifact_replays':'snapshot-requalification-v1/result.json','original_native_suite_exit_code':101},
    'typed_text_consumer_migration':True,
    'readiness_metadata_migration':'metadata-migration-v2/result.json',
    'snapshot_requalification':'snapshot-requalification-v1/result.json',
    'pending_native_paths_remaining':0,'native_disposition_preserved':True,
    'full_archived_integration_admitted':False,'general_exact_kicad_stroke_fidelity_claimed':False,'full_plan_complete':False})
assert result.returncode==0
print('Applied fourteen qualified files; main formatting passed',flush=True)
