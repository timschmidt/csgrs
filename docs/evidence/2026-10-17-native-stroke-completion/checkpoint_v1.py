"""Pin independently admitted archived components without rewriting disposition history."""
import ast,concurrent.futures,gzip,hashlib,json,os,re,subprocess,sys,zipfile
from pathlib import Path
sys.dont_write_bytecode=True
W=Path(__file__).resolve().parent;ROOT=W.parents[2]
E=ROOT/'docs/evidence/2026-10-17-native-stroke-completion'
PARENT=ROOT/'docs/evidence/2026-10-17-kicad-track-failures';PREFIX=str(E.relative_to(ROOT))+'/'
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
assert len(prior)==1081965 and len(missing)==27093
application=read(E/'application-v1.json');decision=read(E/'decision-v1.json')
BASE=application['main_parent_commit'];assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
implementation=application['implementation_sha256'];assert implementation==decision['implementation_sha256']
assert decision['single_line_kernel_admitted'] and not decision['general_exact_kicad_stroke_fidelity_claimed']
inherited=previous['inherited_implementation_sha256']|previous['implementation_sha256']
extra=previous['extra_unchanged_inputs_sha256'];dependencies=previous['live_source_dependency_pins_sha256']
live=previous['restored_native_sha256']|inherited|implementation
assert all(sha(ROOT/p)==h for pins in [live,extra,dependencies] for p,h in pins.items())
reused=application['independently_admitted_archived_components_sha256']
assert set(reused)=={'crates/hypercurve/src/bezier_region/linear_stroke.rs','crates/hypercurve/tests/native_linear_stroke.rs'}
assert all(reused[p]==implementation[p]==previous['rejected_native_sha256'][p] for p in reused)
assert reused==decision['independently_admitted_archived_components_sha256']
assert all(not (ROOT/p).exists() for p in previous['removed_added_paths'] if p not in (inherited|implementation))
assert all(sha(ROOT/previous['candidate_snapshot']/p)==h for p,h in previous['rejected_native_sha256'].items())
assert application['format_exit_code']==0 and application['all_final_inputs_match_qualified_sources']
final=gz(E/'final-input-hashes-v1.json.gz');assert final==gz(E/'candidate-preparation-v6/source-hashes.json.gz') and len(final)==2835
assert all(sha(W/'candidate-source-v6'/p)==h and sha(ROOT/p)==h for p,h in final.items())
assert final['docs/properties-status.json']==previous['ledger_after_sha256']
assert sha(E/'final-input-hashes-v1.json.gz')==application['final_input_hashes_sha256']
for n,r in application['retained_completed_evidence'].items():assert sha(E/n)==sha(r['original'])==r['sha256']
for r in application['physical_executables_not_duplicated_in_git']:assert sha(r['path'])==r['sha256']
for r in application['physical_project_inputs_not_duplicated_in_git']:assert sha(r['path'])==r['sha256']
for r in application['physical_libraries_not_duplicated_in_git']:assert sha(r['path'])==r['sha256']
for version,expected in [(1,['original-curve-regressions']),(2,[]),(4,['curve-all','circuit-all','clippy'])]:
    folder=f'candidate-validation-v{version}';runner=f'candidate_v{version}.py';common=f'validation_common_v{version}.py'
    protocol=read(E/folder/'protocol.json');result=read(E/folder/'result.json');assert result['complete'] and result['unexpected_exits']==expected
    assert sha(E/runner)==protocol['runner_sha256'] and sha(E/common)==protocol['common_sha256']
    for row in read(E/folder/'commands.json'):
        for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        assert all(sha(r['retained'])==r['sha256'] for r in row['binaries'].values())
for folder in ['reproduction-v1','reproduction-v2','direct-observation-v2','contracts-observation-v1','contracts-observation-v2',
               'baseline-contracts-v1','circuit-baseline-v1','cost-preflight-v1','cost-preflight-v2','comparison-v1']:
    for row in read(E/folder/'commands.json'):
        for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
for folder in ['direct-observation-v2','contracts-observation-v2','baseline-contracts-v1','cost-preflight-v2','comparison-v1']:
    assert read(E/folder/'result.json')['complete']
requalified=read(E/'consumer-validation-v1/result.json');assert requalified['complete'] and requalified['unexpected_exits']==['circuit-all']
protocol=read(E/'consumer-validation-v1/protocol.json')
assert sha(E/'consumer_v1.py')==protocol['runner_sha256'] and sha(E/'consumer_common_v1.py')==protocol['common_sha256']
for row in read(E/'consumer-validation-v1/commands.json'):
    for suffix in ['stdout','stderr','resources']:assert sha(E/'consumer-validation-v1'/(row['name']+'.'+suffix))==row[suffix+'_sha256']
    assert all(sha(r['retained'])==r['sha256'] for r in row['binaries'].values())
for folder,runner,common,expected in [('ui-validation-v1','ui_v4.py','ui_common_v3.py',['ui-tests','ui-clippy','ui-wasm']),('ui-validation-v2','ui_v5.py','ui_common_v4.py',[])]:
    ui=read(E/folder/'result.json');assert ui['complete'] and ui['unexpected_exits']==expected
    protocol=read(E/folder/'protocol.json')
    assert sha(E/runner)==protocol['runner_sha256'] and sha(E/common)==protocol['common_sha256']
    for row in read(E/folder/'commands.json'):
        for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        assert all(sha(r['retained'])==r['sha256'] for r in row['binaries'].values())
isolation=read(E/'ui-isolation-v1/result.json');assert isolation['all_final_v6_inputs_byte_identical'] and isolation['source_inputs']==2835
isolated_source=Path(isolation['source']);assert str(isolated_source)==application['physical_isolated_demo_source_root']
assert sha(E/'ui-isolation-v1/result.json')==read(E/'ui-validation-v2/protocol.json')['source_isolation_result_sha256']
assert all(sha(isolated_source/p)==h for p,h in final.items())
for p,h in read(E/'ui-isolation-v1/protocol.json')['input_sha256'].items():assert sha(p)==h
for row in read(E/'ui-isolation-v1/commands.json'):
    assert row['exit_code']==0
    for suffix in ['stdout','stderr','resources']:assert sha(E/'ui-isolation-v1'/(row['name']+'.'+suffix))==row[suffix+'_sha256']
assert not read(E/'comparison-v1/result.json')['candidate_incomplete_cases']
assert not any(read(E/'comparison-v1/result.json')['raw_full_board_report_differences'].values())
assert read(E/'structure-metrics-v1/result.json')['complete']
cost_review=read(E/'cost-review-v1.json');assert cost_review['complete']
assert cost_review['runner_sha256']==sha(E/'review_cost_v1.py')
assert cost_review['comparison_sha256']==sha(E/'comparison-v1/result.json')
assert cost_review['structure_sha256']==sha(E/'structure-metrics-v1/result.json')
structure_protocol=read(E/'structure-metrics-v1/protocol.json')
assert sha(E/'measure_structure_v1.py')==structure_protocol['runner_sha256']
assert sha(E/'comparison-v1/result.json')==structure_protocol['comparison_sha256']
for p,r in structure_protocol['format_sources'].items():assert sha(p)==sha(E/'structure-metrics-v1'/r['retained'])==r['sha256']
licenses=read(E/'format-source-licenses-v1/result.json');assert licenses['complete']
assert licenses['runner_sha256']==sha(E/'retain_format_licenses_v1.py')
for name,r in licenses['retained'].items():assert sha(r['source'])==sha(E/'format-source-licenses-v1'/name)==r['sha256']
assert read(E/'project-reports-v1/result.json')['all_report_values_identical']
project_protocol=read(E/'project-reports-v1/protocol.json')
for p,h in project_protocol['input_sha256'].items():assert sha(p)==h
for p,r in project_protocol['project_files'].items():assert sha(p)==r['sha256']
for row in read(E/'project-reports-v1/commands.json'):
    for suffix in ['stdout','stderr','resources']:assert sha(E/'project-reports-v1'/(row['name']+'.'+suffix))==row[suffix+'_sha256']
for p,h in read(E/'comparison-v1/protocol.json')['read_only_library_and_cli_sha256'].items():assert sha(p)==h
assert read(E/'circuit-baseline-v1/result.json')['baseline_compile_failure_reproduced']
reuse=read(E/'v4-qualification-reuse-v2/result.json');assert reuse['complete']
assert sha(E/'verify_qualification_reuse_v2.py')==reuse['runner_sha256']
assert sha(E/'v4-qualification-reuse-v2/metadata.stdout')==reuse['metadata_stdout_sha256']
assert sha(E/'v4-qualification-reuse-v2/metadata.stderr')==reuse['metadata_stderr_sha256']
assert len(reuse['reused_completed_observations'])==9
assert read(E/'qualification-tail-v4/commands.json')[-1]['exit_code']==101
assert read(E/'qualification-tail-v5/result.json')['complete']
for p,h in read(E/'qualification-tail-v5/protocol.json')['input_sha256'].items():assert sha(p)==h
metadata=read(E/'metadata-migration-v2/result.json');assert metadata['complete'] and metadata['geometry_findings_and_other_artifacts_unchanged']
assert sha(E/'verify_metadata_migration_v2.py')==metadata['runner_sha256']
assert sha(E/'artifact-mismatches-v1/result.json')==metadata['decoded_differences_sha256']
assert read(E/'snapshot-requalification-v1/result.json')['all_seven_prior_failures_replayed_successfully']
replay=read(E/'snapshot-requalification-v1/protocol.json')
for p,h in replay['input_sha256'].items():assert sha(p)==h
for p,h in replay['read_only_library_sha256'].items():assert sha(p)==h
for row in read(E/'snapshot-requalification-v1/commands.json'):
    assert row['exit_code']==0
    for suffix in ['stdout','stderr','resources']:assert sha(E/'snapshot-requalification-v1'/(row['name']+'.'+suffix))==row[suffix+'_sha256']
retention=read(E/'snapshot-replay-libraries-v1/result.json');assert retention['complete']
assert sha(E/'retain_circuit_libraries_v1.py')==retention['runner_sha256']
for r in retention['retained'].values():assert sha(r['retained'])==r['sha256']
fixture_checks=[]
for fixture in read(ROOT/'target/properties/kicad-track-failures/baseline-preparation-v1/result.json')['fixtures']:
    relative=Path(fixture['archive']).relative_to(ROOT)
    for source in [ROOT,*[W/name for name in ['baseline-source-v1','candidate-source-v1','candidate-source-v2','candidate-source-v3','candidate-source-v4','candidate-source-v5','candidate-source-v6']]]:
        archive=source/relative;assert sha(archive)==fixture['sha256']
        with zipfile.ZipFile(archive) as z:payload=z.read(fixture['entry'])
        assert len(payload)==fixture['entry_bytes'] and hashlib.sha256(payload).hexdigest()==fixture['entry_sha256']
        fixture_checks.append({'archive':str(archive),'sha256':fixture['sha256'],'entry':fixture['entry'],
            'entry_bytes':len(payload),'entry_sha256':fixture['entry_sha256']})
for p in E.rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
ledger=ROOT/'docs/properties-status.json';raw=ledger.read_bytes();before=sha(ledger);assert before==previous['ledger_after_sha256']
reference=subprocess.check_output(['git','show','9025de62:docs/properties-status.json'],cwd=ROOT)
start,end,current=span(raw);rs,re_,_=span(reference)
assert (raw[:start],raw[end:])==(reference[:rs],reference[re_:])
paths=[];links={};physical_roots=[W,isolated_source]
for physical_root in physical_roots:
    for p in physical_root.rglob('*'):
        assert not p.is_mount(),p
        if p.is_symlink():
            target=p.resolve(strict=True);assert target.is_file() and any(target.is_relative_to(root) for root in physical_roots),p
            links[str(p)]={'link_text':os.readlink(p),'resolved_target':str(target)}
        if p.is_file():paths.append(p)
def inspect(p):
    before,link_before=p.stat(),p.lstat();digest=sha(p);after,link_after=p.stat(),p.lstat()
    for a,b in [(before,after),(link_before,link_after)]:assert all(getattr(a,k)==getattr(b,k) for k in ['st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns']),p
    if str(p) in links:assert os.readlink(p)==links[str(p)]['link_text'] and str(p.resolve(strict=True))==links[str(p)]['resolved_target']
    return str(p),digest
print('Hashing every completed physical path:',len(paths),flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:physical=dict(pool.map(inspect,paths,buffersize=128))
assert prior.keys().isdisjoint(physical)
expectations={'schema_version':3,'kind':'independently admitted exact single-line stroke component; inherited audit incomplete',
    'parent_expectations':previous['artifact_expectations'],'parent_expectations_sha256':previous['artifact_expectations_sha256'],
    'inherited_expected_artifact_paths':len(prior),'inherited_missing_artifact_paths':len(missing),
    'added_physical_artifacts_sha256':physical,'new_physical_artifacts_verified':len(physical),'local_tool_symlink_targets':links,
    'physical_roots':[str(p) for p in physical_roots],
    'expected_artifact_paths':len(prior)+len(physical),'inherited_artifacts_reverified':False,
    'complete_artifact_audit_passed':False,'historical_preservation_complete':False,
    'scope':'Every new physical path independently hashed with stable stat/lstat, including failed attempts, snapshots, targets, executables, source fixtures and raw observations. No inode deduplication or whole-history/zero-loss claim.'}
with gzip.open(E/'artifact-expectations-v1.json.gz','wt') as f:json.dump(expectations,f,sort_keys=True)
current['updated_on']='2026-10-17'
current['native_stroke_completion']={
    'status':decision['status'],'main_parent_commit':BASE,'contract':PREFIX+'contract-v3.md',
    'decision':PREFIX+'decision-v1.json','progress':PREFIX+'progress-v1.md','checkpoint':PREFIX+'checkpoint-v1.json',
    'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz','implementation_sha256':implementation,
    'single_line_kernel_admitted':True,'general_exact_kicad_stroke_fidelity_claimed':False,
    'original_curve_regressions':decision['original_curve_regressions'],'added_contract_regressions':decision['added_contract_regressions'],
    'typed_text_consumer_migration':decision['typed_text_consumer_migration'],
    'readiness_metadata_migration':decision['readiness_metadata_migration'],
    'original_drc_reproduction':decision['original_drc_reproduction'],'comparison':PREFIX+'comparison-v1/result.json',
    'intentional_migration':decision['intentional_migration'],'preserved_failed_attempts':decision['preserved_failed_attempts'],
    'pending_native_paths_remaining':0,'native_disposition_preserved':True,
    'independently_admitted_archived_components_sha256':reused,
    'full_archived_integration_admitted':False,'published_package_pipeline_executed':False,'full_plan_complete':False,
    'limitations':decision['limitations'],'next':decision['next']}
updated=raw[:start]+json.dumps(current,indent=2).encode()+raw[end:];us,ue,_=span(updated)
assert (updated[:us],updated[ue:])==(reference[:rs],reference[re_:])
assert {k:v for k,v in json.loads(updated).items() if k!='current_work'}=={k:v for k,v in json.loads(reference).items() if k!='current_work'}
assert sha(ledger)==before;ledger.write_bytes(updated)
files={str(p.relative_to(ROOT)):sha(p) for p in E.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
overrides={p:h for p,h in (inherited|implementation).items() if p in previous['restored_native_sha256'] or p in previous['removed_added_paths']}
record={'status':decision['status'],'checkpoint_kind':'independent admission of exact native single-line stroke components',
    'previous_closure':str(previous_path.relative_to(ROOT)),'previous_closure_sha256':sha(previous_path),
    'evidence_files_sha256':files,'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz',
    'artifact_expectations_sha256':sha(E/'artifact-expectations-v1.json.gz'),'artifact_expectations_schema':3,
    'new_physical_artifacts_verified':len(physical),'inherited_expected_artifact_paths':len(prior),'inherited_missing_artifact_paths':len(missing),
    'complete_artifact_audit_passed':False,'historical_preservation_complete':False,
    'implementation_sha256':implementation,'inherited_implementation_sha256':inherited,
    'extra_unchanged_inputs_sha256':extra,'pending_native_sha256':{},'rejected_native_sha256':previous['rejected_native_sha256'],
    'restored_native_sha256':previous['restored_native_sha256'],'removed_added_paths':previous['removed_added_paths'],
    'native_disposition_pin_scope':'Original restored hashes and removed paths are historical and retained unchanged. Independently admitted implementation overrides are live; every other restoration/removal remains enforced.',
    'live_native_disposition_overrides_sha256':overrides,
    'independently_reimplemented_archived_paths_sha256':previous['independently_reimplemented_archived_paths_sha256'],
    'independently_admitted_archived_components_sha256':reused,
    'candidate_snapshot':previous['candidate_snapshot'],'live_source_dependency_pins_sha256':dependencies,
    'ledger_before_sha256':before,'ledger_after_sha256':sha(ledger),'only_current_work_changed':True,'python_sources_parse':True,
    'original_workload_archives_verified_after_qualification':fixture_checks,
    'single_line_kernel_admitted':True,'typed_text_consumer_migration':True,
    'general_exact_kicad_stroke_fidelity_claimed':False,'native_disposition_preserved':True,
    'native_candidate_admitted':False,'native_runtime_admitted':False,'full_archived_integration_admitted':False,'full_plan_complete':False}
(E/'checkpoint-v1.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
print(json.dumps({'new_physical_artifacts_verified':len(physical),'only_current_work_changed':True,'single_line_kernel_admitted':True}),flush=True)
