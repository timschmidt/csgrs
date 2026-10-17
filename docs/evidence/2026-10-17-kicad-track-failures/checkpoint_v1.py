"""Pin the admitted loader while preserving original native-disposition history."""
import ast,concurrent.futures,gzip,hashlib,json,os,re,subprocess,sys,zipfile
from pathlib import Path
sys.dont_write_bytecode=True
W=Path(__file__).resolve().parent;ROOT=W.parents[2]
E=ROOT/'docs/evidence/2026-10-17-kicad-track-failures'
PARENT=ROOT/'docs/evidence/2026-10-16-containment-bounds';PREFIX=str(E.relative_to(ROOT))+'/'
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
assert len(prior)==1056218 and len(missing)==27093
application=read(E/'application-v1.json');decision=read(E/'decision-v1.json')
BASE=application['main_parent_commit'];assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
implementation=application['implementation_sha256'];assert implementation==decision['implementation_sha256']
assert decision['track_load_failures_admitted'] and not decision['general_exact_stroke_fidelity_claimed']
inherited=previous['inherited_implementation_sha256']|previous['implementation_sha256']
extra=previous['extra_unchanged_inputs_sha256'];dependencies=previous['live_source_dependency_pins_sha256']
live=previous['restored_native_sha256']|inherited|implementation
assert all(sha(ROOT/p)==h for pins in [live,extra,dependencies] for p,h in pins.items())
reintroduced={p:implementation[p] for p in previous['removed_added_paths'] if p in implementation}
assert set(reintroduced)=={'crates/hyperdrc/src/kicad/tracks.rs'}
assert all(not (ROOT/p).exists() for p in previous['removed_added_paths'] if p not in reintroduced)
assert all(sha(ROOT/previous['candidate_snapshot']/p)==h for p,h in previous['rejected_native_sha256'].items())
assert all(reintroduced[p]!=previous['rejected_native_sha256'][p] for p in reintroduced)
assert application['format_exit_code']==0 and application['all_final_inputs_match_qualified_sources']
final=gz(E/'final-input-hashes-v1.json.gz');assert final==gz(E/'candidate-preparation-v3/source-hashes.json.gz') and len(final)==2832
assert all(sha(W/'candidate-source-v3'/p)==h and sha(ROOT/p)==h for p,h in final.items())
assert final['docs/properties-status.json']==previous['ledger_after_sha256']
assert sha(E/'final-input-hashes-v1.json.gz')==application['final_input_hashes_sha256']
for n,r in application['retained_completed_evidence'].items():assert sha(E/n)==sha(r['original'])==r['sha256']
for folder,runner,common,expected in [
    ('baseline-validation-v1','baseline_v1.py','validation_common_v1.py',[]),
    ('candidate-validation-v1','candidate_v1.py','validation_common_v1.py',['native-tests','full-board','copper-neck','clippy']),
    ('candidate-validation-v2','candidate_v2.py','validation_common_v2.py',['native-tests','full-board','copper-neck','gerber','clippy']),
    ('candidate-validation-v3','candidate_v3.py','validation_common_v3.py',[])]:
    protocol=read(E/folder/'protocol.json');result=read(E/folder/'result.json');assert result['complete'] and result['unexpected_exits']==expected
    assert sha(E/runner)==protocol['runner_sha256'] and sha(E/common)==protocol['common_sha256']
    for row in read(E/folder/'commands.json'):
        for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        assert all(sha(r['retained'])==r['sha256'] for r in row['binaries'].values())
for folder in ['baseline-probe-v1','candidate-probe-v1','comparison-v1']:
    assert read(E/folder/'result.json')['complete']
    for row in read(E/folder/'commands.json'):
        for suffix in ['stdout','stderr','resources']:assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
for p,h in read(E/'comparison-v1/protocol.json')['read_only_library_and_cli_sha256'].items():assert sha(p)==h
fixture_checks=[]
for fixture in read(E/'baseline-preparation-v1/result.json')['fixtures']:
    relative=Path(fixture['archive']).relative_to(ROOT)
    for source in [ROOT,*[W/name for name in ['baseline-source-v1','candidate-source-v1','candidate-source-v2','candidate-source-v3']]]:
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
paths=[];links={}
for p in W.rglob('*'):
    assert not p.is_mount(),p
    if p.is_symlink():
        target=p.resolve(strict=True);assert target.is_file() and target.is_relative_to(W),p
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
expectations={'schema_version':3,'kind':'typed KiCad track failures and exact coincident disks; inherited audit incomplete',
    'parent_expectations':previous['artifact_expectations'],'parent_expectations_sha256':previous['artifact_expectations_sha256'],
    'inherited_expected_artifact_paths':len(prior),'inherited_missing_artifact_paths':len(missing),
    'added_physical_artifacts_sha256':physical,'new_physical_artifacts_verified':len(physical),'local_tool_symlink_targets':links,
    'expected_artifact_paths':len(prior)+len(physical),'inherited_artifacts_reverified':False,
    'complete_artifact_audit_passed':False,'historical_preservation_complete':False,
    'scope':'Every new physical path independently hashed with stable stat/lstat, including every failed candidate, snapshot, target, executable, source fixture and raw observation. No inode deduplication or whole-history/zero-loss claim.'}
with gzip.open(E/'artifact-expectations-v1.json.gz','wt') as f:json.dump(expectations,f,sort_keys=True)
current['updated_on']='2026-10-17'
current['kicad_track_failures']={
    'status':decision['status'],'main_parent_commit':BASE,'contract':PREFIX+'contract-v4.md',
    'decision':PREFIX+'decision-v1.json','progress':PREFIX+'progress-v1.md','checkpoint':PREFIX+'checkpoint-v1.json',
    'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz','implementation_sha256':implementation,
    'track_load_failures_admitted':True,'general_exact_stroke_fidelity_claimed':False,
    'new_public_tests':39,'typed_invalid_source_observations':96,'full_board_disks':5,
    'original_native_regressions':decision['original_native_regressions'],'comparison':PREFIX+'comparison-v1/result.json',
    'intentional_migration':decision['intentional_migration'],'preserved_failed_attempts':decision['preserved_failed_attempts'],
    'pending_native_paths_remaining':0,'native_disposition_preserved':True,
    'independently_reimplemented_archived_paths_sha256':reintroduced,
    'published_package_pipeline_executed':False,'full_plan_complete':False,
    'limitations':decision['limitations'],'next':decision['next']}
updated=raw[:start]+json.dumps(current,indent=2).encode()+raw[end:];us,ue,_=span(updated)
assert (updated[:us],updated[ue:])==(reference[:rs],reference[re_:])
assert {k:v for k,v in json.loads(updated).items() if k!='current_work'}=={k:v for k,v in json.loads(reference).items() if k!='current_work'}
assert sha(ledger)==before;ledger.write_bytes(updated)
files={str(p.relative_to(ROOT)):sha(p) for p in E.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
overrides={p:h for p,h in implementation.items() if p in previous['restored_native_sha256'] or p in reintroduced}
record={'status':decision['status'],'checkpoint_kind':'typed source-failure migration and native coincident-disk completion',
    'previous_closure':str(previous_path.relative_to(ROOT)),'previous_closure_sha256':sha(previous_path),
    'evidence_files_sha256':files,'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz',
    'artifact_expectations_sha256':sha(E/'artifact-expectations-v1.json.gz'),'artifact_expectations_schema':3,
    'new_physical_artifacts_verified':len(physical),'inherited_expected_artifact_paths':len(prior),'inherited_missing_artifact_paths':len(missing),
    'complete_artifact_audit_passed':False,'historical_preservation_complete':False,
    'implementation_sha256':implementation,'inherited_implementation_sha256':inherited,
    'extra_unchanged_inputs_sha256':extra,'pending_native_sha256':{},'rejected_native_sha256':previous['rejected_native_sha256'],
    'restored_native_sha256':previous['restored_native_sha256'],'removed_added_paths':previous['removed_added_paths'],
    'native_disposition_pin_scope':'Original restored hashes and removed paths are historical and retained unchanged. The independently admitted implementation overrides listed paths; every other restoration/removal remains live.',
    'live_native_disposition_overrides_sha256':overrides,'independently_reimplemented_archived_paths_sha256':reintroduced,
    'candidate_snapshot':previous['candidate_snapshot'],'live_source_dependency_pins_sha256':dependencies,
    'ledger_before_sha256':before,'ledger_after_sha256':sha(ledger),'only_current_work_changed':True,'python_sources_parse':True,
    'original_workload_archives_verified_after_qualification':fixture_checks,
    'track_load_failures_admitted':True,'general_exact_stroke_fidelity_claimed':False,'native_disposition_preserved':True,
    'native_candidate_admitted':False,'native_runtime_admitted':False,'full_plan_complete':False}
(E/'checkpoint-v1.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
print(json.dumps({'new_physical_artifacts_verified':len(physical),'only_current_work_changed':True,'track_load_failures_admitted':True}),flush=True)
