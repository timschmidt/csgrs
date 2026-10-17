"""Apply only the qualified certified-bounds proof and preserve its raw evidence."""
import gzip, hashlib, json, shutil, subprocess
from pathlib import Path
WORK=Path(__file__).resolve().parent; ROOT=WORK.parents[2]
E=ROOT/'docs/evidence/2026-10-16-containment-bounds'
PARENT=ROOT/'docs/evidence/2026-10-16-containment-winding'
BASE='311d04fbbe5635a29fe7e754b838cccc4b8add53'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def gz(p):
    with gzip.open(p,'rt') as f:return json.load(f)
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True)
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
parent=read(PARENT/'checkpoint-v1.json')
assert all(sha(ROOT/p)==h for p,h in parent['evidence_files_sha256'].items())
assert sha(ROOT/'docs/properties-status.json')==parent['ledger_after_sha256']
for folder in ['candidate-validation-v1','wasm-validation-v2']:
    r=read(WORK/folder/'result.json');assert r['complete'] and not r['unexpected_exits']
assert read(WORK/'wasm-validation-v2/result.json')['real_js_and_compiled_typescript_consumers_passed']
cost=read(WORK/'cost-v2/result.json')
assert cost['complete'] and cost['baseline_and_candidate_all_correct_certified']
assert cost['all_preregistered_allocation_gates_passed']
baseline=read(WORK/'baseline-v1/result.json')
assert baseline['complete'] and baseline['native_test_exit_code']==baseline['wasm_test_exit_code']==0
assert '4 passed; 0 failed' in (WORK/'baseline-v1/native-tests.stdout').read_text()
text=(WORK/'candidate-validation-v1/containment-regressions.stdout').read_text()
assert all(s in text for s in ['4 passed; 0 failed','10 passed; 0 failed','15 passed; 0 failed'])
main=gz(WORK/'baseline-v1/current-input-hashes.json.gz')
qualified=gz(WORK/'candidate-preparation-v2/source-hashes.json.gz')
prep=read(WORK/'candidate-preparation-v2/result.json');changes=prep['changes']
assert set(changes)=={'crates/hypermesh/src/mesh.rs','tests/containment_bounds.rs','tests/wasm/containment_winding.cjs','tests/wasm/geometry_certainty.cjs'}
assert all(sha(ROOT/p)==h for p,h in main.items())
assert all(sha(WORK/'candidate-source-v2'/p)==h for p,h in qualified.items())
assert set(qualified)-set(main)=={'tests/containment_bounds.rs'}
assert all(not (ROOT/p).exists() for p in set(qualified)-set(main))
native=gz(WORK/'candidate-preparation-v1/source-hashes.json.gz')
assert native.keys()==qualified.keys()
assert {p for p in native if native[p]!=qualified[p]}=={'tests/wasm/geometry_certainty.cjs'}
assert all(sha(WORK/'candidate-source-v1'/p)==h for p,h in native.items())
assert read(WORK/'wasm-validation-v1/result.json')['unexpected_exits']==['real-consumers']
E.mkdir(exist_ok=False)
retained={}
folders=['prototype-v1','baseline-v1','supporting-fact-v1','candidate-preparation-v1','candidate-preparation-v2','candidate-validation-v1','wasm-validation-v1','wasm-validation-v2','cost-v2']
binary_names={'native-bounds-tests','control-probe','matrix-probe','baseline-control','candidate-control','baseline-matrix','candidate-matrix'}
for folder in folders:
    for source in sorted((WORK/folder).rglob('*')):
        assert not source.is_symlink()
        if not source.is_file() or source.name in binary_names:continue
        output=E/folder/source.relative_to(WORK/folder)
        output.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,output)
        assert sha(source)==sha(output)
        retained[str(output.relative_to(E))]={'original':str(source),'sha256':sha(source)}
for source in sorted(WORK.iterdir()):
    if source.is_file():
        output=E/source.name;shutil.copy2(source,output);assert sha(source)==sha(output)
        retained[source.name]={'original':str(source),'sha256':sha(source)}
shutil.copy2(WORK/'prototype-v1/contract-v1.md',E/'contract-v1.md')
for path,h in changes.items():
    dest=ROOT/path;dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_bytes((WORK/'candidate-source-v2'/path).read_bytes());assert sha(dest)==h
assert all(sha(ROOT/p)==h for p,h in qualified.items())
with gzip.open(E/'final-input-hashes-v1.json.gz','wt') as f:json.dump(qualified,f,sort_keys=True)
command=['cargo','fmt','--all','--','--check'];p=subprocess.run(command,cwd=ROOT,capture_output=True)
(E/'main-format.stdout').write_bytes(p.stdout);(E/'main-format.stderr').write_bytes(p.stderr)
patch=subprocess.check_output(['git','diff','--binary','--',*changes],cwd=ROOT)
for path in sorted(set(qualified)-set(main)):
    diff=subprocess.run(['git','diff','--no-index','--binary','--','/dev/null',path],cwd=ROOT,capture_output=True)
    assert diff.returncode==1;patch+=diff.stdout
(E/'applied.patch').write_bytes(patch)
dump(E/'application-v1.json',{'main_parent_commit':BASE,'implementation_sha256':changes,
    'final_source_inputs':len(qualified),'qualified_source_hashes_sha256':sha(WORK/'candidate-preparation-v2/source-hashes.json.gz'),
    'final_input_hashes_sha256':sha(E/'final-input-hashes-v1.json.gz'),
    'all_final_inputs_match_qualified_sources':True,'native_inputs_identical_between_v1_v2':True,'retained_completed_evidence':retained,
    'format_command':command,'format_exit_code':p.returncode,
    'format_stdout_sha256':sha(E/'main-format.stdout'),'format_stderr_sha256':sha(E/'main-format.stderr'),
    'pending_native_paths_remaining':0,'native_disposition_preserved':True,'full_plan_complete':False})
assert p.returncode==0
print('Applied four qualified files; main formatting passed',flush=True)
