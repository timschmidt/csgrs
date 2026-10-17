"""Apply the five qualified files, preserving every attempt and raw observation."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
WORK=Path(__file__).resolve().parent;ROOT=WORK.parents[2]
E=ROOT/'docs/evidence/2026-10-16-containment-input-validation'
PARENT=ROOT/'docs/evidence/2026-10-16-wasm-mesh-failures'
BASE='c9690a7cbd03c79e485bd7670ca10fe68bc5e45e'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def gz(p):
 with gzip.open(p,'rt') as f:return json.load(f)
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True)
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
assert not list(E.iterdir())
parent=read(PARENT/'checkpoint-v1.json')
assert all(sha(ROOT/p)==h for p,h in parent['evidence_files_sha256'].items())
assert sha(ROOT/'docs/properties-status.json')==parent['ledger_after_sha256']
for folder in ['candidate-validation-v2','wasm-validation-v2']:
 r=read(WORK/folder/'result.json');assert r['complete'] and not r['unexpected_exits']
assert read(WORK/'wasm-validation-v2/result.json')['real_js_and_compiled_typescript_consumers_passed']
assert read(WORK/'cost-v1/result.json')['all_expected_values_and_certainties_match']
assert read(WORK/'wasm-reproduction-v1/result.json')['expected_failure_reproduced']
assert '4 passed; 6 failed' in (WORK/'baseline-validation-v1/containment-inputs.stdout').read_text()
assert '10 passed; 0 failed' in (WORK/'candidate-validation-v2/containment-inputs.stdout').read_text()
main=gz(WORK/'baseline-preparation-v1/main-input-hashes.json.gz')
qualified=gz(WORK/'candidate-preparation-v2/source-hashes.json.gz')
prep=read(WORK/'candidate-preparation-v2/result.json');changes=prep['changes']
assert set(changes)=={'crates/hypermesh/src/mesh.rs','src/solid.rs','src/wasm/mesh_js.rs','tests/containment_inputs.rs','tests/wasm/mesh_errors.cjs'}
assert all(sha(ROOT/p)==h for p,h in main.items())
assert all(sha(WORK/'candidate-source-v2'/p)==h for p,h in qualified.items())
assert set(qualified)-set(main)=={'tests/containment_inputs.rs'}
assert not (ROOT/'tests/containment_inputs.rs').exists()
retained={}
folders=['prototype-v1','baseline-preparation-v1','candidate-preparation-v1','candidate-preparation-v2','baseline-validation-v1','wasm-reproduction-v1','winding-diagnostic-v1','winding-followup-v1','candidate-validation-v1','candidate-validation-v2','wasm-validation-v2','cost-v1']
for folder in folders:
 for source in sorted((WORK/folder).rglob('*')):
  assert not source.is_symlink()
  if not source.is_file() or source.name.endswith('-probe'):continue
  output=E/folder/source.relative_to(WORK/folder);output.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,output)
  assert sha(source)==sha(output);retained[str(output.relative_to(E))]={'original':str(source),'sha256':sha(source)}
for source in sorted(WORK.iterdir()):
 if source.is_file():
  output=E/source.name;shutil.copy2(source,output);assert sha(source)==sha(output)
  retained[source.name]={'original':str(source),'sha256':sha(source)}
for path,h in changes.items():
 dest=ROOT/path;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes((WORK/'candidate-source-v2'/path).read_bytes());assert sha(dest)==h
assert all(sha(ROOT/p)==h for p,h in qualified.items())
with gzip.open(E/'final-input-hashes-v1.json.gz','wt') as f:json.dump(qualified,f,sort_keys=True)
command=['cargo','fmt','--all','--','--check'];p=subprocess.run(command,cwd=ROOT,capture_output=True)
(E/'main-format.stdout').write_bytes(p.stdout);(E/'main-format.stderr').write_bytes(p.stderr)
(E/'applied.patch').write_bytes(subprocess.check_output(['git','diff','--binary','--',*changes],cwd=ROOT))
dump(E/'application-v1.json',{'main_parent_commit':BASE,'implementation_sha256':changes,'final_source_inputs':len(qualified),'qualified_source_hashes_sha256':sha(WORK/'candidate-preparation-v2/source-hashes.json.gz'),'final_input_hashes_sha256':sha(E/'final-input-hashes-v1.json.gz'),'all_final_inputs_match_qualified_sources':True,'retained_completed_evidence':retained,'format_command':command,'format_exit_code':p.returncode,'format_stdout_sha256':sha(E/'main-format.stdout'),'format_stderr_sha256':sha(E/'main-format.stderr'),'pending_native_paths_remaining':0,'native_disposition_preserved':True,'full_plan_complete':False})
assert p.returncode==0
print('Applied 5 qualified files; main formatting passed',flush=True)
