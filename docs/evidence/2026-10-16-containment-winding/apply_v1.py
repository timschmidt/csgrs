"""Apply only the qualified nonzero-winding correction; retain every failed attempt."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
WORK=Path(__file__).resolve().parent;ROOT=WORK.parents[2]
E=ROOT/'docs/evidence/2026-10-16-containment-winding'
PARENT=ROOT/'docs/evidence/2026-10-16-containment-input-validation'
BASE='d4eee9a675fa51d84eb0eb29503dd802bdcdbd19'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def gz(p):
 with gzip.open(p,'rt') as f:return json.load(f)
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True)
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
E.mkdir(exist_ok=False)
parent=read(PARENT/'checkpoint-v1.json')
assert all(sha(ROOT/p)==h for p,h in parent['evidence_files_sha256'].items())
assert sha(ROOT/'docs/properties-status.json')==parent['ledger_after_sha256']
for folder in ['candidate-validation-v3','wasm-validation-v3','ffi-validation-v3']:
 r=read(WORK/folder/'result.json');assert r['complete'] and not r['unexpected_exits']
assert read(WORK/'wasm-validation-v3/result.json')['real_js_and_compiled_typescript_consumers_passed']
assert read(WORK/'cost-v1/result.json')['candidate_matrix_all_certified']
assert read(WORK/'baseline-v6/result.json')['complete']
assert '5 passed; 10 failed' in (WORK/'baseline-v6/native-tests.stdout').read_text()
assert '15 passed; 0 failed' in (WORK/'candidate-validation-v3/containment-regressions.stdout').read_text()
main=gz(WORK/'baseline-v6/current-input-hashes.json.gz')
qualified=gz(WORK/'candidate-preparation-v3/source-hashes.json.gz')
prep=read(WORK/'candidate-preparation-v3/result.json');changes=prep['changes']
assert set(changes)=={'crates/hypermesh/src/mesh.rs','crates/hypermesh/src/winding.rs','crates/hypermesh/src/minkowski.rs','crates/hypermesh/src/surface_arrangement.rs','src/solid.rs','src/wasm/mesh_js.rs','scripts/test-wasm-bindings.sh','tests/containment_winding.rs','tests/wasm/containment_winding.cjs'}
assert all(sha(ROOT/p)==h for p,h in main.items())
assert all(sha(WORK/'candidate-source-v3'/p)==h for p,h in qualified.items())
assert set(qualified)-set(main)=={'tests/containment_winding.rs','tests/wasm/containment_winding.cjs'}
assert all(not (ROOT/p).exists() for p in set(qualified)-set(main))
retained={}
folders=['prototype-v1','prototype-v2','prototype-v3','prototype-v4','baseline-v1','baseline-v2','baseline-v3','baseline-v4','baseline-v5','baseline-v6','candidate-preparation-v1','candidate-preparation-v2','candidate-preparation-v3','candidate-validation-v1','candidate-validation-v2','candidate-validation-v3','wasm-validation-v1','wasm-validation-v2','wasm-validation-v3','ffi-probes-v1','ffi-validation-v3','cost-v1']
for folder in folders:
 for source in sorted((WORK/folder).rglob('*')):
  assert not source.is_symlink()
  if not source.is_file() or source.name in {'native-winding-tests','baseline-control','candidate-control','baseline-matrix','candidate-matrix'}:continue
  output=E/folder/source.relative_to(WORK/folder);output.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,output)
  assert sha(source)==sha(output);retained[str(output.relative_to(E))]={'original':str(source),'sha256':sha(source)}
for source in sorted(WORK.iterdir()):
 if source.is_file():
  output=E/source.name;shutil.copy2(source,output);assert sha(source)==sha(output)
  retained[source.name]={'original':str(source),'sha256':sha(source)}
shutil.copy2(WORK/'prototype-v4/contract-v1.md',E/'contract-v1.md')
for path,h in changes.items():
 dest=ROOT/path;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes((WORK/'candidate-source-v3'/path).read_bytes());assert sha(dest)==h
assert all(sha(ROOT/p)==h for p,h in qualified.items())
with gzip.open(E/'final-input-hashes-v1.json.gz','wt') as f:json.dump(qualified,f,sort_keys=True)
command=['cargo','fmt','--all','--','--check'];p=subprocess.run(command,cwd=ROOT,capture_output=True)
(E/'main-format.stdout').write_bytes(p.stdout);(E/'main-format.stderr').write_bytes(p.stderr)
(E/'applied.patch').write_bytes(subprocess.check_output(['git','diff','--binary','--',*changes],cwd=ROOT))
dump(E/'application-v1.json',{'main_parent_commit':BASE,'implementation_sha256':changes,'final_source_inputs':len(qualified),'qualified_source_hashes_sha256':sha(WORK/'candidate-preparation-v3/source-hashes.json.gz'),'final_input_hashes_sha256':sha(E/'final-input-hashes-v1.json.gz'),'all_final_inputs_match_qualified_sources':True,'retained_completed_evidence':retained,'format_command':command,'format_exit_code':p.returncode,'format_stdout_sha256':sha(E/'main-format.stdout'),'format_stderr_sha256':sha(E/'main-format.stderr'),'pending_native_paths_remaining':0,'native_disposition_preserved':True,'full_plan_complete':False})
assert p.returncode==0
print('Applied 9 qualified files; main formatting passed',flush=True)
