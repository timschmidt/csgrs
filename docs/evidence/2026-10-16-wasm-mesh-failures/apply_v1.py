"""Apply only the qualified WASM error-contract change after the native disposition commit."""
import gzip,hashlib,json,re,shutil,subprocess
from pathlib import Path
WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[2]
E=ROOT/'docs/evidence/2026-10-16-wasm-mesh-failures'
PARENT=ROOT/'docs/evidence/2026-10-16-native-change-disposition'
def sha(path):
 with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def read(path):return json.loads(Path(path).read_text())
def gz(path):
 with gzip.open(path,'rt') as stream:return json.load(stream)
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')
def outside(raw):
 start=re.search(rb'"current_work"\s*:\s*',raw).end()
 _,n=json.JSONDecoder().raw_decode(raw[start:].decode())
 end=start+len(raw[start:].decode()[:n].encode())
 return raw[:start],raw[end:]
parent=read(PARENT/'checkpoint-v1.json')
commit=read(ROOT/'target/properties/native-change-disposition-git-v1/commit-v1.json')
assert commit['disposition_complete'] and parent['disposition_complete']
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==commit['commit']
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True)
assert not E.exists()
assert all(sha(ROOT/path)==digest for path,digest in parent['evidence_files_sha256'].items())
assert sha(ROOT/'docs/properties-status.json')==parent['ledger_after_sha256']
assert read(WORK/'baseline-v1/result.json')['complete']
assert read(WORK/'baseline-v1/result.json')['javascript_exit_code']==1
assert read(WORK/'baseline-v1/result.json')['typescript_compile_exit_code']==0
assert read(WORK/'baseline-v1/result.json')['typescript_consumer_exit_code']==1
for folder in ['native-validation-v1','wasm-validation-v1']:
 result=read(WORK/folder/'result.json')
 assert result['complete'] and not result['unexpected_exits']
assert read(WORK/'wasm-validation-v1/result.json')['real_js_and_compiled_typescript_consumers_passed']
cost=read(WORK/'cost-v1/result.json')
assert cost['complete'] and cost['complete_success_values_and_stl_bytes_equal'] and cost['native_diagnostic_messages_equal']
prep=read(WORK/'preparation-v1/result.json')
qualified=gz(WORK/'preparation-v1/source-hashes.json.gz')
previous=gz(ROOT/'target/properties/native-change-disposition/preparation-v1/committed-source-hashes.json.gz')
changes=prep['changes']
assert set(changes)=={'src/wasm/mesh_js.rs','scripts/test-wasm-bindings.sh','tests/wasm/mesh_errors.cjs','tests/wasm/mesh_errors.ts'}
assert all(sha(WORK/'candidate-source-v1'/path)==digest for path,digest in qualified.items())
assert all(sha(ROOT/path)==digest for path,digest in previous.items() if path!='docs/properties-status.json')
assert all(not (ROOT/path).exists() for path in set(qualified)-set(previous))
reference=subprocess.check_output(['git','show','9025de62:docs/properties-status.json'],cwd=ROOT)
old_metadata=(WORK/'candidate-source-v1/docs/properties-status.json').read_bytes()
current_metadata=(ROOT/'docs/properties-status.json').read_bytes()
assert outside(old_metadata)==outside(current_metadata)==outside(reference)
assert {k:v for k,v in json.loads(old_metadata).items() if k!='current_work'}=={k:v for k,v in json.loads(current_metadata).items() if k!='current_work'}
# No implemented source, fixture, policy or dependency is waived: only the
# previously committed admission ledger's current_work value differs.
E.mkdir()
retained={}
folders=['preflight-v1','prototype-v1','baseline-v1','preparation-v1','wasm-validation-v1','native-validation-v1','cost-v1']
for folder in folders:
 for source in sorted((WORK/folder).rglob('*')):
  assert not source.is_symlink(),source
  if not source.is_file() or source.suffix=='.wasm':continue
  output=E/folder/source.relative_to(WORK/folder)
  output.parent.mkdir(parents=True,exist_ok=True)
  shutil.copy2(source,output)
  assert sha(output)==sha(source)
  retained[str(output.relative_to(E))]={'original':str(source),'sha256':sha(source)}
for source in [*WORK.glob('*.py'),WORK/'cost_v1.cjs',WORK/'cost-contract-v1.json']:
 output=E/source.name;shutil.copy2(source,output)
 assert sha(output)==sha(source)
 retained[source.name]={'original':str(source),'sha256':sha(source)}
for path,digest in changes.items():
 assert sha(WORK/'candidate-source-v1'/path)==digest
 output=ROOT/path;output.parent.mkdir(parents=True,exist_ok=True)
 output.write_bytes((WORK/'candidate-source-v1'/path).read_bytes())
final=qualified|{'docs/properties-status.json':parent['ledger_after_sha256']}
assert all(sha(ROOT/path)==digest for path,digest in final.items())
with gzip.open(E/'final-input-hashes-v1.json.gz','wt') as stream:json.dump(final,stream,sort_keys=True)
command=['cargo','fmt','--all','--','--check']
result=subprocess.run(command,cwd=ROOT,capture_output=True)
(E/'main-format.stdout').write_bytes(result.stdout);(E/'main-format.stderr').write_bytes(result.stderr)
record={'main_parent_commit':commit['commit'],'qualified_runtime_base_commit':prep['base_runtime_commit'],
 'implementation_sha256':changes,'final_source_inputs':len(final),
 'qualified_source_hashes_sha256':sha(WORK/'preparation-v1/source-hashes.json.gz'),
 'final_input_hashes_sha256':sha(E/'final-input-hashes-v1.json.gz'),
 'source_inputs_equal_except_admission_metadata':True,
 'admission_metadata_only_difference':{'path':'docs/properties-status.json','qualified_sha256':qualified['docs/properties-status.json'],'current_sha256':parent['ledger_after_sha256'],'raw_bytes_outside_current_work_equal':True,'other_decoded_keys_equal':True},
 'retained_completed_evidence':retained,'format_command':command,'format_exit_code':result.returncode,
 'format_stdout_sha256':sha(E/'main-format.stdout'),'format_stderr_sha256':sha(E/'main-format.stderr'),
 'pending_native_paths_remaining':0,'native_disposition_preserved':True,'full_plan_complete':False}
dump(E/'application-v1.json',record)
assert result.returncode==0
print(json.dumps({'applied_paths':len(changes),'final_inputs':len(final),'format_exit_code':result.returncode,'pending_native_paths':0}),flush=True)
