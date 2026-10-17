"""Prepare independent WASM sources from the qualified committed native snapshot."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[2]
OLD=ROOT/'target/properties/native-change-disposition'
OUT=WORK/'preparation-v1'
SOURCE=WORK/'candidate-source-v1'
PROTO=WORK/'prototype-v1'
def sha(path):
 with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def read(path):return json.loads(Path(path).read_text())
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')
assert read(WORK/'baseline-v1/result.json')['complete']
baseline=read(WORK/'baseline-v1/protocol.json')
assert all(sha(path)==digest for path,digest in baseline['prototype_sha256'].items())
prep=read(OLD/'preparation-v1/result.json')
manifest=OLD/'preparation-v1/committed-source-hashes.json.gz'
assert sha(manifest)==prep['variants']['committed']['source_hashes_sha256']
with gzip.open(manifest,'rt') as stream:inputs=json.load(stream)
assert len(inputs)==2824
assert all(sha(OLD/'committed-source-v1'/path)==digest for path,digest in inputs.items())
OUT.mkdir(exist_ok=False);SOURCE.mkdir(exist_ok=False)
for path,digest in inputs.items():
 output=SOURCE/path;output.parent.mkdir(parents=True,exist_ok=True)
 shutil.copy2(OLD/'committed-source-v1'/path,output)
 assert sha(output)==digest
changes=['src/wasm/mesh_js.rs','scripts/test-wasm-bindings.sh','tests/wasm/mesh_errors.cjs','tests/wasm/mesh_errors.ts']
for path in changes:
 output=SOURCE/path;output.parent.mkdir(parents=True,exist_ok=True)
 shutil.copy2(PROTO/path,output);inputs[path]=sha(output)
assert len(inputs)==2826 and all(sha(SOURCE/path)==digest for path,digest in inputs.items())
with gzip.open(OUT/'source-hashes.json.gz','wt') as stream:json.dump(inputs,stream,sort_keys=True)
record={'complete':True,'runner_sha256':sha(__file__),'base_runtime_commit':'99b88f707b04053de7f40a86d6d029e3c07460d9',
 'source':str(SOURCE),'source_inputs':len(inputs),'source_hashes_sha256':sha(OUT/'source-hashes.json.gz'),
 'qualified_parent_source_manifest_sha256':sha(manifest),'qualified_parent_result_sha256':sha(OLD/'wasm-validation-v1/result.json'),
 'changes':{path:sha(SOURCE/path) for path in changes},'prototype_sha256':baseline['prototype_sha256'],
 'main_sources_modified':False,'completed_targets_reopened':False,
 'ledger_snapshot':'The source snapshot retains the E29 qualified source ledger (E28 current_work). Subsequent admission-only current_work changes are not runtime code; any later application must explicitly verify all other source inputs and preserve the raw protected ledger boundary.',
 'timing_isolation':'This build may overlap E29 curve correctness tests. Its completion must precede E29 timed board gates; overlapping gate evidence must not be called isolated.',
 'new_per_command_external_limit_seconds':720,'candidate_admitted':False}
dump(OUT/'result.json',record)
print(json.dumps({'source_inputs':len(inputs),'modified_or_added_runtime_paths':len(changes),'main_sources_modified':False}),flush=True)
