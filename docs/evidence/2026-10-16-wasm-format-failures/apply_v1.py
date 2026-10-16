"""Apply the seven byte-verified WASM paths after all qualification completed."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
WORK=ROOT/'target/properties/wasm-format-failures'
PARENT=E.parent/'2026-10-16-export-projection-contracts'
BASE='3d335db43eac00d003bc2fb0ed1beb2a9c19d79b'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())
def gz(path):
    with gzip.open(path,'rt') as stream:return json.load(stream)

assert not (E/'application-v1.json').exists()
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
parent=read(PARENT/'checkpoint-v1.json')
assert len(parent['pending_native_sha256'])==35
assert all(sha(ROOT/path)==digest for path,digest in parent['pending_native_sha256'].items())
baseline=gz(WORK/'baseline-v1/source-hashes.json.gz')
assert all(sha(ROOT/path)==digest for path,digest in baseline.items())
assert sha(ROOT/'docs/properties-status.json')==parent['ledger_after_sha256']
preparation=read(WORK/'preparation-v1/result.json')
assert preparation['complete'] and preparation['native_parser_and_validation_unchanged']
inputs=gz(WORK/'preparation-v1/source-hashes.json.gz')
assert sha(WORK/'preparation-v1/source-hashes.json.gz')==preparation['source_hashes_sha256']
assert all(sha(WORK/'candidate-source-v1'/path)==digest for path,digest in inputs.items())
paths=preparation['implementation_paths']
assert len(paths)==7 and set(paths)=={'src/wasm/mod.rs','src/wasm/mesh_js.rs','src/wasm/io_error_js.rs',
    'scripts/test-wasm-bindings.sh','tests/wasm/export_projection.cjs','tests/wasm/io_contracts.cjs','tests/wasm/io_contracts.ts'}
assert set(inputs)-set(baseline)=={'src/wasm/io_error_js.rs','tests/wasm/io_contracts.cjs','tests/wasm/io_contracts.ts'}
assert all(inputs[path]==digest for path,digest in baseline.items() if path not in paths)
for folder,commands in [('native-validation-v1',6),('wasm-validation-v1',3)]:
    result=read(WORK/folder/'result.json')
    assert result['complete'] and not result['unexpected_exits'] and result['commands']==commands
    assert result['sources_unchanged'] and result['read_only_tools_unchanged']
    assert read(WORK/folder/'protocol.json')['source_hashes_sha256']==preparation['source_hashes_sha256']
wasm=read(WORK/'wasm-validation-v1/result.json')
assert wasm['real_js_and_compiled_typescript_consumers_passed']
assert all(sha(path)==record['sha256'] for path,record in wasm['artifacts'].items())
cost=read(WORK/'cost-v1/result.json')
assert cost['complete'] and cost['processes']==95 and cost['rows']==190
assert cost['complete_inputs_and_outputs_byte_equal'] and cost['artifacts_and_probes_unchanged']
resources=read(WORK/'import-resources-v1/result.json')
assert resources['complete'] and resources['wasm_artifacts_unchanged']
assert read(WORK/'baseline-v1/result.json')['all_expected_exits']
retained={}
for folder in ['baseline-v1','preparation-v1','native-validation-v1','wasm-validation-v1','import-resources-v1','cost-v1']:
    destination=E/folder
    assert not destination.exists()
    for source in sorted((WORK/folder).rglob('*')):
        assert not source.is_symlink(),source
        if not source.is_file():continue
        output=destination/source.relative_to(WORK/folder)
        output.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,output)
        assert sha(source)==sha(output)
        retained[str(output.relative_to(E))]={'original':str(source),'sha256':sha(source)}
shutil.copyfile(WORK/'preparation-v1/source-hashes.json.gz',E/'final-input-hashes-v1.json.gz')
for path in paths:
    source=WORK/'candidate-source-v1'/path
    destination=ROOT/path
    assert not destination.is_symlink()
    if path not in baseline:assert not destination.exists()
    shutil.copyfile(source,destination)
assert all(sha(ROOT/path)==digest for path,digest in inputs.items())
command=['cargo','fmt','--all','--','--check']
result=subprocess.run(command,cwd=ROOT,capture_output=True)
(E/'main-format.stdout').write_bytes(result.stdout)
(E/'main-format.stderr').write_bytes(result.stderr)
record={'main_parent_commit':BASE,'implementation_sha256':{path:sha(ROOT/path) for path in paths},
        'final_input_hashes_sha256':sha(E/'final-input-hashes-v1.json.gz'),
        'source_input_count':len(inputs),'main_build_inputs_equal_tested_snapshot':True,
        'validation_partition':'One frozen source snapshot qualifies both native and WASM targets; every final production/test/harness byte and Cargo input is shared.',
        'format_command':command,'format_exit_code':result.returncode,
        'format_stdout_sha256':sha(E/'main-format.stdout'),'format_stderr_sha256':sha(E/'main-format.stderr'),
        'retained_completed_evidence':retained,'pending_native_files_unchanged':35,
        'native_runtime_admitted':False,'published_package_pipeline_executed':False,'full_plan_complete':False}
(E/'application-v1.json').write_text(json.dumps(record,indent=2)+'\n')
assert result.returncode==0
assert all(sha(ROOT/path)==digest for path,digest in parent['pending_native_sha256'].items())
print(json.dumps({'applied_paths':len(paths),'source_inputs':len(inputs),'retained_evidence_files':len(retained),'main_fmt_exit_code':result.returncode}),flush=True)
