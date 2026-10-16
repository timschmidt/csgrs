"""Pin the verified WASM contract and new physical evidence without rewriting history."""
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
PARENT=E.parent/'2026-10-16-export-projection-contracts'
WORK=ROOT/'target/properties/wasm-format-failures'
OLD=ROOT/'target/properties/export-projection-contracts'
BASE='3d335db43eac00d003bc2fb0ed1beb2a9c19d79b'
PREFIX=str(E.relative_to(ROOT))+'/'
sys.path.insert(0,str(E.parent/'2026-10-16-physical-build-requalification'))
from load_expectations_v1 import load


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())
def gz(path):
    with gzip.open(path,'rt') as stream:return json.load(stream)


def span(raw):
    start=re.search(rb'"current_work"\s*:\s*',raw).end()
    value,length=json.JSONDecoder().raw_decode(raw[start:].decode())
    return start,start+len(raw[start:].decode()[:length].encode()),value


assert not (E/'checkpoint-v1.json').exists() and not (E/'artifact-expectations-v1.json.gz').exists()
assert (E/'progress-v1.md').is_file()
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
previous_path=PARENT/'checkpoint-v1.json'
previous=read(previous_path)
assert all(sha(ROOT/path)==digest for path,digest in previous['evidence_files_sha256'].items())
assert sha(ROOT/previous['artifact_expectations'])==previous['artifact_expectations_sha256']
prior,inherited_missing=load(ROOT/previous['artifact_expectations'],allow_unqualified=True)
assert len(prior)==934961 and len(inherited_missing)==27093
pending=previous['pending_native_sha256']
assert len(pending)==35 and all(sha(ROOT/path)==digest for path,digest in pending.items())
inherited=previous['inherited_implementation_sha256']|previous['implementation_sha256']
extra,dependencies=previous['extra_unchanged_inputs_sha256'],previous['live_source_dependency_pins_sha256']
decision,application=read(E/'decision-v1.json'),read(E/'application-v1.json')
implementation=application['implementation_sha256']
assert decision['wasm_format_contracts_admitted'] and implementation==decision['implementation_sha256']
assert set(implementation)=={'src/wasm/mod.rs','src/wasm/mesh_js.rs','src/wasm/io_error_js.rs',
    'scripts/test-wasm-bindings.sh','tests/wasm/export_projection.cjs','tests/wasm/io_contracts.cjs','tests/wasm/io_contracts.ts'}
assert all(sha(ROOT/path)==digest for pins in [inherited|implementation,extra,dependencies] for path,digest in pins.items())
inputs=gz(E/'final-input-hashes-v1.json.gz')
assert len(inputs)==2830 and inputs==gz(E/'preparation-v1/source-hashes.json.gz')
assert all(sha(WORK/'candidate-source-v1'/path)==digest for path,digest in inputs.items())
assert all(sha(ROOT/path)==digest for path,digest in inputs.items())
assert application['format_exit_code']==0
assert sha(E/'main-format.stdout')==application['format_stdout_sha256']
assert sha(E/'main-format.stderr')==application['format_stderr_sha256']
for path in E.rglob('*.py'):ast.parse(path.read_text(),filename=str(path))
assert all(sha(E/name)==row['sha256']==sha(row['original']) for name,row in application['retained_completed_evidence'].items())
assert sha(E/'storage_v1.py')==read(E/'storage-v1.json')['runner_sha256']
prep=read(E/'preparation-v1/result.json')
assert sha(E/'prepare_v1.py')==prep['runner_sha256']
assert all(sha(E/path)==digest for path,digest in prep['prototype_sha256'].items())
assert prep['source_hashes_sha256']==sha(E/'final-input-hashes-v1.json.gz')
assert all(sha(path)==digest for path,digest in read(E/'baseline-v1/baseline-artifact-hashes.json').items())
baseline_protocol=read(E/'baseline-v1/protocol.json')
assert sha(E/'baseline_v1.py')==baseline_protocol['runner_sha256']
assert sha(E/'baseline-v1/source-hashes.json.gz')==baseline_protocol['source_hashes_sha256']
assert sha(E/'baseline-v1/baseline-artifact-hashes.json')==baseline_protocol['baseline_artifact_hashes_sha256']
baseline_probe_bindings={}
for logical,digest in baseline_protocol['probes'].items():
    path=Path(logical)
    # This baseline consumer's only later change is its obsolete importer
    # comment. Its exact original bytes remain in the qualified E27 snapshot.
    if path==ROOT/'tests/wasm/export_projection.cjs':
        path=OLD/'wasm-source-v1/tests/wasm/export_projection.cjs'
    assert sha(path)==digest
    baseline_probe_bindings[logical]={'retained_path':str(path),'sha256':digest}
for folder,runner,count in [('native-validation-v1','native_v1.py',6),('wasm-validation-v1','wasm_v1.py',3)]:
    protocol=read(E/folder/'protocol.json')
    result=read(E/folder/'result.json')
    assert result['complete'] and not result['unexpected_exits'] and result['commands']==count
    assert sha(E/runner)==protocol['runner_sha256']
    assert sha(E/'validation_common_v1.py')==protocol['common_sha256']
    assert sha(E/'final-input-hashes-v1.json.gz')==protocol['source_hashes_sha256']
    assert all(sha(path)==digest for path,digest in protocol['tools_read_only_sha256'].items())
    rows=read(E/folder/'commands.json')
    assert len(rows)==count
    for row in rows:
        assert row['exit_code']==row['expected_exit_code']==0
        for suffix in ['stdout','stderr','resources']:
            assert sha(E/folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        assert all(sha(record['retained'])==record['sha256'] for record in row['binaries'].values())
wasm=read(E/'wasm-validation-v1/result.json')
assert wasm['real_js_and_compiled_typescript_consumers_passed']
assert all(sha(path)==record['sha256'] for path,record in wasm['artifacts'].items())
resource_protocol=read(E/'import-resources-v1/protocol.json')
assert sha(E/'import_resources_v1.py')==resource_protocol['runner_sha256']
assert sha(E/'import_resources_v1.cjs')==resource_protocol['probe_sha256']
assert sha(resource_protocol['reader_source'])==resource_protocol['reader_source_sha256']
assert read(E/'import-resources-v1/result.json')['complete']
cost=read(E/'cost-v1/result.json')
assert cost['complete'] and cost['processes']==95 and cost['rows']==190
assert cost['complete_inputs_and_outputs_byte_equal'] and cost['artifacts_and_probes_unchanged']
cost_protocol=read(E/'cost-v1/protocol.json')
assert all(sha(path)==digest for path,digest in cost_protocol['runner_probe_fixture_contract_and_qualification_sha256'].items())
assert all(sha(path)==row['sha256'] for path,row in cost_protocol['artifacts_read_only'].items())
for row in read(E/'cost-v1/commands.json'):
    assert row['exit_code']==0
    for name,record in row['files'].items():
        assert sha(E/'cost-v1'/row['name']/name)==record['sha256']

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


print('Hashing completed physical WASM contract files:',len(paths),flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    physical=dict(pool.map(inspect,paths,buffersize=128))
assert prior.keys().isdisjoint(physical)
expectations={'schema_version':3,'kind':'new physical WASM import/error evidence; inherited historical audit remains failed/incomplete',
 'parent_expectations':previous['artifact_expectations'],'parent_expectations_sha256':previous['artifact_expectations_sha256'],
 'inherited_expected_artifact_paths':len(prior),'inherited_missing_artifact_paths':len(inherited_missing),
 'added_physical_artifacts_sha256':physical,'new_physical_artifacts_verified':len(physical),
 'local_tool_symlink_targets':symlinks,'expected_artifact_paths':len(prior)+len(physical),
 'inherited_artifacts_reverified':False,'complete_artifact_audit_passed':False,'historical_preservation_complete':False,
 'scope':'Completed physical shared source, native/WASM targets, test binaries, metadata, logs and probes are hashed independently per path with stable stat/lstat checks. Parent expectations, known nested-view failures and inherited missing paths remain unchanged. No whole-history verification or zero-loss claim.'}
with gzip.open(E/'artifact-expectations-v1.json.gz','wt') as stream:json.dump(expectations,stream,sort_keys=True)
assert gz(E/'artifact-expectations-v1.json.gz')==expectations
work['wasm_format_contracts']={
 'status':decision['status'],'main_parent_commit':BASE,'contract':PREFIX+'contract-v1.md',
 'progress':PREFIX+'progress-v1.md','decision':PREFIX+'decision-v1.json','checkpoint':PREFIX+'checkpoint-v1.json',
 'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz','implementation_sha256':implementation,
 'cost':PREFIX+'cost-v1/result.json','cost_contract':PREFIX+'cost-contract-v1.md',
 'wasm_format_contracts_admitted':True,'native_runtime_admitted':False,'pending_native_files_unchanged':35,
 'published_package_pipeline_executed':False,'full_plan_complete':False,'next':decision['next']}
updated=raw[:start]+json.dumps(work,indent=2).encode()+raw[end:]
ustart,uend,_=span(updated)
assert (updated[:ustart],updated[uend:])==(reference[:rstart],reference[rend:])
assert {key:value for key,value in json.loads(updated).items() if key!='current_work'}=={
    key:value for key,value in json.loads(reference).items() if key!='current_work'}
assert sha(ledger)==before
ledger.write_bytes(updated)
files={str(path.relative_to(ROOT)):sha(path) for path in E.rglob('*') if path.is_file() and '__pycache__' not in path.parts}
record={'status':decision['status'],'checkpoint_kind':'narrow WASM source admission; historical artifact limitations retained',
 'previous_closure':str(previous_path.relative_to(ROOT)),'previous_closure_sha256':sha(previous_path),
 'evidence_files_sha256':files,'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz',
 'artifact_expectations_sha256':sha(E/'artifact-expectations-v1.json.gz'),'artifact_expectations_schema':3,
 'new_physical_artifacts_verified':len(physical),'inherited_expected_artifact_paths':len(prior),
 'inherited_missing_artifact_paths':len(inherited_missing),'complete_artifact_audit_passed':False,
 'historical_preservation_complete':False,'implementation_sha256':implementation,
 'inherited_implementation_sha256':inherited,'extra_unchanged_inputs_sha256':extra,
 'pending_native_sha256':pending,'live_source_dependency_pins_sha256':dependencies,
 'ledger_before_sha256':before,'ledger_after_sha256':sha(ledger),'only_current_work_changed':True,
 'baseline_probe_bindings':baseline_probe_bindings,'python_sources_parse':True,
 'wasm_format_contracts_admitted':True,'native_runtime_admitted':False,'full_plan_complete':False}
(E/'checkpoint-v1.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
print(json.dumps({key:record[key] for key in ['new_physical_artifacts_verified','only_current_work_changed','wasm_format_contracts_admitted']}),flush=True)
