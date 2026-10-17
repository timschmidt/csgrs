"""Restore only the rejected native integration after qualifying the committed alternative."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
WORK=ROOT/'target/properties/native-change-disposition'
BASE='99b88f707b04053de7f40a86d6d029e3c07460d9'


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())
def gz(path):
    with gzip.open(path,'rt') as stream:return json.load(stream)

assert not (E/'application-v1.json').exists()
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
prep=read(WORK/'preparation-v1/result.json')
candidate=read(WORK/'candidate-validation-v1/result.json')
committed=read(WORK/'committed-validation-v1/result.json')
wasm=read(WORK/'wasm-validation-v1/result.json')
regressions=read(WORK/'committed-proposed-regressions-v1/result.json')
assert candidate['complete'] and set(candidate['failed_commands'])=={'full-board','copper-neck'}
assert committed['complete'] and not committed['failed_commands']
assert wasm['complete'] and not wasm['unexpected_exits'] and wasm['real_js_and_compiled_typescript_consumers_passed']
assert regressions['complete'] and regressions['source_and_libraries_unchanged']
assert read(E/'collection-v1.json')['complete']
original=gz(WORK/'preparation-v1/candidate-source-hashes.json.gz')
final=gz(WORK/'preparation-v1/committed-source-hashes.json.gz')
assert all(sha(ROOT/path)==digest for path,digest in original.items())
assert all(sha(WORK/'candidate-source-v1'/path)==digest for path,digest in original.items())
assert all(sha(WORK/'committed-source-v1'/path)==digest for path,digest in final.items())
pending=prep['pending_native_sha256']
assert len(pending)==35
assert all(sha(ROOT/path)==digest==sha(E/'pending-source-v1'/path) for path,digest in pending.items())
assert len(prep['tracked_paths'])==29 and len(prep['added_paths'])==6
assert set(original)-set(final)==set(prep['added_paths'])
restored={}
for path in prep['tracked_paths']:
    data=(WORK/'committed-source-v1'/path).read_bytes()
    assert data==subprocess.check_output(['git','show',BASE+':'+path],cwd=ROOT)
    assert not (ROOT/path).is_symlink()
    (ROOT/path).write_bytes(data)
    restored[path]=sha(ROOT/path)
for path in prep['added_paths']:
    assert sha(ROOT/path)==pending[path] and sha(E/'pending-source-v1'/path)==pending[path]
    (ROOT/path).unlink()
assert all(sha(ROOT/path)==digest for path,digest in final.items())
assert all(not (ROOT/path).exists() for path in prep['added_paths'])
subprocess.run(['git','diff','--quiet','--',*sorted(pending)],cwd=ROOT,check=True)
shutil.copyfile(WORK/'preparation-v1/committed-source-hashes.json.gz',E/'final-input-hashes-v1.json.gz')
command=['cargo','fmt','--all','--','--check']
result=subprocess.run(command,cwd=ROOT,capture_output=True)
(E/'main-format.stdout').write_bytes(result.stdout)
(E/'main-format.stderr').write_bytes(result.stderr)
record={'main_parent_commit':BASE,'decision':'reject submitted native integration; restore committed sources',
        'runtime_implementation_changes_against_head':0,'implementation_sha256':{},
        'restored_native_sha256':restored,'removed_added_paths':prep['added_paths'],
        'rejected_native_sha256':pending,'candidate_preserved_in_repository':True,
        'candidate_snapshot_prefix':str((E/'pending-source-v1').relative_to(ROOT)),
        'final_input_hashes_sha256':sha(E/'final-input-hashes-v1.json.gz'),'final_source_inputs':len(final),
        'main_inputs_equal_qualified_committed_snapshot':True,
        'legacy_validation_does_not_establish_proposed_correctness_claims':True,
        'format_command':command,'format_exit_code':result.returncode,
        'format_stdout_sha256':sha(E/'main-format.stdout'),'format_stderr_sha256':sha(E/'main-format.stderr'),
        'pending_native_paths_remaining':0,'native_candidate_admitted':False,'full_plan_complete':False}
(E/'application-v1.json').write_text(json.dumps(record,indent=2)+'\n')
assert result.returncode==0
print(json.dumps({'restored_tracked_native_files':29,'removed_saved_additions':6,'candidate_files_preserved':35,'format_exit_code':result.returncode}),flush=True)
