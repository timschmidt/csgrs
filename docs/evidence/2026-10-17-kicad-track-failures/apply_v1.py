"""Apply only qualified track-load failures and retain every completed attempt."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2]
E=ROOT/'docs/evidence/2026-10-17-kicad-track-failures'
PARENT=ROOT/'docs/evidence/2026-10-16-containment-bounds'
BASE='ac14e99174318291a548749395552ab982a7d097'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def gz(p):
    with gzip.open(p,'rt') as f:return json.load(f)
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True);subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
parent=read(PARENT/'checkpoint-v1.json')
assert all(sha(ROOT/p)==h for p,h in parent['evidence_files_sha256'].items())
assert sha(ROOT/'docs/properties-status.json')==parent['ledger_after_sha256']
for folder in ['baseline-validation-v1','candidate-validation-v3']:
    r=read(W/folder/'result.json');assert r['complete'] and not r['unexpected_exits']
assert read(W/'candidate-validation-v1/result.json')['unexpected_exits']==['native-tests','full-board','copper-neck','clippy']
assert read(W/'candidate-probe-v1/result.json')['all_invalid_sources_rejected_typed']
cost=read(W/'comparison-v1/result.json')
assert cost['complete'] and cost['all_previously_materialized_features_identical'] and not cost['unexpected_report_differences']
assert len(cost['geometry_additions']['hvp109a'])==5 and len(cost['independent_full_board_disk_checks'])==15
assert len(cost['malformed_cli'])==32
before=gz(W/'baseline-preparation-v1/source-hashes.json.gz');final=gz(W/'candidate-preparation-v3/source-hashes.json.gz')
changes=read(W/'candidate-preparation-v3/result.json')['changes']
assert len(changes)==6 and len(final)==2832
assert all(sha(ROOT/p)==h for p,h in before.items()) and all(sha(W/'candidate-source-v3'/p)==h for p,h in final.items())
assert set(final)-set(before)=={'crates/hyperdrc/src/kicad/tracks.rs','crates/hyperdrc/tests/kicad_track_failures.rs'}
assert all(not (ROOT/p).exists() for p in set(final)-set(before))
E.mkdir(exist_ok=False);retained={}
folders=['draft-v1','prototype-v1','prototype-v2','baseline-preparation-v1','baseline-validation-v1','baseline-probe-v1',
    'candidate-preparation-v1','candidate-preparation-v2','candidate-preparation-v3','candidate-validation-v1','candidate-validation-v2','candidate-validation-v3','candidate-probe-v1','source-disks-v1','consumer-preflight-v1','comparison-v1']
binary_names={'probe','public-tests','original-regressions','baseline-load','candidate-load','disk-corpus','load','disk'}
for folder in folders:
    for source in sorted((W/folder).rglob('*')):
        assert not source.is_symlink()
        if not source.is_file() or source.name in binary_names:continue
        out=E/folder/source.relative_to(W/folder);out.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,out)
        assert sha(out)==sha(source);retained[str(out.relative_to(E))]={'original':str(source),'sha256':sha(source)}
for source in sorted(W.iterdir()):
    if source.is_file():
        out=E/source.name;shutil.copy2(source,out);assert sha(out)==sha(source)
        retained[source.name]={'original':str(source),'sha256':sha(source)}
for path,h in changes.items():
    dest=ROOT/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/'candidate-source-v3'/path,dest);assert sha(dest)==h
assert all(sha(ROOT/p)==h for p,h in final.items())
with gzip.open(E/'final-input-hashes-v1.json.gz','wt') as f:json.dump(final,f,sort_keys=True)
command=['cargo','fmt','--all','--','--check'];result=subprocess.run(command,cwd=ROOT,capture_output=True)
(E/'main-format.stdout').write_bytes(result.stdout);(E/'main-format.stderr').write_bytes(result.stderr)
patch=subprocess.check_output(['git','diff','--binary','--',*changes],cwd=ROOT)
for path in sorted(set(final)-set(before)):
    diff=subprocess.run(['git','diff','--no-index','--binary','--','/dev/null',path],cwd=ROOT,capture_output=True)
    assert diff.returncode==1;patch+=diff.stdout
(E/'applied.patch').write_bytes(patch)
dump(E/'application-v1.json',{'main_parent_commit':BASE,'implementation_sha256':changes,'final_source_inputs':len(final),
    'all_final_inputs_match_qualified_sources':True,'final_input_hashes_sha256':sha(E/'final-input-hashes-v1.json.gz'),
    'retained_completed_evidence':retained,'format_command':command,'format_exit_code':result.returncode,
    'format_stdout_sha256':sha(E/'main-format.stdout'),'format_stderr_sha256':sha(E/'main-format.stderr'),
    'pending_native_paths_remaining':0,'native_disposition_preserved':True,
    'independently_reimplemented_archived_path':'crates/hyperdrc/src/kicad/tracks.rs',
    'full_plan_complete':False})
assert result.returncode==0
print('Applied six qualified files; main formatting passed',flush=True)
