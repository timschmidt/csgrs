"""Evaluate direct native stroke construction without the failed area shortcut."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2]
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
BASE='b137381247a03342ce5b6184710475fcc22c9d63'
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
result=read(W/'candidate-validation-v1/result.json');assert result['complete'] and result['unexpected_exits']==['original-curve-regressions']
with gzip.open(W/'candidate-preparation-v1/baseline-source-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(ROOT/p)==h and sha(W/'baseline-source-v1'/p)==h for p,h in inputs.items())
archive=ROOT/read(ROOT/'docs/evidence/2026-10-17-kicad-track-failures/checkpoint-v1.json')['candidate_snapshot']
source=W/'candidate-source-v2';out=W/'candidate-preparation-v2';source.mkdir(exist_ok=False);out.mkdir(exist_ok=False)
for p,h in inputs.items():
    dest=source/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/'baseline-source-v1'/p,dest);assert sha(dest)==h
name='crates/hypercurve/src/bezier_region.rs';text=(source/name).read_text()
assert text.count('mod corner_edit;\n')==1;text=text.replace('mod corner_edit;\n','mod corner_edit;\nmod linear_stroke;\n')
anchor='        let closed = match crate::curve::curve_path_is_closed(path, policy) {'
assert text.count(anchor)==1
text=text.replace(anchor,'''        if let [curve] = path.curves()
            && let Some(CurveGeometry2::Line(line)) = curve.geometry()
            && line.endpoints_decided_distinct()
            && matches!(cap_style, OffsetCap::Butt | OffsetCap::Round)
        {
            return linear_stroke::native_line_stroke(line, &half_width, cap_style, policy)
                .map(Classification::Decided);
        }
'''+anchor)
(source/name).write_text(text)
added=['crates/hypercurve/src/bezier_region/linear_stroke.rs','crates/hypercurve/tests/native_linear_stroke.rs']
for p in added:shutil.copy2(archive/p,source/p)
fmt=subprocess.run(['cargo','fmt','--all'],cwd=source,capture_output=True)
(out/'format.stdout').write_bytes(fmt.stdout);(out/'format.stderr').write_bytes(fmt.stderr);assert fmt.returncode==0
assert all(sha(source/p)==sha(archive/p) for p in added)
final={p:sha(source/p) for p in inputs|dict.fromkeys(added)}
changes={p:h for p,h in final.items() if inputs.get(p)!=h};assert set(changes)=={name,*added}
with gzip.open(out/'source-hashes.json.gz','wt') as f:json.dump(final,f,sort_keys=True)
dump(out/'result.json',{'complete':True,'base_commit':BASE,'source_inputs':len(final),'source_hashes_sha256':sha(out/'source-hashes.json.gz'),
    'changes':changes,'runner_sha256':sha(__file__),'preserved_failed_candidate':'candidate-validation-v1',
    'archived_files_reused_exactly':{p:sha(archive/p) for p in added},'global_measurement_code_unchanged':True,'main_sources_modified':False})
common=(W/'validation_common_v1.py').read_text().replace('candidate-source-v1','candidate-source-v2').replace('candidate-preparation-v1','candidate-preparation-v2').replace("+'-validation-v1'","+'-validation-v2'").replace("+'-target-v1'","+'-target-v2'").replace("+'-scratch-v1'","+'-scratch-v2'").replace("+'-executables-v1'","+'-executables-v2'")
(W/'validation_common_v2.py').write_text(common)
print(json.dumps({'source_inputs':len(final),'changed_paths':list(changes)}),flush=True)
