"""Freeze the direct construction, independent regressions and public contract."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2]
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert read(W/'candidate-validation-v2/result.json')['unexpected_exits']==[]
assert read(W/'contracts-observation-v2/result.json')['contract_exit_code']==0
with gzip.open(W/'candidate-preparation-v2/source-hashes.json.gz','rt') as f:prior=json.load(f)
with gzip.open(W/'candidate-preparation-v1/baseline-source-hashes.json.gz','rt') as f:baseline=json.load(f)
assert all(sha(W/'candidate-source-v2'/p)==h for p,h in prior.items())
assert all(sha(ROOT/p)==h for p,h in baseline.items())
source=W/'candidate-source-v4';out=W/'candidate-preparation-v4';source.mkdir(exist_ok=False);out.mkdir(exist_ok=False)
for p,h in prior.items():
    dest=source/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/'candidate-source-v2'/p,dest);assert sha(dest)==h
test='crates/hypercurve/tests/native_linear_stroke_contracts.rs';shutil.copy2(W/'contract_tests_v1.rs',source/test)
name='crates/hypercurve/src/bezier_region.rs';text=(source/name).read_text()
anchor='    /// General polynomial, rational, B-spline, and NURBS paths retain analytic\n'
assert text.count(anchor)==1
text=text.replace(anchor,'''    /// A single line with certified distinct endpoints retains its two exact
    /// parallels and butt or semicircular endpoint caps. Its native boundary
    /// remains reusable for exact area, bounds, point queries and subsequent
    /// region operations. Boundary subdivisions are an implementation detail;
    /// this construction can change them while preserving the represented set.
    /// Unresolved endpoint distinctness retains the general certified path.
    ///
'''+anchor);(source/name).write_text(text)
name='crates/hypercurve/README.md';text=(source/name).read_text()
anchor='  segment_certified, offset}` is the mixed-family region API.'
assert text.count(anchor)==1;text=text.replace(anchor,'  segment_certified, offset, stroke_path}` is the mixed-family region API.')
anchor='  Unordered segment admission returns the region through the same exact-result\n'
assert text.count(anchor)==1
text=text.replace(anchor,'''  `stroke_path` takes a positive half-width, a corner style and an endpoint
  `OffsetCap`. A certified distinct single line supports exact butt and round
  strokes, including diagonal directions, symbolic radii and exact coordinates
  beyond binary64 range. Retained parallels and semicircular caps keep analytic
  area, bounds and rebuilt boundary paths consumable under `STRICT`. Boundary
  subdivision and ordering can differ from earlier arrangement output; compare
  represented sets separately from serialized artifacts. Unresolved predicates
  remain typed blockers. This does not add a total-work or heap-byte bound.
'''+anchor);(source/name).write_text(text)
fmt=subprocess.run(['cargo','fmt','--all'],cwd=source,capture_output=True)
(out/'format.stdout').write_bytes(fmt.stdout);(out/'format.stderr').write_bytes(fmt.stderr);assert fmt.returncode==0
assert sha(source/test)==sha(W/'candidate-source-v3'/test)
assert sha(source/'crates/hypercurve/tests/native_linear_stroke.rs')==prior['crates/hypercurve/tests/native_linear_stroke.rs']
final={p:sha(source/p) for p in prior|{test:''}}
changes={p:h for p,h in final.items() if baseline.get(p)!=h}
assert len(changes)==5
with gzip.open(out/'source-hashes.json.gz','wt') as f:json.dump(final,f,sort_keys=True)
dump(out/'result.json',{'complete':True,'base_commit':read(W/'candidate-preparation-v2/result.json')['base_commit'],'source_inputs':len(final),
    'source_hashes_sha256':sha(out/'source-hashes.json.gz'),'changes':changes,'runner_sha256':sha(__file__),
    'preserved_preparation_failure':'candidate-preparation-v3: formatting-only difference from standalone rustfmt; no candidate build executed','production_behavior_unchanged_from_v2':True,'added_public_contract_tests':2,'main_sources_modified':False})
common=(W/'validation_common_v2.py').read_text().replace('candidate-source-v2','candidate-source-v4').replace('candidate-preparation-v2','candidate-preparation-v4').replace("+'-validation-v2'","+'-validation-v4'").replace("+'-target-v2'","+'-target-v4'").replace("+'-scratch-v2'","+'-scratch-v4'").replace("+'-executables-v2'","+'-executables-v4'")
(W/'validation_common_v4.py').write_text(common)
print(json.dumps({'source_inputs':len(final),'changed_paths':list(changes)}),flush=True)
