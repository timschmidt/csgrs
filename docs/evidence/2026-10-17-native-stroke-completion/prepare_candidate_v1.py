"""Freeze current sources and a single retained-circle measurement experiment."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];E=ROOT/'docs/evidence/2026-10-17-kicad-track-failures'
BASE='b137381247a03342ce5b6184710475fcc22c9d63'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True);subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
assert read(W/'direct-observation-v2/result.json')['complete']
parent=read(E/'checkpoint-v1.json');archive=ROOT/parent['candidate_snapshot']
with gzip.open(E/'final-input-hashes-v1.json.gz','rt') as f:inputs=json.load(f)
inputs['docs/properties-status.json']=parent['ledger_after_sha256']
assert len(inputs)==2832 and all(sha(ROOT/p)==h for p,h in inputs.items())
baseline=W/'baseline-source-v1';source=W/'candidate-source-v1';out=W/'candidate-preparation-v1'
for directory in [baseline,source,out]:directory.mkdir(exist_ok=False)
for p,h in inputs.items():
    for dest in [baseline/p,source/p]:
        dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/p,dest);assert sha(dest)==h
with gzip.open(out/'baseline-source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
name='crates/hypercurve/src/bezier_region.rs';text=(source/name).read_text();old=(archive/name).read_text()
start=old.index('    fn retained_circular_signed_area(');end=old.index('\n}\n\nfn rational_line_signed_area_contribution(',start)
helper=old[start:end]
for signature in [
    '    pub(crate) fn signed_area_contribution_raw(\n        &self,\n        policy: &CurveContext,\n    ) -> CurveResult<Classification<Option<Real>>> {\n',
    '    fn signed_area_contribution_with_cache(\n        &self,\n        policy: &CurveContext,\n        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,\n    ) -> CurveResult<Classification<Option<Real>>> {\n']:
    assert text.count(signature)==1
    text=text.replace(signature,signature+'        if let Some(area) = self.retained_circular_signed_area(policy) {\n            return Ok(Classification::Decided(Some(area)));\n        }\n')
anchor='\n}\n\nfn rational_line_signed_area_contribution('
assert text.count(anchor)==1;text=text.replace(anchor,'\n\n'+helper+anchor);(source/name).write_text(text)
test='crates/hypercurve/tests/native_linear_stroke.rs';shutil.copy2(archive/test,source/test)
fmt=subprocess.run(['cargo','fmt','--all'],cwd=source,capture_output=True)
(out/'format.stdout').write_bytes(fmt.stdout);(out/'format.stderr').write_bytes(fmt.stderr);assert fmt.returncode==0
assert sha(source/test)==sha(archive/test)
final={p:sha(source/p) for p in inputs|{test:''}}
changes={p:h for p,h in final.items() if inputs.get(p)!=h};assert set(changes)=={name,test}
assert all(sha(ROOT/p)==h for p,h in inputs.items())
with gzip.open(out/'source-hashes.json.gz','wt') as f:json.dump(final,f,sort_keys=True)
dump(out/'result.json',{'complete':True,'base_commit':BASE,'source_inputs':len(final),'baseline_inputs':len(inputs),
    'source_hashes_sha256':sha(out/'source-hashes.json.gz'),'changes':changes,'runner_sha256':sha(__file__),
    'archive_component':'Only retained circular signed-area reuse; original eight curve tests byte-identical.',
    'main_sources_modified':False})
common=(ROOT/'target/properties/kicad-track-failures/validation_common_v3.py').read_text()
common=common.replace('target/properties/kicad-track-failures','target/properties/native-stroke-completion').replace('candidate-source-v3','candidate-source-v1').replace('candidate-preparation-v3','candidate-preparation-v1').replace("+'-validation-v3'","+'-validation-v1'").replace("+'-target-v3'","+'-target-v1'").replace("+'-scratch-v3'","+'-scratch-v1'").replace("+'-executables-v3'","+'-executables-v1'")
common=common.replace('ac14e99174318291a548749395552ab982a7d097',BASE)
(W/'validation_common_v1.py').write_text(common)
print(json.dumps({'source_inputs':len(final),'changed_paths':list(changes)}),flush=True)
