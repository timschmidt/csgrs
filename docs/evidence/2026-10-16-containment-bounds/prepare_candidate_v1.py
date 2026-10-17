"""Freeze the certified supporting-plane proof and its boundary regressions."""
import gzip, hashlib, json, shutil, subprocess
from pathlib import Path

WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[2]
OLD=ROOT/'target/properties/containment-winding'
SOURCE=WORK/'candidate-source-v1'
OUT=WORK/'candidate-preparation-v1'
BASE='311d04fbbe5635a29fe7e754b838cccc4b8add53'
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,v): Path(p).write_text(json.dumps(v,indent=2)+'\n')
baseline=json.loads((WORK/'baseline-v1/result.json').read_text())
assert baseline['complete'] and baseline['native_test_exit_code']==baseline['wasm_test_exit_code']==0
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
with gzip.open(WORK/'baseline-v1/current-input-hashes.json.gz','rt') as f: inputs=json.load(f)
assert all(sha(ROOT/p)==h for p,h in inputs.items())
SOURCE.mkdir(exist_ok=False)
OUT.mkdir(exist_ok=False)
for p,h in inputs.items():
    out=SOURCE/p;out.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/p,out)
    assert sha(out)==h
for p in ['tests/containment_bounds.rs','tests/wasm/containment_winding.cjs']:
    shutil.copy2(WORK/'prototype-v1'/p,SOURCE/p)
p=SOURCE/'crates/hypermesh/src/mesh.rs'
s=p.read_text()
old='''    /// edges and vertices follow the represented set's interior. Source-face
    /// queries reuse the native Boolean arrangement to establish that boundary.'''
new='''    /// edges and vertices follow the represented set's interior. Certified
    /// supporting bounds exclude points that cannot be strictly inside. Other
    /// source-face queries reuse the native Boolean arrangement to establish
    /// that boundary.'''
assert s.count(old)==1
s=s.replace(old,new)
old='''                if compare_real_decision(decisions, coordinate, minimum)?.is_lt()
                    || compare_real_decision(decisions, coordinate, maximum)?.is_gt()
                {
                    return Ok(false);
                }'''
new='''                // A validated finite solid has exterior points arbitrarily
                // close to a supporting plane, so that plane has no strict
                // interior. This shortcut needs certified validation, bounds
                // and equality; approximate equality is not that proof.
                let lower = compare_real_decision(decisions, coordinate, minimum)?;
                if lower.is_lt()
                    || (lower.is_eq() && decisions.certainty() == MeshCertainty::Certified)
                {
                    return Ok(false);
                }
                let upper = compare_real_decision(decisions, coordinate, maximum)?;
                if upper.is_gt()
                    || (upper.is_eq() && decisions.certainty() == MeshCertainty::Certified)
                {
                    return Ok(false);
                }'''
assert s.count(old)==1
p.write_text(s.replace(old,new))
changes=['crates/hypermesh/src/mesh.rs','tests/containment_bounds.rs','tests/wasm/containment_winding.cjs']
subprocess.run(['cargo','fmt','--all'],cwd=SOURCE,check=True)
assert {p for p,h in inputs.items() if sha(SOURCE/p)!=h}==set(changes)-{'tests/containment_bounds.rs'}
for p in changes: inputs[p]=sha(SOURCE/p)
with gzip.open(OUT/'source-hashes.json.gz','wt') as f: json.dump(inputs,f,sort_keys=True)
dump(OUT/'result.json',{'complete':True,'base_commit':BASE,'runner_sha256':sha(__file__),
    'source':str(SOURCE),'source_inputs':len(inputs),'source_hashes_sha256':sha(OUT/'source-hashes.json.gz'),
    'changes':{p:inputs[p] for p in changes},'contract_sha256':sha(WORK/'prototype-v1/contract-v1.md'),
    'main_sources_modified':False})
for name in ['validation_common','candidate','wasm']:
    s=(OLD/(name+'_v3.py')).read_text().replace('_v3','_v1').replace('-v3','-v1')
    s=s.replace("target/properties/containment-winding","target/properties/containment-bounds")
    s=s.replace('d4eee9a675fa51d84eb0eb29503dd802bdcdbd19',BASE)
    if name=='candidate':
        s=s.replace("'--test','containment_winding'","'--test','containment_winding','--test','containment_bounds'")
    if name=='wasm':
        s=s.replace("'# tests 9','# pass 9'","'# tests 10','# pass 10'")
        s=s.replace("'public_contract_tests':34","'public_contract_tests':35").replace("'winding_contract_tests':9","'winding_contract_tests':10")
    (WORK/(name+'_v1.py')).write_text(s)
print(json.dumps({'complete':True,'source_inputs':len(inputs),'changes':changes}),flush=True)
