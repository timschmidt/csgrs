"""Fix only the new guard test's error extraction in a fresh immutable candidate."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;S=W/'candidate-source-v3';O=W/'candidate-preparation-v3'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
old=json.loads((W/'candidate-preparation-v2/result.json').read_text())
with gzip.open(W/'candidate-preparation-v2/source-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(W/'candidate-source-v2'/p)==h for p,h in inputs.items())
assert 'error[E0277]' in (W/'candidate-validation-v2/mesh-and-modeling-all-features.stderr').read_text()
S.mkdir(exist_ok=False);O.mkdir(exist_ok=False)
for p,h in inputs.items():
 dest=S/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/'candidate-source-v2'/p,dest);assert sha(dest)==h
p=S/'crates/hypermesh/src/surface_arrangement.rs';s=p.read_text()
needle='arrange_surface(&decisions, &polygons).unwrap_err(),';assert s.count(needle)==1
p.write_text(s.replace(needle,'arrange_surface(&decisions, &polygons).err().unwrap(),'))
subprocess.run(['cargo','fmt','--all'],cwd=S,check=True)
changed={p for p,h in inputs.items() if sha(S/p)!=h};assert changed=={'crates/hypermesh/src/surface_arrangement.rs'}
for p in changed:inputs[p]=sha(S/p)
with gzip.open(O/'source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
dump(O/'result.json',old|{'source':str(S),'runner_sha256':sha(__file__),'source_hashes_sha256':sha(O/'source-hashes.json.gz'),'changes':{p:inputs[p] for p in old['changes']},'prior_attempt':'candidate-v2 focused and actual WASM passed; full native suite failed compiling the newly added test.','correction':'Only test error extraction changes; no production branch, input, policy or assertion is weakened.'})
for name in ['validation_common','candidate','wasm']:
 s=(W/(name+'_v2.py')).read_text().replace('_v2','_v3').replace('-v2','-v3')
 (W/(name+'_v3.py')).write_text(s)
print('Frozen candidate v3: test-only compile correction; production implementation unchanged',flush=True)
