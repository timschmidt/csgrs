"""Strengthen the retained WASM assertion and retain a separate undecided query."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;S=W/'candidate-source-v2';O=W/'candidate-preparation-v2'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
observations=read(W/'supporting-fact-v1/result.json')
assert observations['complete'] and observations['read_only_inputs_unchanged']
baseline=observations['observations']['baseline']['rows']
candidate=observations['observations']['candidate']['rows']
assert [r['status'] for r in baseline[:3]]==['Unknown','Decided','Unknown']
assert all(r['status']=='Decided' and r['certainty']=='Certified' and r['value'] is False for r in candidate[:3])
assert baseline[3:]==candidate[3:]
old=read(W/'candidate-preparation-v1/result.json')
with gzip.open(W/'candidate-preparation-v1/source-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(W/'candidate-source-v1'/p)==h for p,h in inputs.items())
S.mkdir(exist_ok=False);O.mkdir(exist_ok=False)
for p,h in inputs.items():
    dest=S/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/'candidate-source-v1'/p,dest);assert sha(dest)==h
p=S/'tests/wasm/geometry_certainty.cjs';s=p.read_text()
needle='''    const result = keep(restoredCube.containsPointWithContext(surfacePoint, approximate));
    assert.equal(result.status, approximate ? Status.Decided : Status.Unknown);'''
replacement='''    const result = keep(restoredCube.containsPointWithContext(surfacePoint, approximate));
    // The certified lower supporting plane proves non-interiority of this
    // retained mesh, without proving fidelity of the inverse rotation.
    assert.equal(result.status, Status.Decided);
    assert.equal(result.value, false);
    assert.equal(result.certainty, Certainty.Certified);
    assert.equal(result.reason, undefined);
  }
  // Its upper coordinate still requires the unresolved trigonometric identity.
  // Keep strict uncertainty and qualified approximate evidence observable.
  const unresolvedSurfacePoint = keep(new Point3Js(1, 0.5, 0.5));
  for (const approximate of [false, true, false]) {
    const result = keep(restoredCube.containsPointWithContext(unresolvedSurfacePoint, approximate));
    assert.equal(result.status, approximate ? Status.Decided : Status.Unknown);'''
assert s.count(needle)==1;p.write_text(s.replace(needle,replacement))
subprocess.run(['cargo','fmt','--all','--','--check'],cwd=S,check=True)
changed={p for p,h in inputs.items() if sha(S/p)!=h}
assert changed=={'tests/wasm/geometry_certainty.cjs'}
for p in changed:inputs[p]=sha(S/p)
with gzip.open(O/'source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
changes=old['changes']|{p:inputs[p] for p in changed}
dump(O/'result.json',old|{'source':str(S),'runner_sha256':sha(__file__),
    'source_hashes_sha256':sha(O/'source-hashes.json.gz'),'changes':changes,
    'only_changes_from_v1':sorted(changed),'native_inputs_identical_to_v1':True,
    'correction':'Retain the original query and require its new Certified false proof; add the unresolved upper-face query with strict Unknown and qualified approximate false assertions. No production code, original geometry or policy changes.',
    'native_qualification':'Reuse candidate-validation-v1 only because every Rust source, manifest, dependency and native test is identical; the changed JavaScript consumer is requalified in wasm-validation-v2.'})
for name in ['validation_common','wasm']:
    s=(W/(name+'_v1.py')).read_text().replace('_v1','_v2').replace('-v1','-v2')
    s=s.replace('export-projection-contracts/wasm-tools-v2','export-projection-contracts/wasm-tools-v1')
    s=s.replace("OLD/'wasm-tools-v2/result.json'","OLD/'wasm-tools-v1/result.json'")
    s=s.replace("OLD/'wasm-source-v2/tests/wasm/node_modules/typescript'","OLD/'wasm-source-v1/tests/wasm/node_modules/typescript'")
    (W/(name+'_v2.py')).write_text(s)
print(json.dumps({'complete':True,'production_unchanged':True,'native_source_and_tests_unchanged':True,'changed_from_v1':sorted(changed)}),flush=True)
