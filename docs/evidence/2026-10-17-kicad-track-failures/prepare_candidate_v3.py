"""Correct explicit imports in the migrated private tests; retain prior attempts."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;S=W/'candidate-source-v3';O=W/'candidate-preparation-v3'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
prior=read(W/'candidate-validation-v2/result.json')
assert prior['complete'] and set(prior['unexpected_exits'])=={'native-tests','full-board','copper-neck','gerber','clippy'}
assert 'cannot find type `TrackLoadError` in this scope' in (W/'candidate-validation-v2/native-tests.stderr').read_text()
with gzip.open(W/'candidate-preparation-v2/source-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(W/'candidate-source-v2'/p)==h for p,h in inputs.items())
S.mkdir(exist_ok=False);O.mkdir(exist_ok=False)
for p,h in inputs.items():
    dest=S/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/'candidate-source-v2'/p,dest);assert sha(dest)==h
for fixture in ['CPArti FPGA dev board.zip','HVP109A.zip']:
    source=W/'candidate-source-v2/crates/hyperdrc/docs'/fixture;dest=S/'crates/hyperdrc/docs'/fixture
    shutil.copy2(source,dest);assert sha(dest)==sha(source)
path='crates/hyperdrc/src/kicad.rs';p=S/path;s=p.read_text();original=s
old='''mod tests {
    use std::fs;'''
new='''mod tests {
    use std::collections::HashMap;
    use std::fs;'''
assert s.count(old)==1;s=s.replace(old,new)
old='''    use crate::sexp;

    use super::{CopperKind, load_kicad_pcb, xy_from_child_source};'''
new='''    use crate::sexp::{self, Sexp};

    use super::{CopperKind, TrackLoadError, TrackLoadFailure, load_kicad_pcb,
        parse_footprints, parse_tracks_and_vias, parse_zones, xy_from_child_source};'''
assert s.count(old)==1;s=s.replace(old,new)
assert original.split('#[cfg(test)]',1)[0]==s.split('#[cfg(test)]',1)[0]
p.write_text(s)
subprocess.run(['cargo','fmt','--all'],cwd=S,check=True)
assert {p for p,h in inputs.items() if sha(S/p)!=h}=={path}
inputs[path]=sha(S/path)
changes=read(W/'candidate-preparation-v2/result.json')['changes']|{path:inputs[path]}
with gzip.open(O/'source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
dump(O/'result.json',{'complete':True,'base_commit':'ac14e99174318291a548749395552ab982a7d097',
    'source_inputs':len(inputs),'source_hashes_sha256':sha(O/'source-hashes.json.gz'),'changes':changes,
    'runner_sha256':sha(__file__),'delta_from_v2':{path:inputs[path]},'production_code_unchanged_from_v2':True,
    'correction':'Add explicit imports required by the migrated internal tests. No fixture, assertion, policy, runtime behavior, or original workload gate changes.',
    'preserved_failed_attempt':'candidate-validation-v2','main_sources_modified':False})
print(json.dumps({'complete':True,'source_inputs':len(inputs),'changed_from_v2':[path]}),flush=True)
