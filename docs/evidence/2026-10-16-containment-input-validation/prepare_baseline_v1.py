"""Freeze current source and proposed acceptance tests before implementation."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[2]
PARENT=ROOT/'docs/evidence/2026-10-16-wasm-mesh-failures'
BASE='c9690a7cbd03c79e485bd7670ca10fe68bc5e45e'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
parent=json.loads((PARENT/'checkpoint-v1.json').read_text())
with gzip.open(PARENT/'final-input-hashes-v1.json.gz','rt') as f:inputs=json.load(f)
inputs['docs/properties-status.json']=parent['ledger_after_sha256']
assert len(inputs)==2826 and all(sha(ROOT/p)==h for p,h in inputs.items())
assert not subprocess.check_output(['git','diff','--name-only'],cwd=ROOT)
OUT=WORK/'baseline-preparation-v1';OUT.mkdir(exist_ok=False)
SOURCE=WORK/'baseline-source-v1';SOURCE.mkdir(exist_ok=False)
for p,h in inputs.items():
 dest=SOURCE/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/p,dest);assert sha(dest)==h
with gzip.open(OUT/'main-input-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
for p in ['tests/containment_inputs.rs','tests/wasm/mesh_errors.cjs']:
 shutil.copy2(WORK/'prototype-v1'/p,SOURCE/p);inputs[p]=sha(SOURCE/p)
with gzip.open(OUT/'source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
dump(OUT/'result.json',{'complete':True,'base_commit':BASE,'source':str(SOURCE),'source_inputs':len(inputs),'runner_sha256':sha(__file__),'source_hashes_sha256':sha(OUT/'source-hashes.json.gz'),'main_input_hashes_sha256':sha(OUT/'main-input-hashes.json.gz'),'prototype_sha256':{str(p.relative_to(WORK/'prototype-v1')):sha(p) for p in (WORK/'prototype-v1').rglob('*') if p.is_file()},'runtime_changed':False})
print('Frozen baseline runtime with 2 acceptance test inputs',flush=True)
