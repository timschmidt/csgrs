"""Freeze the actual current revision before reproducing track-source failures."""
import gzip,hashlib,json,shutil,subprocess,zipfile
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2]
PARENT=ROOT/'docs/evidence/2026-10-16-containment-bounds'
BASE='ac14e99174318291a548749395552ab982a7d097'
S=W/'baseline-source-v1';O=W/'baseline-preparation-v1'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--quiet'],cwd=ROOT,check=True)
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
parent=read(PARENT/'checkpoint-v1.json')
with gzip.open(PARENT/'final-input-hashes-v1.json.gz','rt') as f:inputs=json.load(f)
inputs['docs/properties-status.json']=parent['ledger_after_sha256']
assert len(inputs)==2830 and all(sha(ROOT/p)==h for p,h in inputs.items())
S.mkdir(exist_ok=False);O.mkdir(exist_ok=False)
for p,h in inputs.items():
    dest=S/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/p,dest)
    assert sha(dest)==h
with gzip.open(O/'source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
fixtures=[]
old=read(ROOT/'docs/evidence/2026-10-16-native-change-disposition/preparation-v1/result.json')
for fixture in old['fixtures']:
    p=S/'crates/hyperdrc/docs'/Path(fixture['archive']).name
    assert sha(p)==fixture['sha256']
    with zipfile.ZipFile(p) as z:
        assert z.testzip() is None
        content=z.read(fixture['entry'])
    assert hashlib.sha256(content).hexdigest()==fixture['entry_sha256'] and len(content)==fixture['entry_bytes']
    fixtures.append(fixture|{'snapshot_archive':str(p)})
dump(O/'result.json',{'complete':True,'base_commit':BASE,'source':str(S),'source_inputs':len(inputs),
    'runner_sha256':sha(__file__),'source_hashes_sha256':sha(O/'source-hashes.json.gz'),
    'fixtures':fixtures,'full_original_archives_preserved':True,'main_sources_modified':False})
common=(ROOT/'target/properties/containment-bounds/validation_common_v1.py').read_text()
common=common.replace('target/properties/containment-bounds','target/properties/kicad-track-failures')
common=common.replace('311d04fbbe5635a29fe7e754b838cccc4b8add53',BASE)
(W/'validation_common_v1.py').write_text(common)
print(json.dumps({'complete':True,'source_inputs':len(inputs),'fixtures':len(fixtures)}),flush=True)
