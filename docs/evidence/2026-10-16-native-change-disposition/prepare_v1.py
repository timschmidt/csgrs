"""Freeze the complete pending integration and its current committed baseline."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
WORK=ROOT/'target/properties/native-change-disposition'
PARENT=E.parent/'2026-10-16-wasm-format-failures'
BASE='99b88f707b04053de7f40a86d6d029e3c07460d9'


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def dump(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')

assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT,check=True)
parent=json.loads((PARENT/'checkpoint-v1.json').read_text())
pending=parent['pending_native_sha256']
assert len(pending)==35 and all(sha(ROOT/path)==digest for path,digest in pending.items())
with gzip.open(PARENT/'final-input-hashes-v1.json.gz','rt') as stream:inputs=json.load(stream)
inputs['docs/properties-status.json']=parent['ledger_after_sha256']
assert all(sha(ROOT/path)==digest for path,digest in inputs.items())
WORK.mkdir(exist_ok=False)
out=WORK/'preparation-v1'
out.mkdir()
raw=subprocess.check_output(['git','diff','--binary','--',*sorted(pending)],cwd=ROOT)
(out/'tracked.patch').write_bytes(raw)
tracked=set(subprocess.check_output(['git','ls-files','-z','--',*sorted(pending)],cwd=ROOT).decode().strip('\0').split('\0'))
assert len(tracked)==29 and tracked<=set(pending)
added=set(pending)-tracked
assert len(added)==6
patches=[]
for path in sorted(added):
    patches.extend(difflib.unified_diff([], (ROOT/path).read_text().splitlines(keepends=True),fromfile='/dev/null',tofile='b/'+path))
(out/'untracked.patch').write_text(''.join(patches))
for path,digest in pending.items():
    destination=E/'pending-source-v1'/path
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/path,destination)
    assert sha(destination)==digest
originals={path:subprocess.check_output(['git','show',BASE+':'+path],cwd=ROOT) for path in tracked}
variants={}
for variant in ['candidate','committed']:
    source=WORK/(variant+'-source-v1')
    source.mkdir()
    manifest={}
    for path,digest in inputs.items():
        if variant=='committed' and path in added:continue
        destination=source/path
        destination.parent.mkdir(parents=True,exist_ok=True)
        if variant=='committed' and path in originals:
            destination.write_bytes(originals[path])
        else:shutil.copy2(ROOT/path,destination)
        manifest[path]=sha(destination)
        if variant=='candidate' or path not in tracked:assert manifest[path]==digest
        assert (destination.stat().st_dev,destination.stat().st_ino)!=((ROOT/path).stat().st_dev,(ROOT/path).stat().st_ino)
    with gzip.open(out/(variant+'-source-hashes.json.gz'),'wt') as stream:json.dump(manifest,stream,sort_keys=True)
    variants[variant]={'source':str(source),'inputs':len(manifest),'source_hashes_sha256':sha(out/(variant+'-source-hashes.json.gz'))}
fixtures=[]
for name,entry,expected in [('CPArti FPGA dev board.zip','CPArti FPGA dev board.kicad_pcb','f57195393ff7a938af0a9651781ba13536d248cbb43f13a41739e2a97cf99e35'),
                           ('HVP109A.zip','HVP109A.kicad_pcb','986b7e26b60c83c97a851c403fb90a927c9018d30ad6b94c7bba5925aa99e28a')]:
    path=ROOT/'crates/hyperdrc/docs'/name
    assert sha(path)==expected
    with zipfile.ZipFile(path) as archive:
        assert archive.testzip() is None
        data=archive.read(entry)
    fixtures.append({'archive':str(path),'sha256':expected,'entry':entry,'entry_bytes':len(data),'entry_sha256':hashlib.sha256(data).hexdigest()})
assert all(sha(ROOT/path)==digest for path,digest in inputs.items())
record={'complete':True,'base_commit':BASE,'runner_sha256':sha(__file__),'pending_native_sha256':pending,
        'tracked_paths':sorted(tracked),'added_paths':sorted(added),'variants':variants,'fixtures':fixtures,
        'native_sources_modified':False,'candidate_preserved_in_repository':True,
        'tracked_patch_sha256':sha(out/'tracked.patch'),'untracked_patch_sha256':sha(out/'untracked.patch'),
        'mount':subprocess.check_output(['findmnt','-T',str(WORK),'-n','-o','TARGET,FSTYPE,OPTIONS'],text=True)}
assert 'ext4' in record['mount'] and 'fuse' not in record['mount']
dump(out/'result.json',record)
print(json.dumps({'variants':variants,'pending_files_preserved':35}),flush=True)
