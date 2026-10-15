"""Hard-link identical non-executable compiler files only in immutable closed trials.

Every path and byte remains available. Future trials must copy seeds, never
resume these directories; copied seeds have independent inodes.
"""
import gzip, hashlib, json, os
from pathlib import Path
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
PRIOR=ROOT/'docs/evidence/2026-10-15-sampled-profile-failures'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 out=E/'compiler-consolidation-v2.json'
 assert not out.exists()
 protected=json.loads((PRIOR/'closure-v1.json').read_text())['preserved_executables_sha256']
 targets=[]
 # Every selected target is a terminal validation phase. No Cargo process is
 # resumed in these immutable directories; new trials must copy their seeds.
 for folder in ['2026-10-15-sampled-profile-failures', '2026-10-15-planar-operation-failures']:
  records=ROOT/'docs/evidence'/folder
  for p in sorted(records.glob('*/result.json')):
   proto=json.loads((p.parent/'protocol.json').read_text())
   if 'environment' not in proto or 'CARGO_TARGET_DIR' not in proto['environment']:continue
   r=json.loads(p.read_text())
   assert r['passed'] or r['commands'][-1]['exit_code']!=0
   for row in r['commands']:protected.update(row['binaries_sha256'])
   target=Path(proto['environment']['CARGO_TARGET_DIR'])
   if target.is_relative_to(ROOT/'target') and target.is_dir():targets.append(target)
 targets=sorted(set(targets))
 assert all(sha(ROOT/p)==h for p,h in protected.items())
 seen={};linked={};reclaimed=0
 for target in targets:
  print(json.dumps({"scanning_closed_target":str(target)}),flush=True)
  for p in sorted(target.rglob('*')):
   if p.suffix not in {'.rlib','.rmeta','.d'} or not p.is_file() or p.is_symlink():continue
   st=p.stat()
   if st.st_mode&0o111:continue
   h=sha(p);key=(h,st.st_size,st.st_dev,st.st_mode)
   if key not in seen:seen[key]=p;continue
   original=seen[key];other=original.stat()
   if st.st_ino==other.st_ino:continue
   linked[str(p)]={'retained_original':str(original),'sha256':h,'bytes':st.st_size,'old_inode':st.st_ino,'old_nlink':st.st_nlink,'original_inode':other.st_ino}
   if st.st_nlink==1:reclaimed+=st.st_size
 manifest=E/'linked-compiler-duplicates-v2.json.gz'
 with gzip.open(manifest,'wt') as f:json.dump(linked,f,sort_keys=True)
 for name,row in linked.items():
  p=Path(name);original=Path(row['retained_original']);tmp=p.with_name(p.name+'.planar-dedupe-v2')
  assert not tmp.exists();assert sha(p)==row['sha256']==sha(original)
  os.link(original,tmp);os.replace(tmp,p)
  assert p.stat().st_ino==original.stat().st_ino and sha(p)==row['sha256']
 assert all(sha(ROOT/p)==h for p,h in protected.items())
 result={'runner_sha256':sha(Path(__file__)),'manifest_sha256':sha(manifest),'linked_paths':len(linked),'duplicate_bytes_consolidated':reclaimed,'protected_artifacts_verified':len(protected),'closed_targets':[str(t) for t in targets],'scope':'Only non-executable compiler artifacts in terminal recent trials on the repository filesystem. Every path and byte retained. Executables, unique files, logs and source snapshots untouched. These targets remain immutable; new phases copy seeds with independent inodes.'}
 out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
if __name__=='__main__':main()
