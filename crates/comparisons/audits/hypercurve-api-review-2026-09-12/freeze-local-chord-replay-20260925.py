from pathlib import Path
import hashlib,json,shutil,sys
A=Path(__file__).resolve().parent
W=A.parent
version,base_version=sys.argv[1:]
assert all(v.startswith('v') and v[1:].isdigit() for v in [version,base_version])
prefix=f'local-chord-complete-replay-20260924-{version}'
root=Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
assert not root.exists()
archive=A/'source-archives'/root.name
assert not archive.exists()
archive.mkdir(parents=True)
root.symlink_to(archive,target_is_directory=True)
assert not (A/f'{prefix}-sources.json').exists()
base=json.loads((A/f'local-chord-complete-replay-20260924-{base_version}-sources.json').read_text())
manifest={}
for name in base:
    src=W/name
    dst=root/name
    dst.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(src,dst)
    assert (src.stat().st_dev,src.stat().st_ino)!=(dst.stat().st_dev,dst.stat().st_ino)
    sha=hashlib.sha256(src.read_bytes()).hexdigest()
    assert hashlib.sha256(dst.read_bytes()).hexdigest()==sha
    manifest[name]=sha
assert len(manifest)==2044
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(version+': 2044 physical source copies bound; sources frozen.')
