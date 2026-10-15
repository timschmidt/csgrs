"""Consolidate identical non-executable artifacts in explicitly closed trials."""
from pathlib import Path
import gzip, hashlib, json, os
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
MEMO = {}


def sha(path):
    stat = path.stat(); key = (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)
    if key not in MEMO:
        h = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''): h.update(block)
        MEMO[key] = h.hexdigest()
    return MEMO[key]


out = E/'tmp-consolidation-v1.json'
assert not out.exists()
roots = []
for directory, phases in [
 ('curve-constructor-failures',['baseline-v1','candidate-v1','candidate-v2','candidate-v3']),
 ('curve-metaball-failures',['baseline-v1','bindings-v1','bindings-v2','candidate-v3','candidate-v4','candidate-v5','candidate-v6','candidate-v7','native-v1','predicate-baseline-v1','wrappers-baseline-v1']),
 ('strict-drilled-workflows',['candidate-v1','qualification-v1']),
]:
    for phase in phases:
        record = ROOT/f'docs/evidence/2026-10-15-{directory}'/phase/'result.json'
        assert record.is_file()
        json.loads(record.read_text())
        roots.append(Path('/tmp')/f'csgrs-{directory}'/phase)
protected = {}
groups = {}
for root in roots:
    for path in root.rglob('*'):
        if not path.is_file(): continue
        assert not path.is_symlink()
        digest = sha(path); protected[str(path)] = digest
        stat = path.stat()
        if path.suffix in {'.rlib','.rmeta','.d'} and not stat.st_mode & 0o111:
            groups.setdefault((digest, stat.st_size, stat.st_dev, stat.st_mode), set()).add(path)
# Preserve every alias already recorded for a path in the selected groups.
manifests = list((ROOT/'docs/evidence').glob('2026-10-15-*/linked-compiler-duplicates-v*.json.gz'))
old_links = []
for manifest in manifests:
    for name,row in json.loads(gzip.decompress(manifest.read_bytes())).items():
        path = Path(name); origin = Path(row['retained_original'])
        assert sha(path) == sha(origin) == row['sha256']
        assert path.stat().st_ino == origin.stat().st_ino
        old_links.append((path,origin,row['sha256']))
        for p in [path,origin]:
            stat=p.stat(); key=(row['sha256'],stat.st_size,stat.st_dev,stat.st_mode)
            if key in groups: groups[key].add(p)
linked = {}
for (digest,size,_,_), paths in groups.items():
    origin = max(paths,key=lambda p:(p.stat().st_nlink,str(p)))
    for path in paths:
        if path.stat().st_ino != origin.stat().st_ino:
            linked[str(path)]={'retained_original':str(origin),'sha256':digest,'bytes':size}
manifest=E/'linked-compiler-duplicates-v1.json.gz'
with gzip.open(manifest,'wt') as stream:json.dump(linked,stream,sort_keys=True)
print(json.dumps({'ready_to_link':len(linked),'protected_paths':len(protected)}),flush=True)
reclaimed=0
for name,row in linked.items():
    path=Path(name);origin=Path(row['retained_original'])
    assert sha(path)==sha(origin)==row['sha256']
    if path.stat().st_nlink==1:reclaimed+=path.stat().st_size
    temporary=path.with_name(path.name+'.strict-drill-dedupe-v1')
    assert not temporary.exists()
    os.link(origin,temporary);os.replace(temporary,path)
for name,digest in protected.items():assert sha(Path(name))==digest
for path,origin,digest in old_links:
    assert sha(path)==sha(origin)==digest and path.stat().st_ino==origin.stat().st_ino
for name,row in linked.items():
    assert sha(Path(name))==row['sha256'] and Path(name).stat().st_ino==Path(row['retained_original']).stat().st_ino
record={'runner_sha256':sha(Path(__file__)),'manifest_sha256':sha(manifest),'closed_roots':list(map(str,roots)),'protected_artifacts_sha256':protected,'prior_aliases_verified':len(old_links),'linked_paths':len(linked),'duplicate_logical_bytes_reclaimed':reclaimed,'scope':'Closed compiler .rlib/.rmeta/.d files only; all paths and bytes preserved. No executable, source, raw measurement, or unique compiler artifact removed. Fresh mutable trials must use independent inodes.'}
out.write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({k:v for k,v in record.items() if k not in ['protected_artifacts_sha256','closed_roots']}),flush=True)
