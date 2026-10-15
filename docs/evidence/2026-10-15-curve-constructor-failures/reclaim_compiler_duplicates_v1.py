"""Reclaim only verified non-executable compiler duplicates in closed trials."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PRIOR = ROOT / 'docs/evidence/2026-10-15-curve-metaball-failures'

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    out = E / 'compiler-reclamation-v1.json'
    assert not out.exists()
    protected = json.loads((PRIOR / 'closure-v1.json').read_text())['preserved_executables_sha256']
    for result in E.glob('*/result.json'):
        data = json.loads(result.read_text())
        assert data['passed'] or data['commands'][-1]['exit_code'] != 0
        for row in data['commands']:
            protected.update(row['binaries_sha256'])
    assert all(sha(ROOT / p) == h for p,h in protected.items())
    original_paths = set()
    for manifest in PRIOR.glob('removed-*.json.gz'):
        for row in json.loads(gzip.decompress(manifest.read_bytes())).values():
            original_paths.add(str((ROOT / row['retained_original']).resolve()))
    canon = [ROOT / 'target/curve-metaball-failures/native-v2', ROOT / 'target/metaball-failures/native-v1', Path('/tmp/csgrs-curve-constructor-failures/candidate-v2')]
    targets = [Path('/tmp/csgrs-curve-constructor-failures') / phase for phase in ['baseline-v1','candidate-v1']]
    targets += [Path('/tmp/csgrs-curve-metaball-failures') / phase for phase in ['native-v1','bindings-v1','candidate-v3','candidate-v4','candidate-v5','candidate-v6','wrappers-baseline-v1','predicate-baseline-v1']]
    def eligible(p):
        return p.suffix in {'.rlib','.rmeta','.d'} and p.is_file() and not p.is_symlink() and not p.stat().st_mode & 0o111
    by_name = {}
    for target in canon:
        for p in target.rglob('*'):
            if eligible(p): by_name.setdefault((p.name,p.stat().st_size), []).append(p)
    digests = {}
    removed = {}
    for target in targets:
        records = E if 'curve-constructor-failures' in str(target) else PRIOR
        data = json.loads((records / target.name / 'result.json').read_text())
        assert data['passed'] or data['commands'][-1]['exit_code'] != 0
        for p in target.rglob('*'):
            if not eligible(p) or str(p) in original_paths or str(p) in protected:
                continue
            matches = by_name.get((p.name,p.stat().st_size),[])
            if not matches: continue
            digest=sha(p)
            for original in matches:
                key=str(original)
                if key not in digests: digests[key]=sha(original)
                if digests[key]==digest:
                    removed[str(p)]={'retained_original':key,'sha256':digest,'bytes':p.stat().st_size}
                    break
    manifest=E/'removed-compiler-duplicates-v1.json.gz'
    with gzip.open(manifest,'wt') as f:json.dump(removed,f,sort_keys=True)
    for name,row in removed.items():
        assert sha(Path(name))==row['sha256']==sha(Path(row['retained_original']))
        Path(name).unlink()
    assert all(sha(ROOT / p)==h for p,h in protected.items())
    result={'runner_sha256':sha(Path(__file__)), 'manifest_sha256':sha(manifest), 'protected_artifacts_verified':len(protected), 'duplicate_count':len(removed),'duplicate_bytes_removed':sum(r['bytes'] for r in removed.values()),'reason':'candidate-v2 hit the temporary filesystem quota while compiling dependencies. Only byte-identical non-executable compiler copies in terminal named trials were removed. Earlier reclamation originals, all executed artifacts, unique compiler files and source/log evidence remain.'}
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)

if __name__=='__main__':main()
