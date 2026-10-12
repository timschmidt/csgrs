#!/usr/bin/env python3
"""Restore only this experiment's three source differences to retained bytes."""
import gzip
import hashlib
import json
from pathlib import Path

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
SOURCE=ROOT/'target/scalar-evaluation-candidate/source'
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
out=E/'acos-source-restoration-v1496.json'
assert not out.exists()
rejected=read(E/'acos-outlined-rejection-v1495.json')
current_record=E/rejected['source_record']
assert sha(current_record)==rejected['source_record_sha256']
current=json.loads(gzip.decompress(current_record.read_bytes()))
retained=read(E/'typed-api-prototype-v1404-state.json')
retained_record=E/retained['source_record']
assert sha(retained_record)==retained['source_record_sha256']
original=json.loads(gzip.decompress(retained_record.read_bytes()))
for n,h in current.items():
    assert sha(SOURCE/n)==h,n
changed={n for n,h in current.items() if original.get(n)!=h}
prefix='crates/hyperreal/src/computable/'
assert changed=={prefix+'node.rs',prefix+'approximation/inverse_trig.rs',prefix+'node/acos_warm_probe_tests.rs'}
for n in changed:
    archive=current_record.parent/'inputs'/n
    assert sha(archive)==current[n],n
    if n in original:
        old=retained_record.parent/'inputs'/n
        assert sha(old)==original[n],n
        (SOURCE/n).write_bytes(old.read_bytes())
    else:
        (SOURCE/n).unlink()
assert all(sha(SOURCE/n)==h for n,h in original.items())
guard=json.loads(gzip.decompress((E/'candidate-preparation/main-hashes.json.gz').read_bytes()))
assert all(sha(ROOT/n)==h for n,h in guard.items())
out.write_text(json.dumps({'status':'Retained source restored exactly; both experiments remain archived and rejected',
    'retained_checkpoint':'typed-api-prototype-v1404-state.json','retained_source_record':retained['source_record'],
    'retained_source_sha256':retained['source_record_sha256'],'source_files_verified':len(original),
    'main_files_verified':len(guard),'restored_or_removed':sorted(changed),
    'rejected_source_record':str(current_record.relative_to(E)),'rejected_source_sha256':sha(current_record),
    'fixtures_preserved':'All ten new tests and both runtime variants remain in immutable phase input archives. No pre-existing retained test was edited or removed.'},indent=2)+'\n')
print(out.read_text(),end='')
