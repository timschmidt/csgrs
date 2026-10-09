"""Reconstruct the exact pre-bound regression sources in a separate checkout.

Usage: python3 reproduce_area_baseline.py BASE_CHECKOUT OUTPUT
BASE_CHECKOUT must be a disposable checkout at the recorded base commit. This
copies the native-pad sources and the exact two pre-bound tests before running
them sequentially; it does not modify the current working tree.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

record = Path(__file__).resolve().parent
root = record.parents[2]
checkout,out = map(lambda s:Path(s).resolve(),sys.argv[1:3])
assert checkout != root and not out.exists()
out.mkdir(parents=True)
load = lambda p: json.loads(p.read_bytes())
base = load(record/'baseline.json')['production_commit']
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=checkout,text=True).strip() == base
baseline = load(record/'pre-area-fix/area-regression-baseline.json')
reconstruction = load(record/'pre-area-fix/area-baseline-reconstruction.json')
for path,expected in baseline['source_hashes'].items():
    if path == reconstruction['path']:
        data = subprocess.check_output(['git','show',f'{base}:{path}'],cwd=root)
        anchor = reconstruction['anchor'].encode(); assert data.count(anchor) == 1
        data = data.replace(anchor,(record/'pre-area-fix/area-baseline-tests.rs').read_bytes()+reconstruction['insert_suffix'].encode()+anchor)
    else:
        data = (root/path).read_bytes()
    assert hashlib.sha256(data).hexdigest() == expected,path
    destination = checkout/path; destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_bytes(data)
status = []
for i,row in enumerate(baseline['commands']):
    with (out/f'test-{i+1}.log').open('w') as log:
        result = subprocess.run(row['command'],cwd=checkout,stdout=log,stderr=subprocess.STDOUT)
    status.append(dict(command=row['command'],exit_code=result.returncode))
    assert result.returncode == row['exit_code']
assert all(hashlib.sha256((checkout/p).read_bytes()).hexdigest() == h for p,h in baseline['source_hashes'].items())
(out/'status.json').write_text(json.dumps(status,indent=2)+'\n')
print('Reproduced the original CPArti-pair failure and passing tolerance control.')
