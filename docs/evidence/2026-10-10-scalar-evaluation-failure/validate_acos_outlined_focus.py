#!/usr/bin/env python3
"""Screen the outlined residual against frozen correctness and targeted costs."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
EXPECTED = '2c9f69fb9f20310fb5d9b21acf98cdb5e830eea9e6b0e43975f5b85b1c5b173c'

def checked(phase):
    result = read(E/phase/'result.json')
    assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged'], phase
    assert sha(E/phase/'source-hashes.json.gz') == EXPECTED

def validate(phase, args):
    assert not (E/phase).exists()
    subprocess.run([sys.executable,str(E/'validate_candidate_v2.py'),phase,*args],cwd=ROOT,check=True)
    checked(phase)

first = 'typed-api-prototype-v1487-acos-outlined-build'
checked(first)
subprocess.run([sys.executable,str(E/'validate_acos_warm_allocations.py'),first,'1488'],cwd=ROOT,check=True)
for phase in read(E/'acos-warm-allocation-caps-v1489.json')['tests']:
    checked(phase)
validate('typed-api-prototype-v1490-acos-outlined-unit-tests',['test','--release','-p','hyperreal','--lib','--all-features'])
assert '1064 passed; 0 failed;' in (E/'typed-api-prototype-v1490-acos-outlined-unit-tests/stdout.log').read_text()
validate('typed-api-prototype-v1491-acos-outlined-clippy',['clippy','--release','-p','hyperreal','-p','hyperlimit',
    '-p','hyperlattice','-p','hypercurve','-p','hyperdrc','--all-targets','--all-features','--','-D','warnings'])
build='typed-api-prototype-v1492-acos-outlined-build'
validate(build,['build','--release','-p','hyperreal','-p','hypercurve','--all-features'])
cost='typed-api-prototype-v1493-acos-outlined-targeted-cost'
subprocess.run([sys.executable,str(E/'reproduce_acos_warm_cost_v2.py'),build,cost],cwd=ROOT,check=True)
r=read(E/cost/'summary.json')
assert r['processes']==72 and r['source_hashes_sha256']==EXPECTED and r['guards_unchanged']
for helper in ['replay_acos_outlined_targeted_cost.py','replay_acos_outlined_preconditioned_cost.py']:
    subprocess.run([sys.executable,str(E/helper)],cwd=ROOT,check=True)
print('Outlined candidate screen completed; retention still requires comparison review.',flush=True)
