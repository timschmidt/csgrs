#!/usr/bin/env python3
"""Refresh executable artifacts after rejecting both atan zero-term experiments."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode=True
from parse_test_results_v3 import parse_phase
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
EXPECTED='32b816463eacd2d2cef2f905c0455557c015117690a8a8dd635229a57726d2b2'
def checked(phase):
    r=read(E/phase/'result.json')
    assert r['exit_code']==0 and r['source_unchanged'] and r['main_unchanged']
    assert sha(E/phase/'source-hashes.json.gz')==EXPECTED
    return r
def validate(phase,args):
    assert not (E/phase).exists()
    subprocess.run([sys.executable,str(E/'validate_candidate_v2.py'),phase,*args],cwd=ROOT,check=True)
    return checked(phase)
unit='typed-api-prototype-v1572-restored-scalar-unit-tests'
results={unit:validate(unit,['test','--release','-p','hyperreal','--lib','--all-features'])}
tests=parse_phase(E/unit)
assert tests['top_level_passed']==1054 and tests['child_process_passed']==1
assert tests['top_level_failed']==tests['child_process_failed']==0
build='typed-api-prototype-v1573-restored-build'
results[build]=validate(build,['build','--release','-p','hyperreal','-p','hypercurve','--all-features'])
fmt='typed-api-prototype-v1574-restored-workspace-fmt'
results[fmt]=validate(fmt,['fmt','--all','--','--check'])
source=json.loads(gzip.decompress((E/build/'source-hashes.json.gz').read_bytes()))
original=json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
changed=sorted(n for n,h in source.items() if n.endswith('.rs') and original.get(n)!=h)
assert len(changed)==105
fmt='typed-api-prototype-v1575-restored-included-fmt'
results[fmt]=validate(fmt,['--exec','rustfmt','--edition','2024','--config-path','crates/.rustfmt.toml',
    '--config','skip_children=true','--check',*changed])
libraries=read(E/'typed-api-prototype-v1357-integer-scale-cost/summary.json')['libraries_sha256']
observed={n:sha(Path(n)) for n in libraries}
assert observed==libraries
out=E/'atan-restoration-validation-v1576.json';assert not out.exists()
out.write_text(json.dumps({'status':'Retained source, tests, release libraries and formatting restored',
    'source_record_sha256':EXPECTED,'phases':results,'tests':tests,'included_rust_files':len(changed),
    'retained_v1404_libraries_sha256':libraries,'restored_libraries_sha256':observed,
    'restored_libraries_match_retained':libraries==observed,
    'limits':'The v1404 broad validation remains evidence for these identical source bytes. This restoration reruns scalar units, the release build and formatting only; it does not claim a fresh full-stack or proof run.'},indent=2)+'\n')
print(json.dumps({'record':out.name,'scalar_units':1054,'child_tests':1,'source_sha256':EXPECTED,
    'included_rust_files':len(changed),'release_libraries_match_retained':libraries==observed}),flush=True)
