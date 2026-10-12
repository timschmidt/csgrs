"""Run scalar, consumer, lint and format checks sequentially for the retained-bound variant."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
expected=sha(E/'typed-api-prototype-v1684-domain-bound-candidate-build/source-hashes.json.gz')
assert all(r['result']['exit_code']==0 and r['cap']['exit_code']==0 and not r['cap']['wall_timeout']
 for r in read(E/'domain-bound-candidate-caps-v1693.json')['tests'])
commands=[
 ('typed-api-prototype-v1694-domain-bound-scalar-tests',['cargo','test','--release','-p','hyperreal','--lib','--all-features']),
 ('typed-api-prototype-v1695-domain-bound-clippy',read(E/'typed-api-prototype-v1537-atan-zero-term-clippy/command.json')['command']),
 ('typed-api-prototype-v1696-domain-bound-release-build',['cargo','build','--release','-p','hyperreal','-p','hypercurve','--all-features']),
 ('typed-api-prototype-v1697-domain-bound-workspace-fmt',['cargo','fmt','--all','--','--check']),
 ('typed-api-prototype-v1698-domain-bound-included-fmt',read(E/'typed-api-prototype-v1376-included-fmt/command.json')['command']+[
 'crates/hyperreal/src/computable/node/log_range_probe_tests.rs','crates/hyperreal/src/computable/node/domain_sign_tests.rs',
 'crates/hyperreal/src/computable/node/domain_bound_tests.rs']),
 ('typed-api-prototype-v1699-domain-bound-stack-tests',read(E/'typed-api-prototype-v1372-stack-tests/command.json')['command']),
 ('typed-api-prototype-v1700-domain-bound-default-tests',read(E/'typed-api-prototype-v1373-default-tests/command.json')['command']),
 ('typed-api-prototype-v1701-domain-bound-debug-tests',read(E/'typed-api-prototype-v1371-debug-scalar-unit-tests/command.json')['command']),
 ('typed-api-prototype-v1702-domain-bound-minimal-clippy',read(E/'typed-api-prototype-v1374-minimal-clippy/command.json')['command']),
]
rows={}
for phase,command in commands:
 assert not (E/phase).exists()
 subprocess.run([sys.executable,str(E/'validate_candidate_v2.py'),phase,'--exec',*command],cwd=ROOT,check=True)
 r=read(E/phase/'result.json')
 assert r['exit_code']==0 and r['source_unchanged'] and r['main_unchanged'],(phase,r)
 assert sha(E/phase/'source-hashes.json.gz')==expected
 rows[phase]={'command':command,'result':r}
 print(phase,'passed',flush=True)
out=E/'domain-bound-candidate-validation-v1702.json';assert not out.exists()
out.write_text(json.dumps({'source_record_sha256':expected,'phases':rows},indent=2)+'\n')
