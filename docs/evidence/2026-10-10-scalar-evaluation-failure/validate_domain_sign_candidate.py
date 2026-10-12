"""Run scalar, consumer, lint and format checks sequentially for the retaining variant."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
expected=sha(E/'typed-api-prototype-v1610-domain-sign-retaining-build/source-hashes.json.gz')
assert all(r['result']['exit_code']==0 and r['cap']['exit_code']==0 and not r['cap']['wall_timeout']
 for r in read(E/'domain-sign-retaining-caps-v1621.json')['tests'].values())
commands=[
 ('typed-api-prototype-v1622-domain-sign-scalar-tests',['cargo','test','--release','-p','hyperreal','--lib','--all-features']),
 ('typed-api-prototype-v1623-domain-sign-clippy',read(E/'typed-api-prototype-v1537-atan-zero-term-clippy/command.json')['command']),
 ('typed-api-prototype-v1624-domain-sign-release-build',['cargo','build','--release','-p','hyperreal','-p','hypercurve','--all-features']),
 ('typed-api-prototype-v1625-domain-sign-workspace-fmt',['cargo','fmt','--all','--','--check']),
 ('typed-api-prototype-v1626-domain-sign-included-fmt',read(E/'typed-api-prototype-v1376-included-fmt/command.json')['command']+[
 'crates/hyperreal/src/computable/node/log_range_probe_tests.rs','crates/hyperreal/src/computable/node/domain_sign_tests.rs']),
 ('typed-api-prototype-v1627-domain-sign-stack-tests',read(E/'typed-api-prototype-v1372-stack-tests/command.json')['command']),
 ('typed-api-prototype-v1628-domain-sign-default-tests',read(E/'typed-api-prototype-v1373-default-tests/command.json')['command']),
 ('typed-api-prototype-v1629-domain-sign-debug-tests',read(E/'typed-api-prototype-v1371-debug-scalar-unit-tests/command.json')['command']),
 ('typed-api-prototype-v1630-domain-sign-minimal-clippy',read(E/'typed-api-prototype-v1374-minimal-clippy/command.json')['command']),
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
out=E/'domain-sign-candidate-validation-v1630.json';assert not out.exists()
out.write_text(json.dumps({'source_record_sha256':expected,'phases':rows},indent=2)+'\n')
