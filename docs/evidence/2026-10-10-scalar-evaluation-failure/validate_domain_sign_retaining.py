"""Run the eleven frozen domain and logarithm regressions under independent caps."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
build=E/'typed-api-prototype-v1610-domain-sign-retaining-build'
result=read(build/'result.json')
assert result['exit_code']==0 and result['source_unchanged'] and result['main_unchanged']
paths=re.findall(r'Executable unittests src/lib\.rs \(([^)]+)\)',(build/'stderr.log').read_text())
assert len(paths)==1
binary=Path(paths[0]);binary_hash=sha(binary)
rows={};offset=1611
for module,prefix,old in [
 ('domain_sign_tests','domain_sign_','typed-api-prototype-v1606-domain-sign-publication-build'),
 ('log_range_probe_tests','log_range_','typed-api-prototype-v1580-log-range-retry-build')]:
 source=Path('crates/hyperreal/src/computable/node')/(module+'.rs')
 assert sha(build/'inputs'/source)==sha(E/old/'inputs'/source)
 tests=re.findall(r'    fn ('+prefix+r'\w+)\(\)',(build/'inputs'/source).read_text())
 assert len(tests)==(6 if module == "domain_sign_tests" else 5)
 for test in tests:
  phase=f'typed-api-prototype-v{offset}-domain-sign-cap';offset+=1
  selected='computable::node::'+module+'::'+test
  assert not (E/phase).exists()
  subprocess.run([sys.executable,str(E/'validate_candidate_v2.py'),phase,'--exec',
   sys.executable,str(E/'run_limited_unit_test.py'),str(binary),binary_hash,
   selected,'--exact','--nocapture','--test-threads=1'],cwd=ROOT,check=True)
  result=read(E/phase/'result.json')
  assert result['source_unchanged'] and result['main_unchanged']
  assert sha(E/phase/'source-hashes.json.gz')==sha(build/'source-hashes.json.gz')
  log=(E/phase/'stdout.log').read_text();cap=json.loads(log[log.rfind('{"command":'):])
  assert cap['binary_sha256']==binary_hash
  assert cap['address_space_bytes']==192*1024*1024 and cap['cpu_seconds_soft_hard']==[2,3]
  assert cap['wall_seconds_limit']==4 and cap['helper_sha256']==sha(E/'run_limited_unit_test.py')
  rows[phase]={'test':test,'result':result,'cap':cap}
  print(phase,'exit',result['exit_code'],flush=True)
assert offset==1622
out=E/'domain-sign-retaining-caps-v1621.json';assert not out.exists()
out.write_text(json.dumps({'source_record':str((build/'source-hashes.json.gz').relative_to(E)),
 'source_record_sha256':sha(build/'source-hashes.json.gz'),'tests':rows},indent=2)+'\n')
