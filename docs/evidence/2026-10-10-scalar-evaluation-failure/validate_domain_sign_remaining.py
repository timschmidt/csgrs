"""Preserve the remaining capped modules and full-size numerical replay corpus."""
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
build='typed-api-prototype-v1624-domain-sign-release-build'
unit='typed-api-prototype-v1622-domain-sign-scalar-tests'
expected=read(E/'domain-sign-retaining-caps-v1621.json')['source_record_sha256']
paths=re.findall(r'Running unittests src/lib\.rs \(([^)]+)\)',(E/unit/'stderr.log').read_text());assert len(paths)==1
binary=Path(paths[0]);binary_hash=sha(binary)
assert sha(E/build/'source-hashes.json.gz')==sha(E/unit/'source-hashes.json.gz')==expected
rows=[]
out=E/'domain-sign-remaining-validation-v1671.json';assert not out.exists()
def save(complete=False):
 out.write_text(json.dumps({'source_record_sha256':expected,'complete':complete,'results':rows},indent=2)+'\n')
for old_version,version in zip(range(1377,1396),range(1645,1664)):
 matches=list(E.glob(f'typed-api-prototype-v{old_version}-*/command.json'));assert len(matches)==1
 old=read(matches[0])['command']
 assert 'run_limited_unit_test.py' in old[1] and old[4].startswith('computable::node::')
 phase=f'typed-api-prototype-v{version}-domain-sign-retained-cap';assert not (E/phase).exists()
 command=[sys.executable,str(E/'run_limited_unit_test.py'),str(binary),binary_hash,*old[4:]]
 subprocess.run([sys.executable,str(E/'validate_candidate_v2.py'),phase,'--exec',*command],cwd=ROOT,check=True)
 result=read(E/phase/'result.json');log=(E/phase/'stdout.log').read_text()
 cap=json.loads(log[log.rfind('{"command":'):])
 row={'phase':phase,'baseline_phase':matches[0].parent.name,'result':result,'cap':cap};rows.append(row);save()
 assert result['exit_code']==cap['exit_code']==0 and not cap['wall_timeout']
 assert result['source_unchanged'] and result['main_unchanged'] and sha(E/phase/'source-hashes.json.gz')==expected
 assert any(int(n)>0 for n in re.findall(r'test result: ok\. (\d+) passed',log))
 print(phase,'passed',flush=True)
commands=[
 ('reproduce_candidate.py','typed-api-prototype-v1664-domain-sign-replay',[build,'typed-api-prototype-v1664-domain-sign-replay'],415),
 ('replay_precision.py','typed-api-prototype-v1665-domain-sign-precision-replay',[build,'typed-api-prototype-v1664-domain-sign-replay','typed-api-prototype-v1665-domain-sign-precision-replay'],60),
 ('reproduce_private_kernel.py','typed-api-prototype-v1666-domain-sign-private-kernel-replay',[build,'typed-api-prototype-v1666-domain-sign-private-kernel-replay'],78),
 ('reproduce_private_trig.py','typed-api-prototype-v1667-domain-sign-private-trig-replay',[build,'typed-api-prototype-v1667-domain-sign-private-trig-replay'],600),
 ('reproduce_private_trig_wide.py','typed-api-prototype-v1668-domain-sign-private-trig-wide-replay',[build,'typed-api-prototype-v1668-domain-sign-private-trig-wide-replay',unit],384),
 ('reproduce_symbolic_trig_scales.py','typed-api-prototype-v1669-domain-sign-trig-scale-replay',[build,'typed-api-prototype-v1669-domain-sign-trig-scale-replay'],168),
 ('reproduce_symbolic_trig_residuals.py','typed-api-prototype-v1670-domain-sign-trig-residual-replay',[build,'typed-api-prototype-v1670-domain-sign-trig-residual-replay',unit],456),
 ('reproduce_symbolic_trig_nonrational.py','typed-api-prototype-v1671-domain-sign-trig-nonrational-replay',[build,'typed-api-prototype-v1671-domain-sign-trig-nonrational-replay',unit],384),
]
for helper,phase,args,count in commands:
 assert not (E/phase).exists()
 command=[sys.executable,str(E/helper),*args]
 r=subprocess.run(command,cwd=ROOT)
 row={'phase':phase,'command':command,'helper_sha256':sha(E/helper),'exit_code':r.returncode,'expected_cases':count};rows.append(row);save()
 assert r.returncode==0,row
 s=read(E/phase/'summary.json');assert s['source_hashes_sha256']==expected and s['guards_unchanged']
 if count==415:
  assert s['ordinary_cases']==400 and s['invalid_first_contracts']==s['invalid_retries']==0
  assert s['typed_interruption_errors']==300 and len(s['extremes'])==6 and len(s['cache'])==9
  assert all(r['valid'] for r in s['extremes']+s['cache']) and all(r['exit_code']==0 for r in s['commands'])
 else:
  assert s['cases']==count
  for key in ['incomplete_cases','invalid_numeric_samples','invalid_samples','unexpected_result_shapes','nonzero_exit_cases','invalid_numeric_cases','typed_errors']:
   if key in s: assert not s[key],(phase,key,s[key])
 row['summary_sha256']=sha(E/phase/'summary.json');save()
 print(phase,count,'cases passed',flush=True)
save(True)
assert sum(c[3] for c in commands)==2545
