"""Replay five retained public budget corpora sequentially without changing fixtures."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
build='typed-api-prototype-v1624-domain-sign-release-build'
unit='typed-api-prototype-v1622-domain-sign-scalar-tests'
expected=read(E/'domain-sign-retaining-caps-v1621.json')['source_record_sha256']
assert sha(E/build/'source-hashes.json.gz')==sha(E/unit/'source-hashes.json.gz')==expected
commands=[
 ('inverse_trig','reproduce_inverse_trig_budget.py','typed-api-prototype-v1640-domain-sign-inverse-replay','typed-api-prototype-v1362-integer-scale-inverse-replay',384,[unit,'replay']),
 ('evaluation','reproduce_evaluation_budget.py','typed-api-prototype-v1641-domain-sign-evaluation-replay','typed-api-prototype-v1363-integer-scale-evaluation-replay',384,[unit]),
 ('algebraic','reproduce_algebraic_budget_v2.py','typed-api-prototype-v1642-domain-sign-algebraic-replay','typed-api-prototype-v1364-integer-scale-algebraic-replay',288,[unit,'replay']),
 ('algebraic_wide','reproduce_algebraic_wide_budget.py','typed-api-prototype-v1643-domain-sign-algebraic-wide-replay','typed-api-prototype-v1365-integer-scale-wide-replay',144,[unit,'replay']),
 ('normal_form','reproduce_normal_form_budget.py','typed-api-prototype-v1644-domain-sign-normal-form-replay','typed-api-prototype-v1366-integer-scale-normal-form-replay',384,[unit,'replay'])
]
out=E/'domain-sign-replay-orchestration-v1644.json';assert not out.exists()
rows=[]
for family,script,phase,baseline,count,extra in commands:
 assert not (E/phase).exists()
 started=time.monotonic();command=[sys.executable,str(E/script),build,phase,*extra]
 result=subprocess.run(command,cwd=ROOT)
 row={'family':family,'phase':phase,'baseline':baseline,'command':command,'exit_code':result.returncode,'seconds':time.monotonic()-started,'expected_cases':count}
 rows.append(row)
 if result.returncode==0:
  s=read(E/phase/'summary.json')
  assert s['source_hashes_sha256']==expected and s['guards_unchanged']
  assert s['cases']==count
  assert all(sha(E/phase/f)==sha(E/baseline/f) for f in ['probe.rs','reproduce.py'])
  row['summary_sha256']=sha(E/phase/'summary.json')
 out.write_text(json.dumps({'source_record_sha256':expected,'complete':len(rows)==len(commands) and result.returncode==0,'results':rows},indent=2)+'\n')
 assert result.returncode==0,row
 print(family,count,'replayed',flush=True)
assert sum(row['expected_cases'] for row in rows)==1584
