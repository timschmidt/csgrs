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
build='typed-api-prototype-v1696-domain-bound-release-build'
unit='typed-api-prototype-v1694-domain-bound-scalar-tests'
expected=read(E/'domain-bound-candidate-caps-v1693.json')['source_record_sha256']
assert sha(E/build/'source-hashes.json.gz')==sha(E/unit/'source-hashes.json.gz')==expected
commands=[
 ('inverse_trig','reproduce_inverse_trig_budget.py','typed-api-prototype-v1741-domain-bound-inverse-replay','typed-api-prototype-v1640-domain-sign-inverse-replay',384,[unit,'replay']),
 ('evaluation','reproduce_evaluation_budget.py','typed-api-prototype-v1742-domain-bound-evaluation-replay','typed-api-prototype-v1641-domain-sign-evaluation-replay',384,[unit]),
 ('algebraic','reproduce_algebraic_budget_v2.py','typed-api-prototype-v1743-domain-bound-algebraic-replay','typed-api-prototype-v1642-domain-sign-algebraic-replay',288,[unit,'replay']),
 ('algebraic_wide','reproduce_algebraic_wide_budget.py','typed-api-prototype-v1744-domain-bound-algebraic-wide-replay','typed-api-prototype-v1643-domain-sign-algebraic-wide-replay',144,[unit,'replay']),
 ('normal_form','reproduce_normal_form_budget.py','typed-api-prototype-v1745-domain-bound-normal-form-replay','typed-api-prototype-v1644-domain-sign-normal-form-replay',384,[unit,'replay'])
]
out=E/'domain-bound-replay-orchestration-v1745.json';assert not out.exists()
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
