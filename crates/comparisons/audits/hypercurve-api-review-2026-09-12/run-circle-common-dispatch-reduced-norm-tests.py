from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import json,subprocess,time
p=Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
items=[json.loads(l) for l in (p/'circle-common-dispatch-reduced-norm-build.jsonl').read_text().splitlines()]
assert items[-1]['success']
binary=next(x['executable'] for x in items if x.get('executable') and x['target']['kind']==['lib'])
names=subprocess.check_output([binary,'circle_dispatch_tests','--list'],text=True).splitlines()
names=[x.removesuffix(': test') for x in names if x.endswith(': test')]
assert len(names)==7,names
def run(name):
 stem='circle-common-dispatch-reduced-norm-'+name.rsplit('::',1)[-1]
 start=time.monotonic()
 with (p/(stem+'.log')).open('w') as out:
  try:r=subprocess.run([binary,name,'--exact','--nocapture'],stdout=out,stderr=subprocess.STDOUT,timeout=45);code=r.returncode
  except subprocess.TimeoutExpired:code=124
 record={'name':name,'returncode':code,'elapsed_seconds':time.monotonic()-start,'log':stem+'.log'}
 (p/(stem+'.json')).write_text(json.dumps(record,indent=2)+'\n')
 print(name,code,round(record['elapsed_seconds'],2),flush=True)
 if code:print((p/(stem+'.log')).read_text()[-5500:],flush=True)
 return record
with ThreadPoolExecutor(max_workers=2) as executor: results=list(executor.map(run,names))
(p/'circle-common-dispatch-reduced-norm-results.json').write_text(json.dumps(results,indent=2)+'\n')
raise SystemExit(any(r['returncode'] for r in results))
