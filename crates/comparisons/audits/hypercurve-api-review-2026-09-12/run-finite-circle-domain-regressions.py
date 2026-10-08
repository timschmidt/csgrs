from pathlib import Path
import hashlib,json,os,subprocess,sys,time
root=Path('/home/tim/Documents/GitHub/workspace');a=root/'hypercurve-api-review-2026-09-12';repo=root/'hypercurve';stem='finite-circle-domain-'+sys.argv[1]
(a/(stem+'.patch')).write_bytes(subprocess.check_output(['git','diff'],cwd=repo))
files=subprocess.check_output(['git','diff','--name-only'],cwd=repo,text=True).splitlines()
sources={f:hashlib.sha256((repo/f).read_bytes()).hexdigest() for f in files}
(a/(stem+'-sources.json')).write_text(json.dumps(sources,indent=2)+'\n')
start=time.monotonic()
with (a/(stem+'-build.log')).open('w') as log, (a/(stem+'-build.jsonl')).open('w') as data:
 r=subprocess.run(['cargo','test','--release','--all-features','--offline','--lib','--no-run','--message-format=json'],cwd=repo,env=dict(os.environ,CARGO_BUILD_JOBS='2'),stdout=data,stderr=log,timeout=600)
(a/(stem+'-build.exit')).write_text(str(r.returncode)+'\n')
print('build',r.returncode,round(time.monotonic()-start,2),flush=True)
if r.returncode:
 print((a/(stem+'-build.log')).read_text()[-8000:]);raise SystemExit(r.returncode)
rows=[json.loads(line) for line in (a/(stem+'-build.jsonl')).read_text().splitlines()]
exe=next(r['executable'] for r in rows if r.get('reason')=='compiler-artifact' and r.get('executable') and r['target']['name']=='hypercurve')
results=[]
for name in ['bezier_region::tests::selected_circle_and_retained_rational_arc_fillet_exactly','bezier_region::tests::selected_circle_and_retained_rational_arc_extend_on_full_supports']:
 suffix=name.rsplit('::',1)[-1];cmd=[exe,'--exact',name,'--test-threads=1','--nocapture'];start=time.monotonic()
 with (a/(stem+'-'+suffix+'.log')).open('w') as log:
  try:r=subprocess.run(cmd,cwd=repo,stdout=log,stderr=subprocess.STDOUT,env=dict(os.environ,HYPERCURVE_DEBUG_RATIONAL_BLOCKER='1'),timeout=90);code=r.returncode
  except subprocess.TimeoutExpired:code=124
 row={'command':cmd,'returncode':code,'elapsed_seconds':time.monotonic()-start,'binary':exe,'sha256':hashlib.sha256(Path(exe).read_bytes()).hexdigest(),'log':stem+'-'+suffix+'.log'}
 results.append(row);(a/(stem+'-focused.json')).write_text(json.dumps(results,indent=2)+'\n');print(suffix,code,round(row['elapsed_seconds'],2),flush=True)
 if code:print((a/row['log']).read_text()[-18000:],flush=True)
assert all(hashlib.sha256((repo/f).read_bytes()).hexdigest()==sha for f,sha in sources.items())
raise SystemExit(int(any(r['returncode'] for r in results)))
