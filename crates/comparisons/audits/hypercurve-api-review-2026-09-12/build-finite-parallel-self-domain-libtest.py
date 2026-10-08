from pathlib import Path
import os,subprocess,time,json,hashlib,sys
root=Path('/home/tim/Documents/GitHub/workspace');a=root/'hypercurve-api-review-2026-09-12';stem=sys.argv[1]
patch=subprocess.check_output(['git','diff'],cwd=root/'hypercurve',text=True);(a/(stem+'.patch')).write_text(patch)
command=['cargo','test','--release','--all-features','--offline','--lib','--no-run','--message-format=json']
start=time.monotonic()
with (a/(stem+'.jsonl')).open('w') as out,(a/(stem+'.log')).open('w') as err:
 r=subprocess.run(command,cwd=root/'hypercurve',env=dict(os.environ,CARGO_BUILD_JOBS='2'),stdout=out,stderr=err,timeout=300)
(a/(stem+'.exit')).write_text(str(r.returncode)+'\n')
print(r.returncode,time.monotonic()-start,flush=True)
if r.returncode:
 for l in (a/(stem+'.jsonl')).read_text().splitlines():
  x=json.loads(l)
  if x.get('reason')=='compiler-message' and x['message']['level']=='error':print(x['message']['rendered'])
 raise SystemExit(r.returncode)
artifacts=[json.loads(l) for l in (a/(stem+'.jsonl')).read_text().splitlines()]
b=Path(next(x['executable'] for x in artifacts if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and x['profile']['test']))
for test in sys.argv[2:]:
 name=stem+'-'+test.rsplit('::',1)[-1];command=[str(b),'--exact',test,'--nocapture','--color','never'];start=time.monotonic()
 with (a/(name+'.log')).open('w') as out:r=subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,timeout=120)
 record=dict(returncode=r.returncode,elapsed_seconds=time.monotonic()-start,command=command,binary=str(b),sha256=hashlib.sha256(b.read_bytes()).hexdigest(),source_patch_sha256=hashlib.sha256(patch.encode()).hexdigest())
 (a/(name+'.json')).write_text(json.dumps(record,indent=2)+'\n');print(record,flush=True);print((a/(name+'.log')).read_text(),flush=True)
