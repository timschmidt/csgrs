from pathlib import Path
import subprocess,time,json,hashlib,os
root=Path('/home/tim/Documents/GitHub/workspace');a=root/'hypercurve-api-review-2026-09-12';source=json.loads((a/'finite-circle-rational-domains-control-source.json').read_text());repo=Path(source['repo']);stem='finite-circle-rational-domains-control';cmd=['cargo','build','--release','--all-features','--lib','--offline','--message-format=json'];start=time.monotonic()
with (a/(stem+'-build.jsonl')).open('w') as out,(a/(stem+'-build.log')).open('w') as err:
 r=subprocess.run(cmd,cwd=repo,stdout=out,stderr=err,env=dict(os.environ,CARGO_BUILD_JOBS='2',CARGO_TARGET_DIR=str(root/'hypercurve/target')),timeout=600)
(a/(stem+'-build.exit')).write_text(str(r.returncode)+'\n');print('control build',r.returncode,round(time.monotonic()-start,2),flush=True)
if r.returncode:print((a/(stem+'-build.log')).read_text()[-4000:]);raise SystemExit(r.returncode)
rows=[json.loads(line) for line in (a/(stem+'-build.jsonl')).read_text().splitlines() if line.startswith('{')];lib=next(Path(file) for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='hypercurve' for file in row['filenames'] if file.endswith('.rlib'))
for file,sha in source['sources'].items():assert hashlib.sha256((repo/file).read_bytes()).hexdigest()==sha,file
print('control library',lib,flush=True)
subprocess.run(['python3',str(a/'run-finite-circle-domain-public-probe.py'),'finite-circle-rational-domains-public',stem+'-boolean',str(lib)],cwd=root/'hypercurve',check=False)
