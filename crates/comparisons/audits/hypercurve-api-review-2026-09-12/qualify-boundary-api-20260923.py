from pathlib import Path
import hashlib,json,os,subprocess,time

a=Path(__file__).resolve().parent
w=a.parent
r=Path('/tmp/hypercurve-boundary-api-2026-09-23')
prefix='boundary-api-20260923-check5'
changes=json.loads((a/'boundary-api-20260923-changed-files.json').read_text())
working={repo+'/'+name:hashlib.sha256((w/repo/name).read_bytes()).hexdigest() for repo,files in changes.items() for name in files}
manifest={str(p.relative_to(r)):hashlib.sha256(p.read_bytes()).hexdigest() for repo in r.iterdir() for p in repo.rglob('*') if p.is_file() and 'target' not in p.parts}
(a/(prefix+'-sources.json')).write_text(json.dumps(dict(working=working,isolated=manifest),indent=2)+'\n')
def verify():
 for name,h in working.items(): assert hashlib.sha256((w/name).read_bytes()).hexdigest()==h,name
 for name,h in manifest.items(): assert hashlib.sha256((r/name).read_bytes()).hexdigest()==h,name

env=dict(os.environ,**json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
jobs=[('hypercurve',['check','--all-targets','--all-features']),('hyperbrep',['check','--all-targets','--all-features']),('csgrs',['check','--example','readme_renders','--features','offset'])]
rows=[]
for repo,args in jobs:
 log=a/(prefix+'-'+repo+'.log'); cmd=[cargo,*args,'--locked','--offline']; start=time.monotonic()
 with log.open('w') as out:
  try: code=subprocess.run(cmd,cwd=r/repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
  except subprocess.TimeoutExpired: code='timeout'
 verify(); row=dict(repo=repo,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name); rows.append(row); (a/(prefix+'-runs.json')).write_text(json.dumps(rows,indent=2)+'\n'); print(json.dumps(row),flush=True)
 if code: print(log.read_text()[-9500:],flush=True); break
print('Checks terminal; all working and isolated sources unchanged.',flush=True)
