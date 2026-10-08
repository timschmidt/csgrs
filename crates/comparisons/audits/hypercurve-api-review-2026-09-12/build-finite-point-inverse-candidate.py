from pathlib import Path
import os,shutil,subprocess,json,time,sys,hashlib
w=Path('/home/tim/Documents/GitHub/workspace');a=w/'hypercurve-api-review-2026-09-12';r=Path('/tmp/hypercurve-point-inverse-qualification/hypercurve');p='finite-point-inverse-domains-candidate'+sys.argv[1]
shutil.copy2(w/'hypercurve/src/bezier_offset.rs',r/'src/bezier_offset.rs')
sha=hashlib.sha256((r/'src/bezier_offset.rs').read_bytes()).hexdigest()
settings=json.loads((a/'finite-point-inverse-domains-build-settings.json').read_text());env=dict(os.environ,**settings)
cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo','test','--release','--all-features','--locked','--offline','--lib','--no-run','--message-format=json'];start=time.monotonic()
with (a/(p+'.log')).open('w') as err,(a/(p+'.jsonl')).open('w') as out: result=subprocess.run(cmd,cwd=r,env=env,stdout=out,stderr=err,timeout=500)
assert hashlib.sha256((r/'src/bezier_offset.rs').read_bytes()).hexdigest()==sha
(a/(p+'.json')).write_text(json.dumps(dict(command=cmd,source_sha256=sha,returncode=result.returncode,elapsed_seconds=time.monotonic()-start),indent=2)+'\n')
print(result.returncode,(a/(p+'.log')).read_text()[-3000:])
raise SystemExit(result.returncode)
