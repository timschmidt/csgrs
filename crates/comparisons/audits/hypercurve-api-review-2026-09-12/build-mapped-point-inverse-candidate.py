from pathlib import Path
import os,shutil,subprocess,json,time,sys,hashlib
w=Path('/home/tim/Documents/GitHub/workspace');a=w/'hypercurve-api-review-2026-09-12';r=Path('/tmp/hypercurve-mapped-point-qualification/hypercurve');p='mapped-point-inverse-candidate'+sys.argv[1]
for name in ['bezier_offset.rs', 'rational_bezier_general.rs', 'bezier_parameter.rs', 'bezier_split.rs']:
 shutil.copy2(w/'hypercurve/src'/name,r/'src'/name)
 shutil.copy2(r/'src'/name,a/(p+'-'+name))
for name in ['curve_resultant.rs', 'bareiss.rs', 'algebraic_fiber.rs', 'lib.rs', 'algebraic_fiber/subresultant.rs']:
 shutil.copy2(w/'hypersolve/src'/name,r.parent/'hypersolve/src'/name)
 shutil.copy2(r.parent/'hypersolve/src'/name,a/(p+'-hypersolve-'+name.replace('/', '-')))
source_files={name:hashlib.sha256((r/'src'/name).read_bytes()).hexdigest() for name in ['bezier_offset.rs', 'rational_bezier_general.rs', 'bezier_parameter.rs', 'bezier_split.rs']}
sha=hashlib.sha256((r/'src/bezier_offset.rs').read_bytes()).hexdigest()
settings=json.loads((a/'finite-point-inverse-domains-build-settings.json').read_text());env=dict(os.environ,**settings)
cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo','test','--release','--all-features','--locked','--offline','--lib','--no-run','--message-format=json'];start=time.monotonic()
with (a/(p+'.log')).open('w') as err,(a/(p+'.jsonl')).open('w') as out: result=subprocess.run(cmd,cwd=r,env=env,stdout=out,stderr=err,timeout=500)
for name,value in source_files.items(): assert hashlib.sha256((r/'src'/name).read_bytes()).hexdigest()==value
(a/(p+'.json')).write_text(json.dumps(dict(command=cmd,source_sha256=sha,hypersolve_source_files={name:hashlib.sha256((r.parent/'hypersolve/src'/name).read_bytes()).hexdigest() for name in ['algebraic_fiber.rs', 'lib.rs', 'algebraic_fiber/subresultant.rs']},source_files=source_files,returncode=result.returncode,elapsed_seconds=time.monotonic()-start),indent=2)+'\n')
print(result.returncode,(a/(p+'.log')).read_text()[-3000:])
raise SystemExit(result.returncode)
