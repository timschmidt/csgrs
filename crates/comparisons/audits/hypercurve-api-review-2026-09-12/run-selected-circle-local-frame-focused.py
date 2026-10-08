from pathlib import Path
import hashlib,json,os,subprocess,sys,time
root=Path('/home/tim/Documents/GitHub/workspace')
audit=root/'hypercurve-api-review-2026-09-12'
label=sys.argv[1]
stem='selected-circle-local-frame-'+label
start=time.monotonic()
with (audit/(stem+'-build.jsonl')).open('w') as out,(audit/(stem+'-build.log')).open('w') as err:
    r=subprocess.run(['cargo','test','--release','--all-features','--offline','--lib','--no-run','--message-format=json'],cwd=root/'hypercurve',env=dict(os.environ,CARGO_BUILD_JOBS='2'),stdout=out,stderr=err,timeout=600)
(audit/(stem+'-build.exit')).write_text(str(r.returncode)+'\n')
items=[json.loads(l) for l in (audit/(stem+'-build.jsonl')).read_text().splitlines()]
print('build',r.returncode,'seconds',round(time.monotonic()-start,2),flush=True)
if r.returncode:
    for x in items:
        if x.get('reason')=='compiler-message' and x['message']['level']=='error': print(x['message'].get('rendered',x['message']['message']),flush=True)
    raise SystemExit(r.returncode)
binary=next(x['executable'] for x in items if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and x['profile']['test'])
name='curve_intersection::curve_support_intersection::circle_dispatch_tests::selected_circle_retains_only_its_finite_source_frame'
command=[binary,name,'--exact','--test-threads=1']
start=time.monotonic()
with (audit/(stem+'.log')).open('w') as out:
    try: code=subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,timeout=90).returncode
    except subprocess.TimeoutExpired: code=124
record={'returncode':code,'command':command,'binary_sha256':hashlib.sha256(Path(binary).read_bytes()).hexdigest(),'elapsed_seconds':time.monotonic()-start,'log':stem+'.log'}
(audit/(stem+'.json')).write_text(json.dumps(record,indent=2)+'\n')
print(record,flush=True)
print((audit/(stem+'.log')).read_text()[-6000:],flush=True)
