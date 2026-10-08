from pathlib import Path
import hashlib,json,subprocess,time
a=Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
stem='finite-circle-analytic-domains-projection-check'
source=a/'finite-circle-analytic-domains-public.rs'
items=[json.loads(line) for line in (a/'finite-circle-projection-test-build.jsonl').read_text().splitlines()]
lib=Path(next(f for x in items if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(source),'--extern','hypercurve='+str(lib),'-L','dependency='+str(lib.parent),'-o',str(a/stem)]
with (a/(stem+'-compile.log')).open('w') as out:
    result=subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,timeout=120)
assert result.returncode==0
start=time.monotonic()
with (a/(stem+'.log')).open('w') as out:
    result=subprocess.run([str(a/stem)],stdout=out,stderr=subprocess.STDOUT,timeout=120)
counts=None
for line in reversed((a/(stem+'.log')).read_text().splitlines()):
    try:counts=json.loads(line);break
    except json.JSONDecodeError:pass
report=dict(status='expected remaining public guard; not a passing qualification',returncode=result.returncode,elapsed_seconds=time.monotonic()-start,counts=counts,source=source.name,source_sha256=digest(source),normal_library=str(lib),normal_library_sha256=digest(lib),executable=stem,executable_sha256=digest(a/stem),command=command)
(a/(stem+'.json')).write_text(json.dumps(report,indent=2)+'\n')
assert result.returncode==101 and counts==dict(cases=32,contacts=16,point_replays=32,failures=16),report
print(json.dumps(report),flush=True)
