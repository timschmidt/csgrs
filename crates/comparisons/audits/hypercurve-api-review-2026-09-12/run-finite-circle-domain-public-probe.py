from pathlib import Path
import subprocess,time,json,hashlib,sys
root=Path('/home/tim/Documents/GitHub/workspace');a=root/'hypercurve-api-review-2026-09-12';source=a/(sys.argv[1]+'.rs');stem=sys.argv[2];lib=Path(sys.argv[3]) if len(sys.argv)>3 else root/'hypercurve/target/release/deps/libhypercurve-2c7ff6f3fbe07653.rlib';sha=hashlib.sha256(lib.read_bytes()).hexdigest();deps=root/'hypercurve/target/release/deps'
cmd=['rustc','--edition=2024','-O',str(source),'--extern','hypercurve='+str(lib),'-L','dependency='+str(deps),'-o',str(a/stem)];r=subprocess.run(cmd,capture_output=True,text=True);(a/(stem+'-compile.log')).write_text(r.stdout+r.stderr)
if r.returncode:print(r.stderr);raise SystemExit(r.returncode)
start=time.monotonic()
with (a/(stem+'.log')).open('w') as out:
 try:r=subprocess.run([str(a/stem)],stdout=out,stderr=subprocess.STDOUT,timeout=120);code=r.returncode
 except subprocess.TimeoutExpired:code=124
lines=(a/(stem+'.log')).read_text().splitlines();data=dict(returncode=code,elapsed_seconds=time.monotonic()-start,command=cmd,source=source.name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),executable_sha256=hashlib.sha256((a/stem).read_bytes()).hexdigest(),normal_library_sha256=sha,library=str(lib),log=stem+'.log')
for line in reversed(lines):
 try:data['counts']=json.loads(line);break
 except ValueError:pass
assert hashlib.sha256(lib.read_bytes()).hexdigest()==sha
(a/(stem+'.json')).write_text(json.dumps(data,indent=2)+'\n');print(json.dumps(data,indent=2));print('\n'.join(lines[-12:]));raise SystemExit(code)
