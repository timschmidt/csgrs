from pathlib import Path
import hashlib,json,subprocess,time
p=Path('.').resolve();q=json.loads((p/'finite-parallel-admission-qualification.json').read_text());lib=Path(q['archived_library']);stem='finite-analytic-pair-domains-baseline';source=p/'finite-analytic-pair-domains-public.rs'
assert not (p/(stem+'.json')).exists()
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(source),'--extern','hypercurve='+str(lib),'-L','dependency='+str(Path(q['normal_library']).parent),'-o',str(p/stem)]
r=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=120);(p/(stem+'-compile.log')).write_text(r.stdout)
print('compile',r.returncode,r.stdout[-3000:],flush=True)
if r.returncode:raise SystemExit(r.returncode)
start=time.monotonic()
with (p/(stem+'.log')).open('w') as log:
 try:code=subprocess.run([str(p/stem)],stdout=log,stderr=subprocess.STDOUT,timeout=120).returncode
 except subprocess.TimeoutExpired:code=124
record=dict(commit=q['commit'],command=command,returncode=code,elapsed_seconds=time.monotonic()-start,source=source.name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),binary_sha256=hashlib.sha256((p/stem).read_bytes()).hexdigest(),normal_library_sha256=q['normal_library_sha256'],log=stem+'.log')
(p/(stem+'.json')).write_text(json.dumps(record,indent=2)+'\n');print(record)
print((p/(stem+'.log')).read_text()[-2500:])
