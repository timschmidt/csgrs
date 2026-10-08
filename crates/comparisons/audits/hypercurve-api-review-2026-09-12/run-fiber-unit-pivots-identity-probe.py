from pathlib import Path
import hashlib,json,subprocess,time
p=Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12');source=p/'fiber-unit-pivots-identity-probe.rs'
for label,build in [('parent','finite-projective-parameter-test-build'),('candidate','mapped-point-inverse-candidate15')]:
 entries=[json.loads(l) for l in (p/(build+'.jsonl')).read_text().splitlines()]
 libraries={name:Path(next(f for x in entries if x.get('reason')=='compiler-artifact' and x['target']['name']==name and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib'))) for name in ['hypersolve','hyperreal','hyperlimit']}
 stem='fiber-unit-pivots-identity-'+label;exe=p/stem
 cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(source),'-L','dependency='+str(libraries['hypersolve'].parent),'-o',str(exe)]
 for name,path in libraries.items():cmd+=['--extern',name+'='+str(path)]
 with (p/(stem+'-compile.log')).open('w') as out:code=subprocess.run(cmd,stdout=out,stderr=subprocess.STDOUT,timeout=120).returncode
 assert code==0,(p/(stem+'-compile.log')).read_text()
 start=time.monotonic()
 with (p/(stem+'.log')).open('w') as out:
  try:code=subprocess.run([str(exe)],stdout=out,stderr=subprocess.STDOUT,timeout=60).returncode
  except subprocess.TimeoutExpired:code=124
 row=dict(compile_command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),binary_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),libraries={name:dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for name,path in libraries.items()})
 (p/(stem+'.json')).write_text(json.dumps(row,indent=2)+'\n');print(label,code,(p/(stem+'.log')).read_text(),flush=True)
