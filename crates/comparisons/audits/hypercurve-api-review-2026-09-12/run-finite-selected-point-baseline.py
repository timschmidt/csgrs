from pathlib import Path
import hashlib,json,subprocess,time
p=Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12');stem='finite-selected-point-baseline'
source=p/'finite-selected-point-domains-public.rs';exe=p/stem;lib=Path(json.loads((p/'finite-fixed-distance-domains-qualification.json').read_text())['archived_library'])
hashfile=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(source),'--extern','hypercurve='+str(lib),'-L','dependency=/tmp/hypercurve-pruning-qualification/hyperbrep/target/release/deps','-o',str(exe)]
with (p/(stem+'-compile.log')).open('w') as log: result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=120)
assert result.returncode==0
record=dict(commit='34ae22f1ca9059356b2a6a1fcb3f9263b2c46577',source=source.name,source_sha256=hashfile(source),executable=exe.name,executable_sha256=hashfile(exe),normal_library=str(lib),normal_library_sha256=hashfile(lib),compile_command=cmd,queries=[])
for repeat in [False]:
 for chart in range(4):
  command=[str(exe),str(chart),*(['repeat'] if repeat else [])];log=p/(stem+f'-chart{chart}-repeat{int(repeat)}.log');start=time.monotonic()
  with log.open('w') as output:
   try:code=subprocess.run(command,stdout=output,stderr=subprocess.STDOUT,timeout=60).returncode
   except subprocess.TimeoutExpired:code=124
  row=dict(chart=chart,repeat=repeat,command=command,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name)
  for line in reversed(log.read_text().splitlines()):
   try:counts=json.loads(line)
   except json.JSONDecodeError:continue
   if isinstance(counts,dict):row['counts']=counts;break
  record['queries'].append(row);(p/(stem+'.json')).write_text(json.dumps(record,indent=2)+'\n');print(row,flush=True)
assert hashfile(source)==record['source_sha256'] and hashfile(lib)==record['normal_library_sha256']
