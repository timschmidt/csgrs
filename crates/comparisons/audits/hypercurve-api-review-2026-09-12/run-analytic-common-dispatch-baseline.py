from pathlib import Path
import subprocess,json,time,hashlib
p=Path.cwd();root=p.parent;lib=root/'hypercurve/target/release/deps/libhypercurve-2c7ff6f3fbe07653.rlib';stem='analytic-common-dispatch-baseline';source=p/(stem+'.rs');binary=p/stem
sha=lambda x:hashlib.sha256(x.read_bytes()).hexdigest()
cmd=['rustc','--edition=2024','-O',str(source),'--extern','hypercurve='+str(lib),'-L','dependency='+str(lib.parent),'-o',str(binary)]
r=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=120);(p/(stem+'-compile.log')).write_text(r.stdout)
record={'source_sha256':sha(source),'normal_library_sha256':sha(lib),'compile_returncode':r.returncode,'command':cmd,'baseline_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root/'hypercurve',text=True).strip()}
if r.returncode==0:
 start=time.monotonic()
 with (p/(stem+'.log')).open('w') as out:
  try:record['returncode']=subprocess.run([str(binary)],stdout=out,stderr=subprocess.STDOUT,timeout=120).returncode
  except subprocess.TimeoutExpired:record['returncode']=124
 record.update(elapsed_seconds=time.monotonic()-start,binary_sha256=sha(binary));print((p/(stem+'.log')).read_text())
else:print(r.stdout)
(p/(stem+'.json')).write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record,indent=2))
