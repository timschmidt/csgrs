from pathlib import Path
import subprocess,time,json,hashlib,os
root=Path('/home/tim/Documents/GitHub/workspace')
audit=root/'hypercurve-api-review-2026-09-12'
repo=Path('/tmp/hypercurve-region-carrier-evidence-baseline/hypercurve')
start=time.monotonic()
with (audit/'region-carrier-evidence-baseline-build.log').open('w') as out:
 r=subprocess.run(['cargo','build','--release','--lib','--all-features','--offline','--locked'],cwd=repo,env=dict(os.environ,CARGO_BUILD_JOBS='2'),stdout=out,stderr=subprocess.STDOUT,timeout=600)
assert r.returncode==0, (audit/'region-carrier-evidence-baseline-build.log').read_text()[-3000:]
print('baseline library built',round(time.monotonic()-start,2),flush=True)
rows=[]
for name,lib in [('before',next((repo/'target/release/deps').glob('libhypercurve-*.rlib'))),('after',root/'hypercurve/target/release/deps/libhypercurve-2c7ff6f3fbe07653.rlib')]:
 exe=audit/('region-carrier-evidence-trim-control-'+name)
 sha=hashlib.sha256(lib.read_bytes()).hexdigest()
 r=subprocess.run(['rustc','--edition=2024','-O',str(audit/'region-carrier-evidence-trim-control.rs'),'--extern','hypercurve='+str(lib),'-L','dependency='+str(lib.parent),'-o',str(exe)],capture_output=True,text=True,timeout=120)
 assert r.returncode==0,r.stderr
 start=time.monotonic()
 with exe.with_suffix('.log').open('w') as out:
  try:
   r=subprocess.run([str(exe)],stdout=out,stderr=subprocess.STDOUT,timeout=90)
   code=r.returncode
  except subprocess.TimeoutExpired: code=124
 row={'library':name,'normal_library_sha256':sha,'returncode':code,'elapsed_seconds':time.monotonic()-start,'log':exe.name+'.log'}
 rows.append(row)
 print(row,exe.with_suffix('.log').read_text()[-500:],flush=True)
(audit/'region-carrier-evidence-trim-control.json').write_text(json.dumps(rows,indent=2)+'\n')
