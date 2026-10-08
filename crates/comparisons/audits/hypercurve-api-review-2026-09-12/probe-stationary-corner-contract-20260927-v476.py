from pathlib import Path
from fractions import Fraction as F
import hashlib,json,os,re,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-corner-contract-20260927-v476'
qualified=json.loads((A/'restricted-fiber-sign-20260927-v474-terminal.json').read_text())
assert qualified['qualification_complete'] and qualified['all_processes_reaped']
committed=json.loads((A/'restricted-fiber-sign-20260927-v474-committed.json').read_text())
manifest=json.loads((A/qualified['source_manifest']).read_text());source=A/'stationary-corner-contract-v476.rs';source_sha=hashlib.sha256(source.read_bytes()).hexdigest()
def verify():
 assert hashlib.sha256(source.read_bytes()).hexdigest()==source_sha
 for name,sha in manifest.items():
  for root in [W,Path(qualified['source_directory'])]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for name,record in committed['repos'].items():assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/name,text=True).strip()==record['commit']
verify()
log=A/'restricted-fiber-sign-20260927-v474-hypercurve_curve_region_promotion-build.log'
rows=[]
for line in log.read_text().splitlines():
 try:row=json.loads(line)
 except ValueError:continue
 if row.get('reason')=='compiler-artifact' and row.get('target',{}).get('name')=='hypercurve' and 'lib' in row['target']['kind']:rows.append(row)
row=rows[-1];library=next(Path(p)for p in row['filenames']if p.endswith('.rlib'));library_sha=hashlib.sha256(library.read_bytes()).hexdigest()
library_receipt=json.loads((A/'restricted-fiber-sign-20260927-v474-library.json').read_text())
assert library_receipt['path']==str(library) and library_receipt['sha256']==library_sha and library_receipt['source_sha256']==manifest['hypercurve/src/bezier_offset.rs']
report=dict(diagnostic_only=True,source_sha256=source_sha,source_manifest=qualified['source_manifest'],library=dict(path=str(library),sha256=library_sha),cases=[],all_processes_reaped=False,probe_complete=False)
# Independent rational contacts, tangents and circle equations.
for x,y,cx,cy,r,tx,ty in [(F(3,8),F(9,64),F(15,16),F(-39,64),F(15,16),F(1),F(3,4)),(F(1,4),F(1,8),F(5,8),F(-3,8),F(5,8),F(1),F(3,4))]:
 assert (x-cx)**2+(y-cy)**2==r*r and (x-cx)*tx+(y-cy)*ty==0 and cx==r
 assert -2<cy<0 and 0<x<1 and 0<y<1
# At the interior cusp the retained left branch has limiting tangent -x.
# The CCW unit circle centered at (0,-1) then starts at (0,0) and
# meets the line (1,1)+s*(-4,-3) at s=2/5 with the same oriented tangent.
x,y=F(-3,5),F(-1,5)
assert x*x+(y+1)**2==1 and x*F(-4)+(y+1)*F(-3)==0
assert (x,y)==(1-F(2,5)*4,1-F(2,5)*3)
assert F(-1)*F(-3,5)>0
report['interior_contact_oriented_one_sided_frame']=True
report['independent_rational_contacts_and_tangents']=True
names=['regular_parabola_strict','stationary_reparameterization_strict','one_sided_cusp_strict','regular_parabola_approximate','stationary_reparameterization_approximate','one_sided_cusp_approximate','interior_stationary_contact_strict','interior_stationary_contact_approximate'];report['expected_cases']=names
binary=A/f'{prefix}-tests';env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(cmd,out,timeout):
 process=subprocess.Popen(cmd,cwd=W,env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
 try:return process.wait(timeout=timeout)
 except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();return 124
 except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
code=0
try:
 compile_log=A/f'{prefix}-build.log';command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','--test',str(source),'-C','opt-level=3','-L','dependency='+str(library.parent),'--extern','hypercurve='+str(library),'-o',str(binary)]
 with compile_log.open('w')as out:status=run(command,out,120)
 report['build']=dict(command=command,returncode=status,log=compile_log.name);save()
 if status:print(compile_log.read_text()[-3000:],flush=True);raise RuntimeError('probe compilation failed')
 report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest();save()
 for name in names:
  test_log=A/f'{prefix}-{name}.log';started=time.monotonic()
  with test_log.open('w')as out:status=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],out,90)
  passed=status==0 and re.search(r'test result: ok\. 1 passed;',test_log.read_text()) is not None
  report['cases'].append(dict(name=name,returncode=status,passed=passed,elapsed_seconds=time.monotonic()-started,log=test_log.name));save()
  print(name,'pass'if passed else 'unresolved',round(time.monotonic()-started,3),flush=True)
 report['probe_complete']=True
 code=0 if all(c['passed']for c in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error)
finally:
 verify();assert hashlib.sha256(library.read_bytes()).hexdigest()==library_sha
 report['all_processes_reaped']=True;save()
raise SystemExit(code)
