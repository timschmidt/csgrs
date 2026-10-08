from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-fiber-refinement-20260926-v247';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'public-fillet-families-full-20260926-v236-sources.json').read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha or name=='hypersolve/src/algebraic_fiber.rs',name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),cases=[],builds=[],all_processes_reaped=False)
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name

def save():
 verify();(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def compile_binary(crate,target,selector):
 cmd=[cargo,'test',*selector,'--release','--all-features','--no-run','--message-format=json','--locked','--offline'];log=A/f'{prefix}-{crate}-build.log';start=time.monotonic()
 with log.open('w')as out:code=subprocess.run(cmd,cwd=build/crate,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
 record=dict(crate=crate,command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start);report['builds'].append(record);save()
 if code:
  for line in log.read_text().splitlines():
   try:r=json.loads(line)
   except ValueError:continue
   if r.get('reason')=='compiler-message'and r['message']['level']=='error':print(r['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('build failure')
 rows=[]
 for line in log.read_text().splitlines():
  try:rows.append(json.loads(line))
  except ValueError:pass
 row=next(r for r in rows if r.get('reason')=='compiler-artifact'and r['target']['name']==target and r.get('executable'))
 binary=A/f'{prefix}-{target}';shutil.copy2(row['executable'],binary);record['binary']=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());save();return binary

def case(binary,name):
 log=A/f'{prefix}-case-{len(report["cases"]):03d}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=240).returncode
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(binary=binary.name,name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));save();print(name,code,round(time.monotonic()-start,2),flush=True)
 if code:print(log.read_text()[-3000:],flush=True);raise RuntimeError('case failure')
verify();code=0
try:
 binary=compile_binary('hypersolve','hypersolve',['--lib'])
 focus=['algebraic_fiber::tests::selected_fiber_refines_nonrational_coefficients_before_subdividing_clusters','algebraic_fiber::tests::selected_fiber_accepts_exact_real_base_coefficients']
 names=subprocess.check_output([str(binary),'--list'],text=True).splitlines();names=[s.removesuffix(': test')for s in names if s.startswith('algebraic_fiber::')and s.endswith(': test')]
 for name in focus+[name for name in names if name not in focus]:case(binary,name)
 binary=compile_binary('hypercurve','hypercurve_curve_region_promotion',['--test','hypercurve_curve_region_promotion'])
 for name in ['strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','strict_trim_only_analytic_parallel_support_corners_retain_algebraic_fillet_centers']:
  case(binary,name)
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 report['all_processes_reaped']=True;report['passed']=sum(c['returncode']==0 for c in report['cases']);report['qualification_complete']=code==0;save();print('qualification_complete',code==0,'passed',report['passed'],flush=True)
raise SystemExit(code)
