from pathlib import Path
import hashlib,json,os,re,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-boundary-copy-20260927-v478';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'restricted-fiber-sign-20260927-v474-sources.json').read_text());assert not archive.exists()
for name,sha in prior.items():
 src=W/name;assert hashlib.sha256(src.read_bytes()).hexdigest()==sha,name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target);os.utime(target,None)
for root in [archive,build]:
 p=root/'hypercurve/src/error.rs';s=p.read_text();old='    pub(crate) const fn blocked(';assert s.count(old)==1
 s=s.replace(old,'    #[inline(never)]\n    pub(crate) fn blocked(')
 old='        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))';assert s.count(old)==1
 added="""        if operation == CurveOperation2::Fillet && reason == UncertaintyReason::Boundary {
            static CAPTURES: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            if CAPTURES.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 16 {
                eprintln!("BLOCKED_FILLET_BOUNDARY {}", std::backtrace::Backtrace::force_capture());
            }
        }
"""
 s=s.replace(old,added+old);p.write_text(s)
trial={'hypercurve/src/error.rs':hashlib.sha256((archive/'hypercurve/src/error.rs').read_bytes()).hexdigest()}
source=archive/'stationary-corner-contract.rs';shutil.copy2(A/'stationary-corner-contract-v476.rs',source);source_sha=hashlib.sha256(source.read_bytes()).hexdigest()
report=dict(diagnostic_only=True,source_manifest='restricted-fiber-sign-20260927-v474-sources.json',source_directory=str(archive),trial=trial,fixture_sha256=source_sha,checks=[],cases=[],all_processes_reaped=False,capture_complete=False)
def verify():
 for name,sha in prior.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==(sha if root==W else trial.get(name,sha)),(root,name)
 assert hashlib.sha256(source.read_bytes()).hexdigest()==source_sha

def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+chr(10))
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));env['RUST_BACKTRACE']='0';cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def run(command,cwd,log,timeout):
 start=time.monotonic()
 with log.open('w')as out:
  process=subprocess.Popen(command,cwd=cwd,stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:code=process.wait(timeout=timeout)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 return dict(command=command,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start)
verify();save();code=0
try:
 log=A/f'{prefix}-library-build.log';row=run([cargo,'build','--lib','--release','--all-features','--locked','--offline','--message-format=json'],build/'hypercurve',log,900);report['checks'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('diagnostic library build failed')
 artifacts=[]
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  if v.get('reason')=='compiler-artifact' and v.get('target',{}).get('name')=='hypercurve' and 'lib'in v['target']['kind']:artifacts.append(v)
 library=next(Path(p)for p in artifacts[-1]['filenames']if p.endswith('.rlib'));report['library']=dict(path=str(library),sha256=hashlib.sha256(library.read_bytes()).hexdigest());save()
 binary=A/f'{prefix}-tests';log=A/f'{prefix}-probe-build.log';row=run([env['RUSTC'],'--edition=2024','--test',str(source),'-C','opt-level=3','-L','dependency='+str(library.parent),'--extern','hypercurve='+str(library),'-o',str(binary)],archive,log,120);report['checks'].append(row);save()
 if row['returncode']:print(log.read_text()[-3000:],flush=True);raise RuntimeError('diagnostic fixture build failed')
 report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for name in ['stationary_reparameterization_strict','one_sided_cusp_strict','interior_stationary_contact_strict']:
  log=A/f'{prefix}-{name}.log';row=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive,log,45);row.update(name=name,stack_captured='BLOCKED_FILLET_BOUNDARY' in log.read_text());report['cases'].append(row);save();print(name,row['returncode'],row['stack_captured'],flush=True)
 report['capture_complete']=all(row['stack_captured']for row in report['cases'])
 code=0 if report['capture_complete']else 1
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
