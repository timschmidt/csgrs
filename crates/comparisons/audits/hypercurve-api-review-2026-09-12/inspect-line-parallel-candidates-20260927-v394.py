from pathlib import Path
import hashlib,json,os,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='inspect-line-parallel-candidates-20260927-v394';build=A/'build-workspace-20260925'
prior=json.loads((A/'trace-coincident-line-parallel-20260927-v392-terminal.json').read_text());assert prior['all_processes_reaped'];archive=Path(prior['source_directory']);trial=prior['trial'];manifest=json.loads((A/prior['source_manifest']).read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
source=A/f'{prefix}.rs';binary=A/prefix
report=dict(source_manifest=prior['source_manifest'],source_directory=str(archive),trial=trial,probe_sha256=sha(source),records=[],all_processes_reaped=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
def verify():
 for n,h in manifest.items():
  assert sha(W/n)==h,n
  for root in [archive,build]:assert sha(root/n)==trial.get(n,h),(root,n)
 assert sha(source)==report['probe_sha256']
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(label,cmd,cwd,timeout):
 log=A/f'{prefix}-{label}.log';start=time.monotonic()
 with log.open('w')as out:
  p=subprocess.Popen(cmd,cwd=cwd,env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
  try:code=p.wait(timeout=timeout)
  except BaseException:
   os.killpg(p.pid,signal.SIGKILL);p.wait();raise
 report['records'].append(dict(label=label,returncode=code,elapsed_seconds=time.monotonic()-start,command=cmd,log=log.name));save();print(label,code,round(time.monotonic()-start,2),flush=True)
 if code:print(log.read_text()[-2500:],flush=True);raise RuntimeError(label)
 return log
verify();code=0
try:
 library=Path(prior['library']['path']);assert sha(library)==prior['library']['sha256'];report['library']=prior['library'];deps=library.parent/'deps'
 run('probe-build',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-C','opt-level=3',str(source),'--extern',f'hypercurve={library}','-L',f'dependency={deps}','-o',str(binary)],A,180)
 report['binary']=dict(path=str(binary),sha256=sha(binary));log=run('probe',[str(binary)],A,120);print(log.read_text()[:5500],flush=True)
 assert sha(library)==report['library']['sha256'] and sha(binary)==report['binary']['sha256']
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;report['diagnostic_complete']=code==0;save()
raise SystemExit(code)
