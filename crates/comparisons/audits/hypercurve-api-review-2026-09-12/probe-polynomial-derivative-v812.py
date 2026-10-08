from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='polynomial-derivative-probe-v812'
assert json.loads((A/'polynomial-derivative-v811-reaped.json').read_text())['outer_exit_code']==0
baseline=json.loads((A/'polynomial-derivative-v811-terminal.json').read_text());manifest=json.loads((A/baseline['source_manifest']).read_text())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(baseline['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
verify();rlibs=[];linked_paths=set()
for build in baseline['builds'][-1:]:
 for line in(A/build['log']).read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  if v.get('reason')=='compiler-artifact'and v.get('target',{}).get('name')=='hypercurve':rlibs.extend(Path(p)for p in v.get('filenames',[])if p.endswith('.rlib'))
  if v.get('reason')=='build-script-executed':linked_paths.update(v.get('linked_paths',[]))
assert len(rlibs)==1,rlibs
rlib=rlibs[0];rlib_sha=digest(rlib);source=A/'polynomial-derivative-probe-v809.rs';source_sha=digest(source);binary=A/f'{prefix}-binary';report=dict(baseline=baseline['source_manifest'],rlib=str(rlib),rlib_sha256=rlib_sha,source_sha256=source_sha,processes=[],all_processes_reaped=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
def run(command,label,timeout):
 log=A/f'{prefix}-{label}.log';start=time.monotonic()
 with log.open('w')as out:
  child=subprocess.Popen(command,cwd=A,env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
  try:code=child.wait(timeout=timeout)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait();code=124
  except BaseException:os.killpg(child.pid,signal.SIGKILL);child.wait();raise
 row=dict(command=command,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name);report['processes'].append(row)
 print(label,code,round(row['elapsed_seconds'],3),flush=True)
 if code:print(log.read_text()[-3000:],flush=True);raise RuntimeError(label+' failed')
 return log.read_text()
code=0
try:
 run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','--crate-name','polynomial_derivative_probe',str(source),'--extern','hypercurve='+str(rlib),'-L','dependency='+str(rlib.parent),*[arg for p in sorted(linked_paths)for arg in ['-L',p]],'-C','opt-level=3','-o',str(binary)],'compile',120)
 report['binary_sha256']=digest(binary);output=run([str(binary)],'run',120)
 assert 'polynomial_derivative_probe_complete'in output
 assert output==(A/'polynomial-derivative-probe-v809-run.log').read_text().replace('native=XImageFailed','native=Transformed').replace('endpoint_exact=false','endpoint_exact=true')
 report['probe_complete']=True;print(output,flush=True)
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:
 verify();assert digest(rlib)==rlib_sha and digest(source)==source_sha;report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(code)
