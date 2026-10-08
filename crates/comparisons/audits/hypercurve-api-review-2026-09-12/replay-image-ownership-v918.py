from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='image-ownership-replay-v918';build=A/'build-workspace-20260925'
assert json.loads((A/'image-ownership-v917-reaped.json').read_text())['outer_exit_code']==0
baseline=json.loads((A/'image-ownership-v917-terminal.json').read_text());manifest=json.loads((A/baseline['source_manifest']).read_text())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(baseline['source_directory']),build]:assert digest(root/name)==sha,(root,name)
source=A/'image-ownership-probe-v916.rs';source_sha=digest(source);binary=A/f'{prefix}-binary'
report=dict(baseline=baseline['source_manifest'],source_sha256=source_sha,processes=[],all_processes_reaped=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
def run(command,label,timeout,cwd):
 log=A/f'{prefix}-{label}.log';start=time.monotonic()
 with log.open('w')as out:
  child=subprocess.Popen(command,cwd=cwd,env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
  try:code=child.wait(timeout=timeout)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait();code=124
  except BaseException:os.killpg(child.pid,signal.SIGKILL);child.wait();raise
 row=dict(command=command,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name);report['processes'].append(row)
 print(label,code,round(row['elapsed_seconds'],3),flush=True)
 if code:print(log.read_text()[-3000:],flush=True);raise RuntimeError(label+' failed')
 return log.read_text()
verify();code=0;rlibs={}
try:
 output='\n'.join((A/entry['log']).read_text()for entry in baseline['builds'])
 libraries={};linked_paths=set()
 for line in output.splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  name=v.get('target',{}).get('name')
  if v.get('reason')=='compiler-artifact'and name in {'hypersolve','hyperreal'}:
   paths=[Path(p)for p in v.get('filenames',[])if p.endswith('.rlib')]
   if paths:assert len(paths)==1;libraries[name]=paths[0]
  if v.get('reason')=='build-script-executed':linked_paths.update(v.get('linked_paths',[]))
 assert set(libraries)=={'hypersolve','hyperreal'}
 rlibs={str(p):digest(p)for p in libraries.values()};report['rlibs']=rlibs
 run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','--crate-name','image_ownership_probe',str(source),*[arg for name,p in libraries.items()for arg in ['--extern',name+'='+str(p)]],*[arg for p in sorted({v.parent for v in libraries.values()})for arg in ['-L','dependency='+str(p)]],*[arg for p in sorted(linked_paths)for arg in ['-L',p]],'-C','opt-level=3','-o',str(binary)],'compile',120,A)
 report['binary_sha256']=digest(binary)
 output=run([str(binary)],'cases',30,A);report['output']=output;print(output,flush=True);assert 'complete cases=4 wrong=0 blocked=0'in output;report['probe_complete']=True
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:
 verify();assert digest(source)==source_sha
 for p,sha in rlibs.items():assert digest(Path(p))==sha
 report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(code)
