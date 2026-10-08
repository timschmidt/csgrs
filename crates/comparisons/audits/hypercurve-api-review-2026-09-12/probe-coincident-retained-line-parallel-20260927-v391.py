from pathlib import Path
import hashlib,json,os,subprocess,time,shutil,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='coincident-retained-line-parallel-20260927-v391';qualified='coincident-circular-constraints-20260927-v390'
manifest=json.loads((A/f'{qualified}-sources.json').read_text());qualification=json.loads((A/f'{qualified}-terminal.json').read_text())
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(qualification['source_directory']),A/'build-workspace-20260925']:
   assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
assert json.loads((A/'coincident-circular-constraints-20260927-v390-terminal.json').read_text())['all_processes_reaped']
for name,sha in manifest.items():
 source=W/name;assert hashlib.sha256(source.read_bytes()).hexdigest()==sha,name
 target=A/'build-workspace-20260925'/name
 if target.read_bytes()!=source.read_bytes():shutil.copy2(source,target);os.utime(target,None)
verify()
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
def run_owned(cmd, **kwargs):
 timeout=kwargs.pop('timeout');process=subprocess.Popen(cmd,**kwargs,start_new_session=True)
 try:return subprocess.CompletedProcess(cmd,process.wait(timeout=timeout))
 except BaseException:
  try:os.killpg(process.pid,signal.SIGKILL)
  except ProcessLookupError:pass
  process.wait();raise
cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo','build','--lib','--release','--all-features','--message-format=json','--locked','--offline']
with(A/f'{prefix}-library-build.log').open('w')as out:
 code=run_owned(cmd,cwd=A/'build-workspace-20260925'/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
assert code==0
rows=[]
for line in (A/f'{prefix}-library-build.log').read_text().splitlines():
 try:rows.append(json.loads(line))
 except ValueError:pass
artifact=next(row for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='hypercurve' and any(name.endswith('.rlib') for name in row['filenames']))
library=Path(next(name for name in artifact['filenames'] if name.endswith('.rlib')));source=A/f'{prefix}.rs';binary=A/prefix
report=dict(source_manifest=f'{qualified}-sources.json',source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),library=str(library),library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),all_processes_reaped=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-C','opt-level=3',str(source),'--extern',f'hypercurve={library}','-L',f'dependency={library.parent / "deps" if (library.parent / "deps").is_dir() else library.parent}','-o',str(binary)]
with (A/f'{prefix}-build.log').open('w') as log:
 report['build_returncode']=run_owned(command,cwd=A,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180).returncode
if report['build_returncode']==0:
 report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest();start=time.monotonic()
 with (A/f'{prefix}.log').open('w') as log:
  try:report['returncode']=run_owned([str(binary)],cwd=A,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120).returncode
  except subprocess.TimeoutExpired:report['returncode']=124
 report['elapsed_seconds']=time.monotonic()-start
 print((A/f'{prefix}.log').read_text(),flush=True)
else:print((A/f'{prefix}-build.log').read_text(),flush=True)
verify();assert hashlib.sha256(source.read_bytes()).hexdigest()==report['source_sha256'];assert hashlib.sha256(library.read_bytes()).hexdigest()==report['library_sha256']
report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(report.get('returncode',report['build_returncode']))
