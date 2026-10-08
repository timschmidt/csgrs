from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='collapsed-selected-circle-contact-20260927-v332';qualified='retained-native-circle-contact-20260927-v331'
manifest=json.loads((A/f'{qualified}-sources.json').read_text());qualification=json.loads((A/f'{qualified}-terminal.json').read_text())
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(qualification['source_directory']),A/'build-workspace-20260925']:
   assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
verify()
rows=[]
for line in (A/f'{qualified}-hypercurve_curve-build.log').read_text().splitlines():
 try:rows.append(json.loads(line))
 except ValueError:pass
artifact=next(row for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='hypercurve' and any(name.endswith('.rlib') for name in row['filenames']))
library=Path(next(name for name in artifact['filenames'] if name.endswith('.rlib')));source=A/f'{prefix}.rs';binary=A/prefix
report=dict(source_manifest=f'{qualified}-sources.json',source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),library=str(library),library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),all_processes_reaped=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-C','opt-level=3',str(source),'--extern',f'hypercurve={library}','-L',f'dependency={library.parent}','-o',str(binary)]
with (A/f'{prefix}-build.log').open('w') as log:
 report['build_returncode']=subprocess.run(command,cwd=A,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180).returncode
if report['build_returncode']==0:
 report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest();start=time.monotonic()
 with (A/f'{prefix}.log').open('w') as log:
  try:report['returncode']=subprocess.run([str(binary)],cwd=A,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=60).returncode
  except subprocess.TimeoutExpired:report['returncode']=124
 report['elapsed_seconds']=time.monotonic()-start
 print((A/f'{prefix}.log').read_text(),flush=True)
else:print((A/f'{prefix}-build.log').read_text(),flush=True)
verify();assert hashlib.sha256(source.read_bytes()).hexdigest()==report['source_sha256'];assert hashlib.sha256(library.read_bytes()).hexdigest()==report['library_sha256']
report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(report.get('returncode',report['build_returncode']))
