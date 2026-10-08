from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='fillet-source-square-free-20260926-v271'
prior=json.loads((A/'shared-source-refinement-20260926-v270-terminal.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());prior['workspace_guard']=prior['source_manifest'];guard=json.loads((A/prior['workspace_guard']).read_text());source=A/f'{prefix}.rs';fixture=A/'fillet-reoffset-compact-20260926-v243-fiber.jsonl'
libs={};native=[]
for line in (A/'shared-source-refinement-20260926-v270-hypercurve-build.log').read_text().splitlines():
 try:r=json.loads(line)
 except ValueError:continue
 if r.get('reason')=='compiler-artifact'and r['target']['name'] in ['hyperreal','hypersolve','hyperlimit']:
  libs[r['target']['name']]=next(Path(f)for f in r['filenames']if f.endswith('.rlib'))
 if r.get('reason')=='build-script-executed':native.extend(r['linked_paths'])
inputs={str(f):hashlib.sha256(f.read_bytes()).hexdigest()for f in [source,fixture,Path(__file__).resolve(),*libs.values()]}
def verify():
 for f,sha in inputs.items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==sha,f
 for name,sha in guard.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
 for name,sha in manifest.items():
  for root in [Path(prior['source_directory']),A/'build-workspace-20260925']:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
binary=A/prefix;cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-C','opt-level=3',str(source),'-o',str(binary),'-L','dependency='+str(next(iter(libs.values())).parent)]
for name,p in libs.items():cmd+=['--extern',name+'='+str(p)]
for p in native:cmd+=['-L',p]
report=dict(inputs=inputs,source_manifest=prior['source_manifest'],workspace_guard=prior['workspace_guard'],command=cmd,all_processes_reaped=False)
verify();start=time.monotonic()
with(A/f'{prefix}-build.log').open('w')as log:code=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=120).returncode
report['build_returncode']=code;verify()
if code==0:
 report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 with(A/f'{prefix}.log').open('w')as log:
  try:code=subprocess.run([str(binary),str(fixture)],stdout=log,stderr=subprocess.STDOUT,timeout=40).returncode
  except subprocess.TimeoutExpired:code=124
 report['run_returncode']=code;print((A/f'{prefix}.log').read_text()[-4000:],flush=True)
else:print((A/f'{prefix}-build.log').read_text()[-4000:],flush=True)
verify();report['elapsed_seconds']=time.monotonic()-start;report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');raise SystemExit(code)
