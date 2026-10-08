from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent; W=A.parent; prefix='narrow-selected-root-endpoint-20260927-v358'
prior=json.loads((A/'cached-tower-approximation-20260927-v357-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
manifest=json.loads((A/prior['source_manifest']).read_text());build=A/'build-workspace-20260925'
def verify():
 for n,h in manifest.items():
  for root in [W,Path(prior['source_directory']),build]:assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,n
verify();rows=[]
for line in (A/'cached-tower-approximation-20260927-v357-hypercurve-build.log').read_text().splitlines():
 try:rows.append(json.loads(line))
 except ValueError:pass
libs={}
for crate in ['hyperreal','hypersolve']:
 r=next(r for r in rows if r.get('reason')=='compiler-artifact' and r['target']['name']==crate and 'lib' in r['target']['kind'])
 libs[crate]=next(Path(f) for f in r['filenames'] if f.endswith('.rlib'))
source=A/f'{prefix}.rs';binary=A/prefix;report=dict(source_manifest=prior['source_manifest'],source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),libraries={k:dict(path=str(v),sha256=hashlib.sha256(v.read_bytes()).hexdigest())for k,v in libs.items()},all_processes_reaped=False)
cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(source),'-o',str(binary),'-L','dependency='+str(libs['hypersolve'].parent)]
for k,p in libs.items():cmd.extend(['--extern',k+'='+str(p)])
report['compile']=dict(command=cmd,returncode=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60).returncode)
assert report['compile']['returncode']==0
report['binary']=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());t=time.monotonic()
try:
 r=subprocess.run([str(binary)],capture_output=True,text=True,timeout=60);report['run']=dict(returncode=r.returncode,output=r.stdout,stderr=r.stderr,elapsed_seconds=time.monotonic()-t)
except subprocess.TimeoutExpired as e:report['run']=dict(returncode=124,elapsed_seconds=time.monotonic()-t)
verify();report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['run']),flush=True)
