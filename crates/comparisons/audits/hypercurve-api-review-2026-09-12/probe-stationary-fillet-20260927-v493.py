from pathlib import Path
import hashlib,json,os,re,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-fillet-20260927-v493';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'stationary-fillet-20260927-v489-sources.json').read_text());assert not archive.exists();manifest={}
for name in prior:
 src=W/name;data=src.read_bytes();manifest[name]=hashlib.sha256(data).hexdigest()
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
source=archive/'stationary-corner-contract.rs';shutil.copy2(A/'stationary-fillet-retained-range-v490.rs',source);source_sha=hashlib.sha256(source.read_bytes()).hexdigest()
names=re.findall(r'#\[test\]fn (\w+)\(',source.read_text())
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),fixture_sha256=source_sha,checks=[],cases=[],expected_cases=names,all_processes_reaped=False,probe_complete=False)
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 assert hashlib.sha256(source.read_bytes()).hexdigest()==source_sha

def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
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
  raise RuntimeError('library build failed')
 artifacts=[]
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  if v.get('reason')=='compiler-artifact' and v.get('target',{}).get('name')=='hypercurve' and 'lib'in v['target']['kind']:artifacts.append(v)
 library=next(Path(p)for p in artifacts[-1]['filenames']if p.endswith('.rlib'));report['library']=dict(path=str(library),sha256=hashlib.sha256(library.read_bytes()).hexdigest());save()
 binary=A/f'{prefix}-tests';log=A/f'{prefix}-probe-build.log';row=run([env['RUSTC'],'--edition=2024','--test',str(source),'-C','opt-level=3','-L','dependency='+str(library.parent/'deps'),'--extern','hypercurve='+str(library),'-o',str(binary)],archive,log,120);report['checks'].append(row);save()
 if row['returncode']:print(log.read_text()[-3000:],flush=True);raise RuntimeError('fixture build failed')
 report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for name in names:
  log=A/f'{prefix}-{name}.log';row=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive,log,90);row.update(name=name,passed=row['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(row);save();print(name,row['returncode'],round(row['elapsed_seconds'],3),flush=True)
  if row['returncode']:print(log.read_text()[-1800:],flush=True)
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
