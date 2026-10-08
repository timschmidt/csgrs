from pathlib import Path
import hashlib,json,os,re,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-retained-range-modes-20260927-v494';prior=json.loads((A/'stationary-fillet-20260927-v493-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['probe_complete'] and all(c['passed']for c in prior['cases'])
manifest=json.loads((A/prior['source_manifest']).read_text());source=A/'stationary-fillet-retained-range-v494.rs';source_sha=hashlib.sha256(source.read_bytes()).hexdigest();library=Path(prior['library']['path']);library_sha=prior['library']['sha256'];names=re.findall(r'#\[test\]fn (\w+)\(',source.read_text())
report=dict(source_manifest=prior['source_manifest'],source_directory=prior['source_directory'],fixture_sha256=source_sha,library=prior['library'],expected_cases=names,cases=[],all_processes_reaped=False,probe_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));binary=A/f'{prefix}-tests'
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(prior['source_directory']),A/'build-workspace-20260925']:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 assert hashlib.sha256(source.read_bytes()).hexdigest()==source_sha
 assert hashlib.sha256(library.read_bytes()).hexdigest()==library_sha

def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(command,log,timeout):
 start=time.monotonic()
 with log.open('w')as out:
  process=subprocess.Popen(command,cwd=W,stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:code=process.wait(timeout=timeout)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 return dict(command=command,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start)
verify();save();code=0
try:
 log=A/f'{prefix}-build.log';row=run([env['RUSTC'],'--edition=2024','--test',str(source),'-C','opt-level=3','-L','dependency='+str(library.parent/'deps'),'--extern','hypercurve='+str(library),'-o',str(binary)],log,120);report['build']=row;save()
 if row['returncode']:print(log.read_text()[-3000:],flush=True);raise RuntimeError('build failed')
 report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for name in names:
  log=A/f'{prefix}-{name}.log';row=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],log,90);row.update(name=name,passed=row['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(row);save();print(name,row['returncode'],round(row['elapsed_seconds'],3),flush=True)
  if row['returncode']:print(log.read_text()[-1800:],flush=True)
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
