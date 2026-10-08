from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-boundary-copy-20260927-v479'
r=json.loads((A/'stationary-boundary-copy-20260927-v478-terminal.json').read_text());assert r['all_processes_reaped'] and r['checks'][0]['returncode']==0
archive=Path(r['source_directory']);build=A/'build-workspace-20260925';manifest=json.loads((A/r['source_manifest']).read_text());source=archive/'stationary-corner-contract.rs';library=Path(r['library']['path']);dependencies=library.parent if library.parent.name=='deps' else library.parent/'deps'
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==(sha if root==W else r['trial'].get(name,sha)),(root,name)
 assert hashlib.sha256(source.read_bytes()).hexdigest()==r['fixture_sha256']
 assert hashlib.sha256(library.read_bytes()).hexdigest()==r['library']['sha256']
verify();report=dict(diagnostic_only=True,frozen_input='stationary-boundary-copy-20260927-v478-terminal.json',checks=[],cases=[],all_processes_reaped=False,capture_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));env['RUST_BACKTRACE']='0'
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+chr(10))
def run(cmd,log,timeout):
 start=time.monotonic()
 with log.open('w')as out:
  process=subprocess.Popen(cmd,cwd=archive,stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:code=process.wait(timeout=timeout)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 return dict(command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start)
code=0
try:
 binary=A/f'{prefix}-tests';log=A/f'{prefix}-build.log';row=run([env['RUSTC'],'--edition=2024','--test',str(source),'-C','opt-level=3','-L','dependency='+str(dependencies),'--extern','hypercurve='+str(library),'-o',str(binary)],log,120);report['checks'].append(row);save()
 if row['returncode']:print(log.read_text()[-3000:],flush=True);raise RuntimeError('diagnostic link failed')
 report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 for name in ['stationary_reparameterization_strict','one_sided_cusp_strict','interior_stationary_contact_strict']:
  log=A/f'{prefix}-{name}.log';row=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],log,45);row.update(name=name,stack_captured='BLOCKED_FILLET_BOUNDARY' in log.read_text());report['cases'].append(row);save();print(name,row['returncode'],row['stack_captured'],flush=True)
 report['capture_complete']=all(row['stack_captured']for row in report['cases']);code=0 if report['capture_complete']else 1
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
