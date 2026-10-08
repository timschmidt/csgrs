from pathlib import Path
import hashlib,json,os,re,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-carrier-domains-20260927-v492';prior=json.loads((A/'stationary-fillet-20260927-v489-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
manifest=json.loads((A/prior['source_manifest']).read_text());receipt=json.loads((A/'stationary-fillet-20260927-v489-library.json').read_text());library=Path(receipt['path'])
groups=[('line_chart','stationary-fillet-line-chart-v488.rs',['regular_parabola_strict','stationary_reparameterization_strict','interior_stationary_contact_strict']),('retained_range','stationary-fillet-retained-range-v490.rs',['retained_regular_source_range_strict','retained_nonzero_parallel_range_strict']),('radius_only','stationary-fillet-radius-only-v491.rs',['regular_parabola_strict','stationary_reparameterization_strict','one_sided_cusp_strict','regular_parabola_approximate','stationary_reparameterization_approximate','one_sided_cusp_approximate'])]
fixtures={source:hashlib.sha256((A/source).read_bytes()).hexdigest()for _,source,_ in groups};report=dict(diagnostic_only=True,source_manifest=prior['source_manifest'],source_directory=prior['source_directory'],fixtures=fixtures,library=receipt,expected_cases=groups,builds=[],cases=[],all_processes_reaped=False,probe_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));expected_head='2ff2b956a521e6e3baf778255a0abd92d18b4d00'
def verify():
 assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip()==expected_head
 for name,sha in manifest.items():
  for root in [W,Path(prior['source_directory']),A/'build-workspace-20260925']:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for name,sha in fixtures.items():assert hashlib.sha256((A/name).read_bytes()).hexdigest()==sha,name
 assert hashlib.sha256(library.read_bytes()).hexdigest()==receipt['sha256']
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
 for group,fixture,names in groups:
  source=A/fixture;binary=A/f'{prefix}-{group}-tests';log=A/f'{prefix}-{group}-build.log';dependency=library.parent if library.parent.name=='deps'else library.parent/'deps'
  row=run([env['RUSTC'],'--edition=2024','--test',str(source),'-C','opt-level=3','-L','dependency='+str(dependency),'--extern','hypercurve='+str(library),'-o',str(binary)],log,120);report['builds'].append(row);save()
  if row['returncode']:print(log.read_text()[-3000:],flush=True);raise RuntimeError(group+' build failed')
  row['binary']=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());save()
  for name in names:
   log=A/f'{prefix}-{group}-{name}.log';row=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],log,90);row.update(group=group,name=name,passed=row['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(row);save();print(group,name,row['returncode'],round(row['elapsed_seconds'],3),flush=True)
   if row['returncode']:print(log.read_text()[-1600:],flush=True)
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
