from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='recursive-point-incidence-corrected-20260928-v585';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'recursive-point-incidence-20260928-v583-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['probe_complete']
archive=Path(prior['source_directory']);manifest=json.loads((A/prior['source_manifest']).read_text());production=manifest
names={'hypercurve':[
 'bezier_offset::conversion_tests::recursive_incidence_preserves_selected_point_fields',
 'bezier_offset::conversion_tests::recursive_incidence_keeps_stationary_and_collapsed_semantics',
 'bezier_offset::conversion_tests::recursive_incidence_preserves_exterior_domains_and_pole_barriers',
]}
import re
names['hypercurve'] += [name for name in json.loads((A/'finite-parameter-interval-20260928-v577-terminal.json').read_text())['expected_cases']['hypercurve'] if 'recursive_polynomial' in name or 'recursive_ordered_field' in name]
report=dict(normal_production_build=True,source_manifest=prior['source_manifest'],source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases=names,all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for name,sha in manifest.items():
  for root in [archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for name,sha in production.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,(W,name)
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
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
 report['builds']=[prior['builds'][0]]
 for target,cases in names.items():
  binary=Path(report['builds'][0]['binaries'][target]['path'])
  assert hashlib.sha256(binary.read_bytes()).hexdigest()==report['builds'][0]['binaries'][target]['sha256']
  listing=subprocess.check_output([str(binary),'--list'],text=True,cwd=archive/'hypercurve')
  (A/f'{prefix}-listing.txt').write_text(listing)
  available={line[:-6] for line in listing.splitlines() if line.endswith(': test')}
  assert set(cases)<=available,set(cases)-available
 save()
 for target,cases in names.items():
  binary=Path(report['builds'][0]['binaries'][target]['path'])
  for name in cases:
   log=A/f'{prefix}-{name.split("::")[-1]}.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,240);case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
   if not case['passed']:print(log.read_text()[-1800:],flush=True)
   for line in log.read_text().splitlines():
    if line.startswith('SOURCE_CELL_WORK '):print(line,flush=True)
 report['probe_complete']=True
 if not all(row['passed']for row in report['cases']):raise RuntimeError('focused regression failure')
 report['qualification_complete']=False
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
