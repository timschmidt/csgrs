from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='radical-polynomial-evaluation-20260928-v629';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
assert json.loads((A/'retained-structural-pair-diagnostic-20260928-v627-reaped.json').read_text())['outer_exit_code']==1
assert json.loads((A/'radical-polynomial-evaluation-20260928-v628-reaped.json').read_text())['outer_exit_code']==0
prior=json.loads((A/'parameter-construction-20260928-v605-terminal.json').read_text());guard=json.loads((A/prior['source_manifest']).read_text())
fixture=A/'radical-polynomial-evaluation-v629.rs';fixture_sha=hashlib.sha256(fixture.read_bytes()).hexdigest();manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;data=src.read_bytes();assert hashlib.sha256(data).hexdigest()==sha,name;production[name]=sha
 if name=='hyperreal/src/real/arithmetic/tests.rs':data+=fixture.read_bytes()
 manifest[name]=hashlib.sha256(data).hexdigest();dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
names=['real::arithmetic::radical_polynomial_evaluation_probe::'+suffix for suffix in ['cold_integral_cusp_polynomial','cold_actual_resultant_cusp_polynomial','cold_quadratic_formula_cusp_polynomial']]
report=dict(normal_production_build=False,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases={'hyperreal':names},all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 assert hashlib.sha256(fixture.read_bytes()).hexdigest()==fixture_sha
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
 log=A/f'{prefix}-build.log';row=run([cargo,'test','--lib','--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hyperreal',log,900);report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('scalar fixture build failed')
 artifact=None
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  if v.get('reason')=='compiler-artifact'and v.get('target',{}).get('name')=='hyperreal'and v.get('executable'):artifact=v
 assert artifact
 binary=A/f'{prefix}-hyperreal-tests';shutil.copy2(artifact['executable'],binary);row['binaries']={'hyperreal':dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())};save()
 listing=subprocess.check_output([str(binary),'--list'],text=True,cwd=archive/'hyperreal');assert all(name+': test' in listing for name in names)
 for name in names:
  log=A/f'{prefix}-{name.split("::")[-1]}.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hyperreal',log,60);case.update(target='hyperreal',name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(log.read_text()[-12000:],flush=True)
 report['probe_complete']=True;code=0 if all(case['passed']for case in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
