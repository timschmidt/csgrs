from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time,runpy
A=Path(__file__).resolve().parent;W=A.parent;prefix='ph-pair-tangent-20260927-v514';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'retained-rational-domain-20260927-v511-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
assert (A/'retained-rational-domain-20260927-v511-committed.json').exists()
guard=json.loads((A/prior['source_manifest']).read_text());manifest=dict(guard);assert not archive.exists()
fixture=A/'probe-ph-pair-tangent-v514-fixture.py';fixture_sha=hashlib.sha256(fixture.read_bytes()).hexdigest()
for name,sha in guard.items():
 src=W/name;assert hashlib.sha256(src.read_bytes()).hexdigest()==sha,name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target);os.utime(target,None)
name='hypercurve/src/bezier_offset.rs';source=archive/name;source.write_text(runpy.run_path(str(fixture))['instrument'](source.read_text()));shutil.copy2(source,build/name);manifest[name]=hashlib.sha256(source.read_bytes()).hexdigest()
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
names=['bezier_offset::conversion_tests::regular_ph_branch_pairs_retain_exact_cusp_incidence']
report=dict(diagnostic_only=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),workspace_guard=prior['source_manifest'],fixture_sha256=fixture_sha,builds=[],cases=[],expected_cases=names,all_processes_reaped=False,probe_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for name,sha in guard.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
 for name,sha in manifest.items():
  for root in [archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 assert hashlib.sha256(fixture.read_bytes()).hexdigest()==fixture_sha
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
 log=A/f'{prefix}-build.log';row=run([cargo,'test','--lib','--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hypercurve',log,900);report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('fixture build failed')
 artifacts=[]
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  if v.get('reason')=='compiler-artifact'and v.get('target',{}).get('name')=='hypercurve'and v.get('executable'):artifacts.append(v)
 binary=A/f'{prefix}-tests';shutil.copy2(artifacts[-1]['executable'],binary);row['binary']=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());save()
 for name in names:
  log=A/f'{prefix}-{name.split("::")[-1]}.log';row=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,120);row.update(name=name,passed=row['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(row);save();print(name,row['returncode'],round(row['elapsed_seconds'],3),log.read_text()[-5000:],flush=True)
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
