from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='regular-pair-domains-20260927-v500';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'stationary-contact-signs-20260927-v498-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
assert (A/'stationary-contact-signs-20260927-v498-committed.json').exists()
guard=json.loads((A/prior['source_manifest']).read_text());manifest={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;current=hashlib.sha256(src.read_bytes()).hexdigest();assert name=='hypercurve/src/bezier_offset.rs' or current==sha,name;manifest[name]=current
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
names=json.loads((A/'stationary-contact-signs-20260927-v497-terminal.json').read_text())['expected_cases']+['bezier_offset::regular_parallel_contact_tests::exterior_regular_pair_keeps_contacts_outside_ancestral_bounds','bezier_offset::regular_parallel_contact_tests::general_regular_pair_replays_unequal_contact_scale_changes']
report=dict(normal_production_build=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],expected_cases=names,all_processes_reaped=False,probe_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
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
  log=A/f'{prefix}-{name.split("::")[-1]}.log';row=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,120);row.update(name=name,passed=row['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(row);save();print(name,row['returncode'],round(row['elapsed_seconds'],3),log.read_text()[-1800:],flush=True)
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
