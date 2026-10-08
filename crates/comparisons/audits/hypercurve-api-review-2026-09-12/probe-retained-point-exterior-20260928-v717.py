from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-point-exterior-20260928-v717';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925';candidate=A/'retained-point-exterior-candidate-v716'
assert (A/'composition-evidence-20260928-v711-committed.json').exists()
baseline=json.loads((A/'composition-evidence-20260928-v711-sources.json').read_text());assert len(baseline)==2048
assert json.loads((A/'retained-point-classification-20260928-v714-reaped.json').read_text())['outer_exit_code']==1
changed=set(json.loads((A/'retained-point-exterior-base-v716.json').read_text()));test_source='hypercurve/src/bezier_region.rs';assert (candidate/test_source).read_text().split('\n#[cfg(test)]\nmod retained_point_classification_probe_v713 {')[0].rstrip()==(W/test_source).read_text().rstrip()
manifest={};assert not archive.exists()
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for name,sha in baseline.items():
 assert digest(W/name)==sha,name
 data=(candidate/name).read_bytes()if name in changed else(W/name).read_bytes();manifest[name]=hashlib.sha256(data).hexdigest()
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(baseline,indent=2)+'\n')
report=dict(copied_source_probe=True,source_manifest=f'{prefix}-sources.json',production_manifest=f'{prefix}-production-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],all_processes_reaped=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for name,sha in baseline.items():assert digest(W/name)==sha,name
 for name,sha in manifest.items():
  for root in [archive,build]:assert digest(root/name)==sha,(root,name)
 for name in changed:assert digest(candidate/name)==manifest[name]
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
 log=A/f'{prefix}-hypercurve-build.log';row=run([cargo,'test','--lib','--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hypercurve',log,900);report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:message=json.loads(line)
   except ValueError:continue
   if message.get('reason')=='compiler-message'and message['message']['level']=='error':print(message['message'].get('rendered','')[:2500],flush=True)
  raise RuntimeError('probe build failed')
 artifacts=[]
 for line in log.read_text().splitlines():
  try:message=json.loads(line)
  except ValueError:continue
  if message.get('reason')=='compiler-artifact'and message.get('target',{}).get('name')=='hypercurve'and message.get('executable'):artifacts.append(message)
 assert len(artifacts)==1;binary=A/f'{prefix}-hypercurve-tests';shutil.copy2(artifacts[0]['executable'],binary);row['binaries']={'hypercurve':dict(path=str(binary),sha256=digest(binary))};save();print('Probe compiled',round(row['elapsed_seconds'],3),flush=True)
 for kind in ['chord_normal','analytic_parallel','similarity','lazy_endpoint']:
  name=f'bezier_region::retained_point_classification_probe_v713::{kind}_points_classify_on_exact_rectangle_oracles';log=A/f'{prefix}-{kind}.log'
  case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,240);case.update(name=name,target='hypercurve',passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(kind,case['returncode'],round(case['elapsed_seconds'],3),log.read_text()[-1800:],flush=True)
  if not case['passed']:raise RuntimeError(kind+' point probe failed')
 report['qualification_complete']=True
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
