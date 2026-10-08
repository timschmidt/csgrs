from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='structural-overlap-reentry-20260928-v693';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925';candidate=A/'structural-overlap-reentry-v692.rs'
assert json.loads((A/'resultant-interpolation-20260928-v687-reaped.json').read_text())['outer_exit_code']==0
assert (A/'resultant-interpolation-20260928-v687-committed.json').exists()
prior=json.loads((A/'resultant-interpolation-20260928-v687-terminal.json').read_text());guard=json.loads((A/prior['source_manifest']).read_text())
base=json.loads((A/'structural-overlap-inventory-base-v688.json').read_text());overrides={base['path']:base['sha256']};assert all(guard[name]==sha for name,sha in overrides.items())
manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 data=(W/name).read_bytes();assert hashlib.sha256(data).hexdigest()==sha,name;production[name]=sha
 if name in overrides:data=candidate.read_bytes()
 manifest[name]=hashlib.sha256(data).hexdigest();dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
names={'hypercurve':['bezier_offset::structural_overlap_trace_regression::monotone_reparameterization_preserves_structural_pair_crossings_strict']}
report=dict(normal_production_build=False,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],test_listings={},expected_cases=names,all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for name,sha in manifest.items():
  for root in [archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for name,sha in production.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,(W,name)
 for name in overrides:assert hashlib.sha256(candidate.read_bytes()).hexdigest()==manifest[name],name
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
 for crate in names:
  log=A/f'{prefix}-{crate}-build.log';row=run([cargo,'test','--lib','--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/crate,log,900);report['builds'].append(row);save()
  if row['returncode']:
   for line in log.read_text().splitlines():
    try:v=json.loads(line)
    except ValueError:continue
    if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
   raise RuntimeError(crate+' candidate build failed')
  print(crate,'compiled',round(row['elapsed_seconds'],3),flush=True)
  artifacts=[]
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-artifact'and v.get('target',{}).get('name')==crate and v.get('executable'):artifacts.append(v)
  assert len(artifacts)==1
  binary=A/f'{prefix}-{crate}-tests';shutil.copy2(artifacts[0]['executable'],binary);row['binaries']={crate:dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())};save()
  log=A/f'{prefix}-{crate}-list.log';listing=run([str(binary),'--list'],archive/crate,log,30);report['test_listings'][crate]=listing;assert listing['returncode']==0
  available={line.removesuffix(': test')for line in log.read_text().splitlines()if line.endswith(': test')}
  assert set(names[crate])<=available
  save();print(crate,'selected',len(names[crate]),flush=True)
  for name in names[crate]:
   log=A/f'{prefix}-{name.split("::")[-1]}-{hashlib.sha256((crate+name).encode()).hexdigest()[:10]}.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/crate,log,180);case.update(target=crate,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
   if not case['passed']:print(log.read_text()[-2500:],flush=True);raise RuntimeError('focused regression failed: '+name)
 report['probe_complete']=True
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
