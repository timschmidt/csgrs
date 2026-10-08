from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-family-constraint-diagnostic-20260927-v538';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'stationary-pair-cells-broad-20260927-v536-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
assert (A/'stationary-pair-cells-broad-20260927-v536-committed.json').exists()
guard=json.loads((A/prior['source_manifest']).read_text());manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;current=hashlib.sha256(src.read_bytes()).hexdigest();assert current==sha,name;production[name]=current
 data=src.read_bytes()
 if name in ['hypercurve/src/curve.rs','hypercurve/src/curve_fillet.rs']:
  source=data.decode().replace('ExactCurveError::blocked(', 'fillet_diagnostic_blocked(')
  source=source.replace('blocked(reason)', 'blocked({ eprintln!("V538 blocked call line={}", line!()); reason })')
  source=source.replace('blocked(crate::UncertaintyReason::Predicate)', 'blocked({ eprintln!("V538 incomplete replay line={}", line!()); crate::UncertaintyReason::Predicate })')
  if name=='hypercurve/src/curve.rs':
   source+='\n#[track_caller]\nfn fillet_diagnostic_blocked(operation: CurveOperation2, family: CurveFamily2, reason: crate::UncertaintyReason) -> ExactCurveError { let caller=std::panic::Location::caller(); eprintln!("V538 blocked at {}:{} reason={:?}",caller.file(),caller.line(),reason); ExactCurveError::blocked(operation,family,reason) }\n'
  else:
   source=source.replace('            centers.components.extend(components);', '            eprintln!("V538 pair cell {} {} contacts={} components={}", first, second, intersections.contacts().len(), components.len());\n            centers.components.extend(components);')
   source=source.replace('        let selected = match component.constrain(', '        eprintln!("V538 select component constrained_axes={}", constraints.iter().filter(|x| x.is_some()).count());\n        let selected = match component.constrain(')
   fixture=(A/'stationary-continuous-family-v533.rs').read_text().replace('for selected in requests {','for (request_index, selected) in requests.into_iter().enumerate() {\n                    eprintln!("V538 request {request_index} reversed={reversed} policy={policy:?}");')
   source+='\n'+fixture
  data=source.encode()
 manifest[name]=hashlib.sha256(data).hexdigest()
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
names={'hypercurve':['curve::curve_fillet::stationary_continuous_family_regression::continuous_fillets_preserve_stationary_reparameterization']}
report=dict(normal_production_build=False,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases=names,all_processes_reaped=False,probe_complete=False,qualification_complete=False)
fixture_sha=hashlib.sha256((A/'stationary-continuous-family-v533.rs').read_bytes()).hexdigest()
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 assert hashlib.sha256((A/'stationary-continuous-family-v533.rs').read_bytes()).hexdigest()==fixture_sha
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
 log=A/f'{prefix}-build.log';row=run([cargo,'test','--lib','--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hypercurve',log,900);report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('fixture build failed')
 artifacts={}
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  target=v.get('target',{}).get('name')
  if v.get('reason')=='compiler-artifact' and target in names and v.get('executable'):artifacts[target]=v
 assert set(artifacts)==set(names)
 row['binaries']={}
 for target,artifact in artifacts.items():
  binary=A/f'{prefix}-{target}-tests';shutil.copy2(artifact['executable'],binary);row['binaries'][target]=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());save()
 for target,cases in names.items():
  binary=Path(report['builds'][0]['binaries'][target]['path'])
  for name in cases:
   log=A/f'{prefix}-case.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,240);case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
   if not case['passed']:print(log.read_text()[-1800:],flush=True)
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
 if code:raise RuntimeError('focused regression failure')
 report['qualification_complete']=False
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
