from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-point-query-broad-20260928-v567';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'finite-field-root-identity-20260928-v563-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
guard=json.loads((A/prior['source_manifest']).read_text());manifest={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;current=hashlib.sha256(src.read_bytes()).hexdigest();assert name in {'hypercurve/src/bezier_offset.rs','hypercurve/src/curve_fillet.rs'} or current==sha,name;manifest[name]=current
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
focus=json.loads((A/'retained-point-query-20260928-v566-terminal.json').read_text());assert focus['all_processes_reaped'] and focus['probe_complete'] and all(c['passed']for c in focus['cases'])
assert json.loads((A/focus['source_manifest']).read_text())==manifest
names={target:list(cases)for target,cases in prior['expected_cases'].items()}
new_case='curve::curve_fillet::stationary_retained_point_constraint_regression::stationary_fillet_family_accepts_independent_retained_point_constraints'
names['hypercurve'].insert(0,new_case)
assert sum(map(len,names.values()))==451
reused={(c['target'],c['name'])for c in focus['cases']}
assert len(reused)==31
order=[(target,name)for target,cases in names.items()for name in cases if 'trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions'in name]
order.extend((target,name)for target,cases in names.items()for name in cases if (target,name)not in order and (target,name)not in reused)
assert len(order)==420 and len(set(order))==420

report=dict(normal_production_build=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases=names,reused_qualification='retained-point-query-20260928-v566-terminal.json',all_processes_reaped=False,probe_complete=False,qualification_complete=False)
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
 log=A/f'{prefix}-build.log';row=run([cargo,'test','--lib',*[arg for target in names if target!='hypercurve' for arg in ['--test',target]],'--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hypercurve',log,900);report['builds'].append(row);save()
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
 assert row['binaries']['hypercurve']['sha256']==focus['builds'][0]['binaries']['hypercurve']['sha256']
 report['cases']=list(focus['cases']);save()
 for target,name in order:
  binary=Path(report['builds'][0]['binaries'][target]['path'])
  log=A/f'{prefix}-{name.split("::")[-1]}.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,180 if "trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions" in name else 240);case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
  if not case['passed']:
   print(log.read_text()[-1800:],flush=True)
   raise RuntimeError('finite-field regression failed')
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
 if code:raise RuntimeError('focused regression failure')
 checks=[
  ('hypercurve-clippy-all-features',[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings'],'hypercurve'),
  ('hypercurve-clippy-no-default',[cargo,'clippy','--all-targets','--no-default-features','--locked','--offline','--','-D','warnings'],'hypercurve'),
  ('hypercurve-format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check','src/bezier_offset.rs','src/curve_fillet.rs'],'hypercurve'),
  ('fuzz-check',[cargo,'check','--manifest-path','fuzz/Cargo.toml','--bin','curve_string_editing','--locked','--offline'],'hypercurve'),
  ('hypercurve-documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],'hypercurve'),
  ('hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],'hyperbrep'),
 ]
 env['RUSTDOCFLAGS']='-D warnings'
 for label,command,crate in checks:
  log=A/f'{prefix}-{label}.log';row=run(command,build/crate,log,900);row['label']=label;report['checks'].append(row);save();print(label,row['returncode'],round(row['elapsed_seconds'],3),flush=True)
  if row['returncode']:print(log.read_text()[-3500:],flush=True);raise RuntimeError(label+' failed')
 report['qualification_complete']=True
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
