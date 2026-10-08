from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='known-fillet-centers-20260928-v665';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
assert json.loads((A/'known-fillet-centers-20260928-v663-reaped.json').read_text())['outer_exit_code']==1
assert (A/'incident-cusp-side-20260928-v657-committed.json').exists()
prior=json.loads((A/'incident-cusp-side-20260928-v657-terminal.json').read_text());guard=json.loads((A/prior['source_manifest']).read_text())
changed={'hypercurve/src/curve.rs','hypercurve/src/curve_fillet.rs'}
manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;data=src.read_bytes();current=hashlib.sha256(data).hexdigest();assert name in changed or current==sha,name;production[name]=current
 manifest[name]=current;dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
assert {name for name,sha in manifest.items()if sha!=guard[name]}==changed
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
previous=json.loads((A/'unit-domain-core-20260928-v649-terminal.json').read_text())['expected_cases']
related=json.loads((A/'incident-cusp-side-20260928-v657-terminal.json').read_text())['expected_cases']['hypercurve']
selected=[name for name in previous['hypercurve'] if name.startswith('curve::curve_fillet::')]
new='curve::curve_fillet::incident_parallel_cusp_nonlinear_regression::incident_parallel_cusp_fillet_retains_nonlinear_partner_contact'
loop='curve::curve_fillet::constrained_regular_loop_fillet_regression::point_constraint_preserves_distinct_regular_loop_contacts'
names={'hypercurve': list(dict.fromkeys([new,loop,*related,*selected])), 'hypercurve_stationary_fillets':previous['hypercurve_stationary_fillets']}
print('Selected inventories', {k:len(v)for k,v in names.items()},flush=True)
report=dict(normal_production_build=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases=names,test_listings={},all_processes_reaped=False,probe_complete=False,qualification_complete=False)
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
 log=A/f'{prefix}-build.log';row=run([cargo,'test','--lib',*[arg for target in names if target!='hypercurve' for arg in ['--test',target]],'--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hypercurve',log,900);report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('known-center fillet build failed')
 print('Candidate compiled',round(row['elapsed_seconds'],3),flush=True)
 artifacts={}
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  target=v.get('target',{}).get('name')
  if v.get('reason')=='compiler-artifact'and target in names and v.get('executable'):artifacts[target]=v
 assert set(artifacts)==set(names)
 binaries={}
 for target,artifact in artifacts.items():
  binary=A/f'{prefix}-{target}-tests';shutil.copy2(artifact['executable'],binary);binaries[target]=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
 row['binaries']=dict(binaries);save()
 for target,selected in names.items():
  binary=binaries[target]['path'];listing_log=A/f'{prefix}-{target}-list.log';listing=run([binary,'--list'],archive/'hypercurve',listing_log,30);report['test_listings'][target]=listing;save();assert listing['returncode']==0
  available={line.removesuffix(': test')for line in listing_log.read_text().splitlines()if line.endswith(': test')};assert set(selected)<=available,(target,set(selected)-available)
 for target,selected in names.items():
  for name in selected:
   log=A/f'{prefix}-{name.split("::")[-1]}-{hashlib.sha256((target+name).encode()).hexdigest()[:10]}.log';case=run([binaries[target]['path'],'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,180);case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
   if not case['passed']:
    print(log.read_text()[-2500:],flush=True);raise RuntimeError('focused regression failed: '+name)
 report['probe_complete']=True;code=0 if all(case['passed']for case in report['cases'])else 1
 checks=[
  ('hypercurve-clippy-all-features',[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings'],'hypercurve'),
  ('hypercurve-clippy-no-default',[cargo,'clippy','--all-targets','--no-default-features','--locked','--offline','--','-D','warnings'],'hypercurve'),
  ('format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check',*[str(build/n)for n in sorted(changed)]],'hypercurve'),
  ('fuzz-check',[cargo,'check','--manifest-path','fuzz/Cargo.toml','--bin','curve_string_editing','--bin','bezier_arrangement','--bin','bezier_region','--bin','bezier_split_materialization','--locked','--offline'],'hypercurve'),
  ('hypercurve-documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],'hypercurve'),
  ('hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],'hyperbrep'),
 ]
 env['RUSTDOCFLAGS']='-D warnings'
 for label,command,crate in checks:
  log=A/f'{prefix}-{label}.log';row=run(command,build/crate,log,900);row['label']=label;report['checks'].append(row);save();print(label,row['returncode'],round(row['elapsed_seconds'],3),flush=True)
  if row['returncode']:print(log.read_text()[-5000:],flush=True);raise RuntimeError(label+' failed')
 report['qualification_complete']=True
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
