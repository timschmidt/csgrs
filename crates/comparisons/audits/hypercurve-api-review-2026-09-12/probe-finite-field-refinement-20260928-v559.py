from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='finite-field-refinement-20260928-v559';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'finite-field-roots-20260928-v557-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['builds'][0]['returncode']==0
assert (A/'finite-field-roots-20260928-v557-interrupted.json').exists()
guard=json.loads((A/prior['source_manifest']).read_text());manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;data=src.read_bytes();current=hashlib.sha256(data).hexdigest();assert current==sha,name;production[name]=current
 if name=='hypercurve/src/bezier_parameter.rs':
  source=data.decode();a=source.index('fn exact_nonrational_bernstein_interval_roots(');b=source.index('\nfn exact_nonrational_bernstein_unit_roots(',a);part=source[a:b]
  assert part.count('refinement_steps: 0')==1;part=part.replace('refinement_steps: 0','refinement_steps: 8')
  needle='    let report = match isolate_ordered_field_polynomial_roots('
  part=part.replace(needle, '    eprintln!("V559 finite begin degree={}", polynomial.degree());\n    let started = std::time::Instant::now();\n'+needle)
  needle='    if report.status != OrderedFieldRootIsolationStatus::Isolated {'
  part=part.replace(needle, '    eprintln!("V559 finite end degree={} status={:?} roots={} subdivisions={} seconds={:.6}", polynomial.degree(), report.status, report.intervals.len(), report.subdivision_steps, started.elapsed().as_secs_f64());\n'+needle)
  source=source[:a]+part+source[b:];data=source.encode()
 manifest[name]=hashlib.sha256(data).hexdigest()
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
names={'hypercurve':[
 'bezier_parameter::finite_field_bernstein_regression::finite_field_isolation_reuses_original_polynomial_and_simple_root_proof',
 'bezier_parameter::finite_field_bernstein_regression::finite_field_isolation_keeps_repeated_roots_and_closed_boundaries',
 'curve::curve_fillet::stationary_family_composition_regression::normalized_region_selects_and_reuses_a_stationary_fillet_family',
], 'hypercurve_curve_region_promotion':[
 'strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions',
 'approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions',
]}
report=dict(normal_production_build=False,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases=names,all_processes_reaped=False,probe_complete=False,qualification_complete=False)
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
 log=A/f'{prefix}-build.log';row=run([cargo,'test','--lib','--test','hypercurve_curve_region_promotion','--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hypercurve',log,900);report['builds'].append(row);save()
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
   log=A/f'{prefix}-{name.split("::")[-1]}.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,180);case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
   if not case['passed']:
    print(log.read_text()[-2200:],flush=True)
    raise RuntimeError('refinement probe failed')
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
 if code:raise RuntimeError('focused regression failure')
 report['qualification_complete']=False
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
