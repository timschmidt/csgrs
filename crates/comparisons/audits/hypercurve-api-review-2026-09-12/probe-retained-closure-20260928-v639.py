from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-closure-probes-20260928-v639';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
assert json.loads((A/'parameter-root-closure-20260928-v638-reaped.json').read_text())['outer_exit_code']==0
assert (A/'parameter-root-closure-20260928-v638-committed.json').exists()
prior=json.loads((A/'parameter-root-closure-20260928-v638-terminal.json').read_text());guard=json.loads((A/prior['source_manifest']).read_text())
fixtures={'hypercurve/src/bezier_offset.rs':A/'structural-overlap-trace-probe-v634.rs','hypercurve/src/curve_fillet.rs':A/'incident-parallel-cusp-fillet-v599.rs','hypercurve/src/bezier_split.rs':A/'finite-conic-split-probe-v639.rs'}
fixture_hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest()for path in fixtures.values()};manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;data=src.read_bytes();assert hashlib.sha256(data).hexdigest()==sha,name;production[name]=sha
 if name in fixtures:data+=b'\n'+fixtures[name].read_bytes()
 manifest[name]=hashlib.sha256(data).hexdigest();dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
names=['bezier_offset::structural_overlap_trace_regression::'+suffix for suffix in ['monotone_reparameterization_preserves_structural_pair_crossings_strict','monotone_reparameterization_preserves_structural_pair_crossings_approximate','reversed_monotone_reparameterization_preserves_structural_pair_crossings_strict','reversed_monotone_reparameterization_preserves_structural_pair_crossings_approximate']]
names.extend('curve::curve_fillet::incident_parallel_cusp_fillet_regression::'+suffix for suffix in ['incident_parallel_cusp_fillet_retains_native_line_contact','incident_parallel_cusp_fillet_retains_polynomial_line_contact'])
names.insert(0,'bezier_split::finite_conic_split_regression::finite_conic_split_retains_zero_intermediate_homogeneous_weight')
report=dict(normal_production_build=False,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases={'hypercurve':names},all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for path,sha in fixture_hashes.items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha,path
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
  raise RuntimeError('retained closure fixture build failed')
 artifact=None
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  if v.get('reason')=='compiler-artifact'and v.get('target',{}).get('name')=='hypercurve'and v.get('executable'):artifact=v
 assert artifact
 binary=A/f'{prefix}-hypercurve-tests';shutil.copy2(artifact['executable'],binary);row['binaries']={'hypercurve':dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())};save()
 listing=subprocess.check_output([str(binary),'--list'],text=True,cwd=archive/'hypercurve');assert all(name+': test' in listing for name in names)
 for name in names:
  log=A/f'{prefix}-{name.split("::")[-1]}.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,180);case.update(target='hypercurve',name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True);print(log.read_text()[-2500:],flush=True)
 report['probe_complete']=True;code=0 if all(case['passed']for case in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
