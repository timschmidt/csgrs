from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-contact-signs-20260927-v498';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
probe=json.loads((A/'stationary-contact-signs-20260927-v497-terminal.json').read_text());assert probe['probe_complete'] and probe['all_processes_reaped'] and all(c['passed'] for c in probe['cases'])
prior=json.loads((A/'fillet-retained-domains-20260927-v495-sources.json').read_text());manifest={};assert not archive.exists()
allowed={'hypercurve/src/bezier_offset.rs'}
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert name in allowed or sha==old_sha,name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if not target.exists() or target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha
assert {name for name,sha in manifest.items()if sha!=prior[name]}==allowed

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),cases=[],builds=[],all_processes_reaped=False)
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name

def save():
 (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run_owned(cmd, *, cwd, env, stdout, stderr, timeout):
 process=subprocess.Popen(cmd,cwd=cwd,env=env,stdout=stdout,stderr=stderr,start_new_session=True)
 try:return process.wait(timeout=timeout)
 except BaseException:
  os.killpg(process.pid,signal.SIGKILL);process.wait();raise

def compile_binary(crate,target,selector):
 cmd=[cargo,'test',*selector,'--release','--all-features','--no-run','--message-format=json','--locked','--offline'];log=A/f'{prefix}-{target}-build.log';start=time.monotonic()
 with log.open('w')as out:code=run_owned(cmd,cwd=build/crate,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900)
 record=dict(crate=crate,command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start);report['builds'].append(record);save()
 if code:
  for line in log.read_text().splitlines():
   try:r=json.loads(line)
   except ValueError:continue
   if r.get('reason')=='compiler-message'and r['message']['level']=='error':print(r['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('build failure')
 rows=[]
 for line in log.read_text().splitlines():
  try:rows.append(json.loads(line))
  except ValueError:pass
 row=next(r for r in rows if r.get('reason')=='compiler-artifact'and r['target']['name']==target and r.get('executable'))
 binary=A/f'{prefix}-{target}';shutil.copy2(row['executable'],binary);record['binary']=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());save();return binary

def case(binary,name):
 log=A/f'{prefix}-case-{len(report["cases"]):03d}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=run_owned([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=360 if "trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions" in name else 240)
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(binary=binary.name,name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));save();print(name,code,round(time.monotonic()-start,2),flush=True)
 if code or not re.search(r'test result: ok\. 1 passed;',log.read_text()):print(log.read_text()[-3000:],flush=True);raise RuntimeError('case failure or missing exact test')
def run_record(label,cmd,cwd,timeout=900):
 log=A/f'{prefix}-{label}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=run_owned(cmd,cwd=cwd,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=timeout)
  except subprocess.TimeoutExpired:code=124
 record=dict(label=label,command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start);report['checks'].append(record);save();print(label,code,round(record['elapsed_seconds'],2),flush=True)
 if code:
  with log.open()as f:f.seek(max(0,log.stat().st_size-5000));print(f.read()[-3000:],flush=True)
  raise RuntimeError(label+' failed')
 return log,record
verify();code=0;report['checks']=[];report['qualification_scope']='Contact-local parallel tangent signs replace range-midpoint orientation; stationary endpoints retain exact one-sided polynomial signs'
try:
 broad=json.loads((A/'fillet-retained-domains-20260927-v495-terminal.json').read_text())
 assert broad['qualification_complete'] and broad['all_processes_reaped']
 expected={target:list(names)for target,names in broad['expected_cases'].items()}
 assert sum(map(len,expected.values()))==353
 expected['hypercurve'].extend(probe['expected_cases'])
 report['known_unresolved']=['Stationary parallel-pair fillet dispatch and owned cusp contact frames remain a separate open repair']
 report['expected_cases']=expected;save()
 binary=compile_binary('hypercurve','hypercurve',['--lib'])
 first_cases=['curve::curve_fillet::tests::joined_path_selects_and_replays_a_continuous_fillet_family','curve::curve_fillet::tests::nonlinear_linear_fillet_components_retain_unique_contacts_and_tangents']
 for name in first_cases:case(binary,name)
 medians=json.loads((A/'newton-fillet-paired-20260927-v467-terminal.json').read_text())['medians']
 for result in report['cases']:
  assert result['elapsed_seconds']<=medians[result['name']]['before']*1.1+0.1,('rational dispatch did not restore baseline cost',result['name'])
 report['initial_dispatch_check']='Both previously regressed fillet workloads return near their paired baseline';save()
 for label,features in [('hypercurve-clippy-all-features',['--all-features']),('hypercurve-clippy-no-default',['--no-default-features'])]:
  run_record(label,[cargo,'clippy','--all-targets',*features,'--locked','--offline','--','-D','warnings'],build/'hypercurve')
 rustfmt='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt'
 run_record('hypercurve-format',[rustfmt,'--edition','2024','--config','skip_children=true','--check','src/bezier_offset.rs'],build/'hypercurve')
 run_record('fuzz-check',[cargo,'check','--manifest-path','fuzz/Cargo.toml','--bin','curve_string_editing','--locked','--offline'],build/'hypercurve')
 env['RUSTDOCFLAGS']='-D warnings'
 for crate in ['hypercurve']:
  run_record(crate+'-documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],build/crate)
 run_record('hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],build/'hyperbrep')
 new_binary=compile_binary('hypercurve','hypercurve_stationary_fillets',['--test','hypercurve_stationary_fillets'])
 for name in expected['hypercurve_stationary_fillets']:case(new_binary,name)
 for target,names in expected.items():
  if target=='hypercurve_stationary_fillets':continue
  if target!='hypercurve':binary=compile_binary('hypercurve',target,['--test',target])
  for name in names:
   if not(target=='hypercurve'and name in first_cases):case(binary,name)
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('broad_qualification_complete',code==0,flush=True)
raise SystemExit(code)
