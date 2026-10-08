from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='initial-endpoint-root-replay-20260927-v360';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'cached-tower-approximation-20260927-v357-sources.json').read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha or name=='hypersolve/src/root_sign.rs',name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha

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
  try:code=run_owned([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=240)
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(binary=binary.name,name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));save();print(name,code,round(time.monotonic()-start,2),flush=True)
 if code:print(log.read_text()[-3000:],flush=True);raise RuntimeError('case failure')
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
verify();code=0;report['checks']=[];report['qualification_scope']='Native selected-root signs with undecided initial isolator endpoint signs'
try:
 binary=compile_binary('hypersolve','hypersolve',['--lib'])
 case(binary,'root_sign::tests::narrow_isolators_replay_before_initial_endpoint_signs_are_available')
 run_record('hypersolve-full-lib',[str(binary),'--nocapture','--test-threads=2'],archive/'hypersolve',900)
 integration=[p.stem for p in sorted((archive/'hypersolve/tests').glob('*.rs'))]
 selectors=[arg for name in integration for arg in ['--test',name]]
 run_record('hypersolve-integration',[cargo,'test',*selectors,'--release','--all-features','--locked','--offline','--','--test-threads=2'],build/'hypersolve',900)
 run_record('hypersolve-clippy',[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings'],build/'hypersolve')
 run_record('format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check','src/root_sign.rs'],build/'hypersolve')
 binary=compile_binary('hypercurve','hypercurve',['--lib'])
 prior_geometry=json.loads((A/'cached-tower-approximation-20260927-v357-terminal.json').read_text())
 names=prior_geometry['expected_cases']['hypercurve']
 selected=[n for n in names if n.startswith('bezier_offset::conversion_tests::') or n.startswith('bezier_offset::parameter_component::') or n.startswith('rational_bezier_general::')]
 selected+=['bezier_region::tests::one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope','curve::curve_fillet::tests::continuous_fillet_constraints_accept_arbitrary_exact_real_contacts','curve::curve_fillet::tests::collapsed_selected_circle_fillet_reuses_constrained_contacts_and_support','curve::curve_fillet::tests::native_circle_fillet_retains_selected_contacts_through_reconstruction']
 report['expected_geometry_cases']=selected;save()
 for name in selected:case(binary,name)
 env['RUSTDOCFLAGS']='-D warnings'
 run_record('hypersolve-documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],build/'hypersolve')
 run_record('hypercurve-clippy',[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings'],build/'hypercurve')
 report['known_unresolved']=['strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions']
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('scoped_qualification_complete',code==0,flush=True)
raise SystemExit(code)
