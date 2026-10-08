from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='constrained-fillet-api-20260926-v304';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'constrained-fillet-api-20260926-v302-sources.json').read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha or name.startswith('hypercurve/'),name
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
 verify();(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def compile_binary(crate,target,selector):
 cmd=[cargo,'test',*selector,'--release','--all-features','--no-run','--message-format=json','--locked','--offline'];log=A/f'{prefix}-{target}-build.log';start=time.monotonic()
 with log.open('w')as out:code=subprocess.run(cmd,cwd=build/crate,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
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
  try:code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=240).returncode
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(binary=binary.name,name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));save();print(name,code,round(time.monotonic()-start,2),flush=True)
 if code:print(log.read_text()[-3000:],flush=True);raise RuntimeError('case failure')
def run_record(label,cmd,cwd,timeout=900):
 log=A/f'{prefix}-{label}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=subprocess.run(cmd,cwd=cwd,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=timeout).returncode
  except subprocess.TimeoutExpired:code=124
 record=dict(label=label,command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start);report['checks'].append(record);save();print(label,code,round(record['elapsed_seconds'],2),flush=True)
 if code:
  with log.open()as f:f.seek(max(0,log.stat().st_size-5000));print(f.read()[-3000:],flush=True)
  raise RuntimeError(label+' failed')
 return log,record
verify();code=0;report['checks']=[];report['qualification_scope']='Constrained fillet public API and direct caller migration; authored parameter charts, isolated and continuous solutions, region compositions'
try:
 for label,features in [('clippy-all-features',['--all-features']),('clippy-no-default',['--no-default-features'])]:
  run_record(label,[cargo,'clippy','--all-targets',*features,'--locked','--offline','--','-D','warnings'],build/'hypercurve')
 changed=subprocess.check_output(['git','diff','--name-only'],cwd=W/'hypercurve',text=True).splitlines()
 rust=[p for p in changed if p.endswith('.rs')]
 run_record('format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check',*rust],build/'hypercurve')
 report['known_unresolved_not_requalified']=['strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions']
 targets=['hypercurve','hypercurve_curve','hypercurve_curve_region_promotion','hypercurve_path_closure','hypercurve_analytic_parallel_region','hypercurve_curve_intersection','hypercurve_curve_region_boolean']
 report['expected_cases']={};save()
 for target in targets:
  binary=compile_binary('hypercurve',target,['--lib'] if target=='hypercurve' else ['--test',target])
  names=subprocess.check_output([str(binary),'--list'],text=True).splitlines();names=[n.removesuffix(': test')for n in names if n.endswith(': test')]
  selected=[n for n in names if ('fillet' in n or (target=='hypercurve' and ('parameter_component::' in n or 'circular_spline_contact_parameters_distinguish_repeated_point_visits' in n))) and n not in report['known_unresolved_not_requalified']]
  selected=sorted(selected,key=lambda n:('circular_spline_contact_parameters_distinguish_repeated_point_visits' not in n,n))
  assert selected,target
  report['expected_cases'][target]=selected;save()
  for name in selected:case(binary,name)
 run_record('fuzz-check',[cargo,'check','--manifest-path','fuzz/Cargo.toml','--bin','curve_string_editing','--locked','--offline'],build/'hypercurve')
 env['RUSTDOCFLAGS']='-D warnings'
 run_record('documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],build/'hypercurve')
 run_record('hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],build/'hyperbrep')
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('scoped_qualification_complete',code==0,flush=True)
raise SystemExit(code)
