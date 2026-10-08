from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-linear-component-replay-20260927-v407';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'fillet-component-witnesses-20260927-v401-sources.json').read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha or name in ['hypercurve/src/curve_fillet.rs','hypercurve/src/bezier_offset.rs','hypercurve/src/curve.rs'],name
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
verify();code=0;report['checks']=[];report['qualification_scope']='Retained chord fillet constraints reuse one affine chart, retain original chord locations, and respect empty polynomial zeros'
try:
 broad=json.loads((A/'fillet-component-witnesses-20260927-v401-terminal.json').read_text())
 focused=json.loads((A/'retained-linear-component-replay-20260927-v406-terminal.json').read_text())
 assert broad['qualification_complete'] and broad['all_processes_reaped'] and focused['qualification_complete'] and focused['all_processes_reaped']
 assert manifest==json.loads((A/focused['source_manifest']).read_text())
 expected={target:list(names)for target,names in broad['expected_cases'].items()}
 expected['hypercurve'].append('bezier_offset::conversion_tests::bivariate_products_accept_empty_zero_coefficients')
 for name in focused['expected_cases']['hypercurve']:
  if name not in expected['hypercurve']:expected['hypercurve'].append(name)
 report['expected_cases']=expected;report['known_unresolved']=broad['known_unresolved'];report['reused_source_qualification']='retained-linear-component-replay-20260927-v406-terminal.json';save()
 for label,features in [('hypercurve-clippy-all-features',['--all-features']),('hypercurve-clippy-no-default',['--no-default-features'])]:
  run_record(label,[cargo,'clippy','--all-targets',*features,'--locked','--offline','--','-D','warnings'],build/'hypercurve')
 run_record('hypercurve-format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check','src/curve.rs','src/curve_fillet.rs','src/bezier_offset.rs','src/curve_corner_chain.rs'],build/'hypercurve')
 run_record('fuzz-check',[cargo,'check','--manifest-path','fuzz/Cargo.toml','--bin','curve_string_editing','--locked','--offline'],build/'hypercurve')
 env['RUSTDOCFLAGS']='-D warnings'
 run_record('hypercurve-documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],build/'hypercurve')
 run_record('hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],build/'hyperbrep')
 reusable={r['name']:r for r in focused['cases']}
 focused_binary=focused['builds'][0]['binary']
 for target,names in expected.items():
  binary=compile_binary('hypercurve',target,['--lib'] if target=='hypercurve' else ['--test',target])
  for name in names:
   if target=='hypercurve' and name in reusable:
    assert hashlib.sha256(binary.read_bytes()).hexdigest()==focused_binary['sha256']
    record=dict(reusable[name]);assert record['returncode']==0 and 'test result: ok. 1 passed;' in (A/record['log']).read_text();record['binary']=binary.name;record['reused_from']=report['reused_source_qualification'];report['cases'].append(record);save()
   else:case(binary,name)
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('broad_qualification_complete',code==0,flush=True)
raise SystemExit(code)
