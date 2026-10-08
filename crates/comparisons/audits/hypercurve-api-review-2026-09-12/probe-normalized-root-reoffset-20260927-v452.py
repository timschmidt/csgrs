from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='normalized-root-reoffset-20260927-v452';build=A/'build-workspace-20260925'
prior=json.loads((A/'root-sign-tower-branches-20260927-v451-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
base=Path(prior['source_directory']);manifest=json.loads((A/prior['source_manifest']).read_text());trial=dict(prior['trial']);archive=A/'source-archives'/prefix;assert not archive.exists()
for name,sha in manifest.items():
 src=base/name;assert hashlib.sha256(src.read_bytes()).hexdigest()==trial.get(name,sha),name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
 if (build/name).read_bytes()!=src.read_bytes():shutil.copy2(src,build/name);os.utime(build/name,None)


name='hypercurve/tests/hypercurve_curve_region_promotion.rs'
for root in [archive,build]:
 p=root/name;value=p.read_text()
 anchor='''    for (corner, candidate, filleted) in filleted {
        let fillet_circle_count = filleted.boundary_loops()[0]
            .curves()
            .iter()'''
 replacement='''    for (corner, candidate, result) in &filleted {
        let counts = result.boundary_loops().iter().map(|boundary| {
            (boundary.curves().len(), boundary.curves().iter()
                .filter(|fragment| fragment.family() == CurveFamily2::CircularArc).count())
        }).collect::<Vec<_>>();
        eprintln!("fillet layout mode={mode:?} corner={corner} candidate={candidate} loops={counts:?}");
    }
    for (corner, candidate, filleted) in filleted {
        let fillet_circle_count = filleted.boundary_loops()
            .iter()
            .flat_map(|boundary| boundary.curves())'''
 assert value.count(anchor)==1;p.write_text(value.replace(anchor,replacement,1))
trial[name]=hashlib.sha256((archive/name).read_bytes()).hexdigest()

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),cases=[],builds=[],trial=trial,all_processes_reaped=False)
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==(sha if root==W else trial.get(name,sha)),name

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
  try:code=run_owned([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=360)
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
verify();code=0;report['checks']=[];report['qualification_scope']='Extended-fillet re-offset with reduced normalized root queries, tower branch signs, and fillet chart ownership across all regularized loops'
try:
 expected={'hypercurve_curve_region_promotion':prior['known_unresolved']}
 report['expected_cases']=expected;save()
 binary=compile_binary('hypercurve','hypercurve_curve_region_promotion',['--test','hypercurve_curve_region_promotion'])
 for name in expected['hypercurve_curve_region_promotion']:case(binary,name)
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('composition_qualification_complete',code==0,flush=True)
raise SystemExit(code)
