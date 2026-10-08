from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-circular-constraint-trace-20260927-v383';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'coincident-circular-constraints-20260927-v382-sources.json').read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha or name in ['hypercurve/src/curve_fillet.rs','hypercurve/src/curve.rs'],name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
trial={}
for root in [archive,build]:
 p=root/'hypercurve/src/error.rs';s=p.read_text();s=s.replace('    pub(crate) const fn blocked(', '    #[track_caller]\n    pub(crate) fn blocked(');s=s.replace('        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))', '        eprintln!("blocked {operation:?} {family:?} {reason:?} at {}", std::panic::Location::caller());\n        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))');p.write_text(s)
 p=root/'hypercurve/src/curve_fillet.rs';s=p.read_text();start=s.index('    fn coincident_circular_fillet_constraints_complete_the_authored_chart_solutions()');end=s.index('    #[test]',start);part=s[start:end];part=part.replace('                            let selected = path.fillet_vertex', '                            eprintln!("constraint={constraint}, reversed={reversed}, mode={mode:?}, past_center={past_center}");\n                            let selected = path.fillet_vertex');s=s[:start]+part+s[end:];p.write_text(s)
for n in ['hypercurve/src/error.rs','hypercurve/src/curve_fillet.rs']:trial[n]=hashlib.sha256((archive/n).read_bytes()).hexdigest()
(A/f'{prefix}-trial.json').write_text(json.dumps(trial,indent=2)+'\n')

env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),cases=[],builds=[],all_processes_reaped=False)
def verify():
 for name,sha in manifest.items():
  assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
  for root in [archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==trial.get(name,sha),name

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
verify();code=0;report['checks']=[];report['qualification_scope']='Coincident native circular fillet families retain exact constraints and authored domains'
try:
 binary=compile_binary('hypercurve','hypercurve',['--lib'])
 case(binary,'curve::curve_fillet::tests::coincident_circular_fillet_constraints_complete_the_authored_chart_solutions')
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('focused_qualification_passed',code==0,flush=True)
raise SystemExit(code)
