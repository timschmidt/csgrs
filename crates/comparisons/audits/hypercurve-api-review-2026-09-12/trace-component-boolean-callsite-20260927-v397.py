from pathlib import Path
import hashlib,json,os,subprocess,time,shutil,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='trace-component-boolean-callsite-20260927-v397';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
manifest=json.loads((A/'fillet-component-witnesses-20260927-v395-sources.json').read_text());assert not archive.exists()
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for name,h in manifest.items():
 source=W/name;assert sha(source)==h;target=archive/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
 target=build/name
 if target.read_bytes()!=source.read_bytes():shutil.copy2(source,target);os.utime(target,None)
for root in [archive,build]:
 p=root/'hypercurve/src/error.rs';s=p.read_text().replace('    pub(crate) const fn blocked(', '    #[track_caller]\n    pub(crate) fn blocked(').replace('        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))','        eprintln!("blocked {operation:?} {family:?} {reason:?} at {}", std::panic::Location::caller());\n        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))');p.write_text(s)
for root in [archive,build]:
 p=root/'hypercurve/src/curve_region_boolean.rs';s=p.read_text().replace('    fn blocked(&self, carrier_index:', '    #[track_caller]\n    fn blocked(&self, carrier_index:');p.write_text(s)
trial={n:sha(archive/n) for n in ['hypercurve/src/error.rs','hypercurve/src/curve_region_boolean.rs']}
source=A/f'{prefix}.rs';binary=A/prefix
report=dict(source_manifest='fillet-component-witnesses-20260927-v395-sources.json',source_directory=str(archive),trial=trial,probe_sha256=sha(source),records=[],all_processes_reaped=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
def verify():
 for n,h in manifest.items():
  assert sha(W/n)==h,n
  for root in [archive,build]:assert sha(root/n)==trial.get(n,h),(root,n)
 assert sha(source)==report['probe_sha256']
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(label,cmd,cwd,timeout):
 log=A/f'{prefix}-{label}.log';start=time.monotonic()
 with log.open('w')as out:
  p=subprocess.Popen(cmd,cwd=cwd,env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
  try:code=p.wait(timeout=timeout)
  except BaseException:
   os.killpg(p.pid,signal.SIGKILL);p.wait();raise
 report['records'].append(dict(label=label,returncode=code,elapsed_seconds=time.monotonic()-start,command=cmd,log=log.name));save();print(label,code,round(time.monotonic()-start,2),flush=True)
 if code:print(log.read_text()[-2500:],flush=True);raise RuntimeError(label)
 return log
verify();code=0
try:
 log=run('library-build',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo','build','--lib','--release','--all-features','--message-format=json','--locked','--offline'],build/'hypercurve',900)
 rows=[]
 for line in log.read_text().splitlines():
  try:rows.append(json.loads(line))
  except ValueError:pass
 artifact=next(r for r in rows if r.get('reason')=='compiler-artifact'and r['target']['name']=='hypercurve'and any(n.endswith('.rlib')for n in r['filenames']))
 library=Path(next(n for n in artifact['filenames']if n.endswith('.rlib')));report['library']=dict(path=str(library),sha256=sha(library));deps=library.parent/'deps' if(library.parent/'deps').is_dir()else library.parent
 run('probe-build',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-C','opt-level=3',str(source),'--extern',f'hypercurve={library}','-L',f'dependency={deps}','-o',str(binary)],A,180)
 report['binary']=dict(path=str(binary),sha256=sha(binary));log=run('probe',[str(binary)],A,120);print(log.read_text()[:5500],flush=True)
 assert sha(library)==report['library']['sha256'] and sha(binary)==report['binary']['sha256']
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;report['diagnostic_complete']=code==0;save()
raise SystemExit(code)
