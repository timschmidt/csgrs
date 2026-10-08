from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='coarser-singleton-sign-followup-20260926-v283';build=A/'build-workspace-20260925'
prior=json.loads((A/'coarser-singleton-sign-20260926-v282-terminal.json').read_text());assert prior['all_processes_reaped'];assert prior['checks'][0]['passed']==525
manifest=json.loads((A/prior['source_manifest']).read_text());archive=Path(prior['source_directory'])
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
fixture=A/'fillet-selected-sign-export-20260926-v279-sign.jsonl';probe_source=A/'fillet-selected-sign-replay-20260926-v280.rs';inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in [fixture,probe_source,Path(__file__).resolve()]}
report=dict(source_manifest=prior['source_manifest'],source_directory=str(archive),preceding_test_receipt='coarser-singleton-sign-20260926-v282-terminal.json',passed_tests=525,inputs=inputs,checks=[],all_processes_reaped=False)
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
 for p,sha in inputs.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha,p
 binary=prior['builds'][0]['binary'];assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
def save():
 verify();(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(label,cmd,cwd,timeout=900):
 log=A/f'{prefix}-{label}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=subprocess.run(cmd,cwd=cwd,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=timeout).returncode
  except subprocess.TimeoutExpired:code=124
 report['checks'].append(dict(label=label,command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));save();print(label,code,round(time.monotonic()-start,2),flush=True)
 if code:print(log.read_text()[-5000:],flush=True);raise RuntimeError(label+' failed')
 return log
verify();code=0
try:
 liblog=run('hypersolve-library-build',[cargo,'build','--lib','--release','--all-features','--features','hyperreal/serde','--message-format=json','--locked','--offline'],build/'hypersolve')
 libs={};native=[]
 for line in liblog.read_text().splitlines():
  try:r=json.loads(line)
  except ValueError:continue
  if r.get('reason')=='compiler-artifact'and r['target']['name'] in ['hyperreal','hypersolve','hyperlimit']:
   rl=[Path(f)for f in r['filenames']if f.endswith('.rlib')]
   if rl:libs[r['target']['name']]=rl[0]
  if r.get('reason')=='build-script-executed':native.extend(r['linked_paths'])
 assert set(libs)=={'hyperreal','hypersolve','hyperlimit'}
 for p in libs.values():inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
 scalar=A/f'{prefix}-scalar';cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-C','opt-level=3',str(probe_source),'-o',str(scalar),'-L','dependency='+str(next(iter(libs.values())).parent)]
 for name,p in libs.items():cmd+=['--extern',name+'='+str(p)]
 for p in native:cmd+=['-L',p]
 run('scalar-build',cmd,archive,120);report['scalar_binary']=dict(path=str(scalar),sha256=hashlib.sha256(scalar.read_bytes()).hexdigest());save()
 log=run('scalar-replay',[str(scalar),str(fixture),'native'],archive,40);result=log.read_text();assert 'mode=native sign=Some(Greater)' in result;print(result[-1000:],flush=True)
 for label,cmd in [('format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check','src/root_sign.rs']),('clippy-all-features',[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings']),('clippy-no-default',[cargo,'clippy','--all-targets','--no-default-features','--locked','--offline','--','-D','warnings'])]:run(label,cmd,build/'hypersolve')
except Exception as error:code=1;report['failure']=str(error)
finally:report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('qualification_complete',code==0,flush=True)
raise SystemExit(code)
