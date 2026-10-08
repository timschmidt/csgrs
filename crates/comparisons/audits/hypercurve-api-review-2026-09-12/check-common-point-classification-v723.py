from pathlib import Path
import hashlib,json,os,signal,subprocess,sys,time
A=Path(__file__).resolve().parent;W=A.parent;candidate=A/'common-point-classification-candidate-v722';build=A/'build-workspace-20260925'
round_id=sys.argv[1];crate=sys.argv[2]if len(sys.argv)>2 else'hypercurve';assert round_id.isdigit()and crate in {'hypercurve','hyperbrep','hypersdf','csgrs'}
prefix=f'common-point-classification-v723-r{round_id}-{crate}';archive=A/'source-archives'/prefix;assert not archive.exists()
baseline=json.loads((A/'retained-point-exterior-20260928-v720-sources.json').read_text());base=json.loads((A/'common-point-classification-base-v722.json').read_text());assert len(baseline)==2048
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
manifest={}
for name,sha in baseline.items():
 assert digest(W/name)==sha,name
 if name in base:assert base[name]==sha,name
 data=(candidate/name).read_bytes()if name in base else(W/name).read_bytes();manifest[name]=hashlib.sha256(data).hexdigest()
 path=archive/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
 path=build/name
 if path.read_bytes()!=data:path.write_bytes(data);os.utime(path,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
report=dict(source_manifest=f'{prefix}-sources.json',production_manifest='retained-point-exterior-20260928-v720-sources.json',source_directory=str(archive),candidate=str(candidate),crate=crate,all_processes_reaped=False,compile_passed=False)
def verify():
 for name,sha in baseline.items():assert digest(W/name)==sha,name
 for name,sha in manifest.items():
  for root in [archive,build]:assert digest(root/name)==sha,(root,name)
 for name in base:assert digest(candidate/name)==manifest[name],name
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
verify();save();code=0
try:
 command=[cargo,'check','--all-targets','--all-features','--locked','--offline','--message-format=json'];log=A/f'{prefix}-check.log';start=time.monotonic()
 with log.open('w')as out:
  process=subprocess.Popen(command,cwd=build/crate,stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:rc=process.wait(timeout=900)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();rc=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 report['check']=dict(command=command,log=log.name,returncode=rc,elapsed_seconds=time.monotonic()-start);messages=[]
 for line in log.read_text().splitlines():
  try:message=json.loads(line)
  except ValueError:continue
  if message.get('reason')=='compiler-message':messages.append(message)
 (A/f'{prefix}-diagnostics.json').write_text(json.dumps(messages,indent=2)+'\n')
 errors=[m['message']for m in messages if m['message']['level']=='error'];warnings=[m['message']for m in messages if m['message']['level']=='warning']
 report.update(compile_passed=rc==0,errors=len(errors),warnings=len(warnings),diagnostics=f'{prefix}-diagnostics.json');print('Compilation',rc,'seconds',round(report['check']['elapsed_seconds'],3),'errors',len(errors),'warnings',len(warnings),flush=True)
 for error in errors[:12]:
  spans=[dict(file=s['file_name'],line=s['line_start'],label=s.get('label'))for s in error['spans']if s['is_primary']]
  print(json.dumps(dict(message=error['message'],spans=spans)),flush=True)
 code=0 if rc==0 else 1
except Exception as error:code=1;report['failure']=str(error);print(type(error).__name__,str(error)[:1000],flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
