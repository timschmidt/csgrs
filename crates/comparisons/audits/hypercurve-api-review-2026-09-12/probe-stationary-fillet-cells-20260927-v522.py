from pathlib import Path
import hashlib,json,os,re,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-fillet-cells-20260927-v522';prior=json.loads((A/'empty-incident-components-20260927-v523-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
manifest=json.loads((A/prior['source_manifest']).read_text())
sealed=json.loads((A/'empty-incident-components-20260927-v523-committed.json').read_text())
library=None;receipt=None
fixture='stationary-fillet-line-chart-v516.rs';names=re.findall(r'#\[test\]\s*fn\s+(\w+)',(A/fixture).read_text());assert len(names)==16
assert len(set(names))==len(names)
groups=[('line_chart',fixture,names)]
fixtures={source:hashlib.sha256((A/source).read_bytes()).hexdigest()for _,source,_ in groups};report=dict(diagnostic_only=True,source_manifest=prior['source_manifest'],source_directory=prior['source_directory'],fixtures=fixtures,library=receipt,expected_cases=groups,builds=[],cases=[],all_processes_reaped=False,probe_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));expected_head=sealed['repos']['hypercurve']['commit']
def verify():
 assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip()==expected_head
 for name,sha in manifest.items():
  for root in [W,Path(prior['source_directory']),A/'build-workspace-20260925']:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for name,sha in fixtures.items():assert hashlib.sha256((A/name).read_bytes()).hexdigest()==sha,name
 if library is not None:assert hashlib.sha256(library.read_bytes()).hexdigest()==receipt['sha256']
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(command,log,timeout,cwd=W):
 start=time.monotonic()
 with log.open('w')as out:
  process=subprocess.Popen(command,cwd=cwd,stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:code=process.wait(timeout=timeout)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 return dict(command=command,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start)
verify();save();code=0
try:
 cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
 log=A/f'{prefix}-library-build.log';row=run([cargo,'build','--lib','--release','--all-features','--locked','--offline','--message-format=json'],log,900,A/'build-workspace-20260925/hypercurve');report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:message=json.loads(line)
   except ValueError:continue
   if message.get('reason')=='compiler-message'and message['message']['level']=='error':print(message['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('normal production library build failed')
 libraries=set()
 for line in log.read_text().splitlines():
  try:message=json.loads(line)
  except ValueError:continue
  if message.get('reason')=='compiler-artifact'and message.get('target',{}).get('name')=='hypercurve':libraries.update(name for name in message.get('filenames',[])if name.endswith('.rlib'))
 assert len(libraries)==1,libraries
 original_library=Path(libraries.pop());dependency=original_library.parent
 library=A/f'libhypercurve-{prefix}.rlib';assert not library.exists();library.write_bytes(original_library.read_bytes())
 receipt=dict(path=str(library),sha256=hashlib.sha256(library.read_bytes()).hexdigest(),qualified_library=str(original_library),dependency_directory=str(dependency),source_manifest=prior['source_manifest'])
 (A/f'{prefix}-library.json').write_text(json.dumps(receipt,indent=2)+'\n');report['library']=receipt;save();verify()
 for group,fixture,names in groups:
  source=A/fixture;binary=A/f'{prefix}-{group}-tests';log=A/f'{prefix}-{group}-build.log'
  row=run([env['RUSTC'],'--edition=2024','--test',str(source),'-C','opt-level=3','-L','dependency='+str(dependency),'--extern','hypercurve='+str(library),'-o',str(binary)],log,120);report['builds'].append(row);save()
  if row['returncode']:print(log.read_text()[-3000:],flush=True);raise RuntimeError(group+' build failed')
  row['binary']=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());save()
  for name in names:
   log=A/f'{prefix}-{group}-{name}.log';row=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],log,90);row.update(group=group,name=name,passed=row['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(row);save();print(group,name,row['returncode'],round(row['elapsed_seconds'],3),flush=True)
   if row['returncode']:print(log.read_text()[-1600:],flush=True)
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
