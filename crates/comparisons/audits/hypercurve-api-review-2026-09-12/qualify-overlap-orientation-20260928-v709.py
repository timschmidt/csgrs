from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='overlap-orientation-20260928-v709';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
assert (A/'native-path-shims-20260928-v707-committed.json').exists()
prior=json.loads((A/'native-path-shims-20260928-v707-terminal.json').read_text());baseline=json.loads((A/prior['source_manifest']).read_text());assert len(baseline)==2047
promotion=json.loads((A/'overlap-orientation-promotion-v708.json').read_text());changed=set(promotion['promoted']);manifest={};assert not archive.exists()
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for name,sha in baseline.items():
 data=(W/name).read_bytes();manifest[name]=hashlib.sha256(data).hexdigest()
 if name not in changed:assert manifest[name]==sha,name
 else:assert manifest[name]==promotion['promoted'][name],name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
assert {name for name in manifest if manifest[name]!=baseline[name]}==changed
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
report=dict(normal_production_checks=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),checks=[],all_processes_reaped=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));env['RUSTDOCFLAGS']='-D warnings';cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert digest(root/name)==sha,(root,name)
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(command,cwd,log,timeout):
 start=time.monotonic()
 with log.open('w')as out:
  process=subprocess.Popen(command,cwd=cwd,stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:code=process.wait(timeout=timeout)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 return dict(command=command,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start)
verify();save();code=0
try:
 checks=[
  ('hypercurve-clippy-all-features',[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings'],'hypercurve'),
  ('hypercurve-clippy-no-default',[cargo,'clippy','--all-targets','--no-default-features','--locked','--offline','--','-D','warnings'],'hypercurve'),
  ('format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check',*[str(build/n)for n in sorted(changed)]],'hypercurve'),
  ('fuzz-check',[cargo,'check','--manifest-path','fuzz/Cargo.toml','--bin','bezier_arrangement','--bin','bezier_region','--bin','retained_import','--locked','--offline'],'hypercurve'),
  ('hypercurve-documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],'hypercurve'),
  ('hyperbrep-clippy',[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings'],'hyperbrep'),
 ]
 for label,command,crate in checks:
  log=A/f'{prefix}-{label}.log';check=run(command,build/crate,log,900);check['label']=label;report['checks'].append(check);save();print(label,check['returncode'],round(check['elapsed_seconds'],3),flush=True)
  if check['returncode']:print(log.read_text()[-4000:],flush=True);raise RuntimeError(label+' failed')
 report['qualification_complete']=True
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
