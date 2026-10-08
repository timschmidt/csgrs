from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='conic-subdivision-20260928-v653';build=A/'build-workspace-20260925'
prior_prefix='conic-subdivision-20260928-v651'
assert json.loads((A/f'{prior_prefix}-reaped.json').read_text())['outer_exit_code']==0
promotion=json.loads((A/'conic-subdivision-promotion-v652.json').read_text());assert promotion['production_matches_tested_source']
report=json.loads((A/f'{prior_prefix}-terminal.json').read_text());assert report['probe_complete']and report['all_processes_reaped']and len(report['cases'])==122 and all(c['passed']for c in report['cases'])
manifest=json.loads((A/report['source_manifest']).read_text());archive=Path(report['source_directory']);changed=set(promotion['changed_files'])
assert len(changed)==7
report.update(promoted_candidate_qualification=True,tests_reused_from=f'{prior_prefix}-terminal.json',all_processes_reaped=False,qualification_complete=False,checks=[])
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for row in report['builds']:
  for artifact in row['binaries'].values():assert hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest()==artifact['sha256']
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
  ('fuzz-check',[cargo,'check','--manifest-path','fuzz/Cargo.toml','--bin','curve_string_editing','--bin','bezier_arrangement','--bin','bezier_region','--bin','bezier_split_materialization','--locked','--offline'],'hypercurve'),
  ('hypercurve-documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],'hypercurve'),
  ('hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],'hyperbrep'),
 ]
 env['RUSTDOCFLAGS']='-D warnings'
 for label,command,crate in checks:
  log=A/f'{prefix}-{label}.log';row=run(command,build/crate,log,900);row['label']=label;report['checks'].append(row);save();print(label,row['returncode'],round(row['elapsed_seconds'],3),flush=True)
  if row['returncode']:print(log.read_text()[-5000:],flush=True);raise RuntimeError(label+' failed')
 report['qualification_complete']=True
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
