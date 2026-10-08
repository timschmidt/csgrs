from pathlib import Path
import ast,hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='interpolation-content-20260928-v701';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
probe_prefix='interpolation-content-20260928-v697'
assert json.loads((A/f'{probe_prefix}-reaped.json').read_text())['outer_exit_code']==0
probe=json.loads((A/f'{probe_prefix}-terminal.json').read_text());assert probe['probe_complete']and probe['all_processes_reaped']
manifest=json.loads((A/probe['source_manifest']).read_text());assert len(manifest)==2047
baseline=json.loads((A/'projection-overlap-20260928-v695-sources.json').read_text())
changed={'hypersolve/src/curve_resultant.rs'}
assert {name for name in manifest if manifest[name]!=baseline[name]}==changed
assert (A/'interpolation-content-promotion-v700.json').exists()
assert not archive.exists()
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for name,sha in manifest.items():
 for root in [W,Path(probe['source_directory']),build]:assert digest(root/name)==sha,(root,name)
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes((W/name).read_bytes())
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
library=json.loads((A/'projection-overlap-20260928-v695-terminal.json').read_text())['expected_cases']['hypercurve']
assert len(library)==25
names={'hypersolve':probe['expected_cases']['hypersolve'],'hypercurve':library,'hypercurve_curve':[],'hypercurve_curve_intersection':[],'hypercurve_analytic_parallel_region':[],'hypercurve_stationary_fillets':[]}
report=dict(normal_production_build=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],test_listings={},expected_cases=names,reused_qualification=f'{probe_prefix}-terminal.json',all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
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
def compile_tests(crate,targets):
 log=A/f'{prefix}-{crate}-build.log';row=run([cargo,'test','--lib',*[arg for target in targets if target!=crate for arg in ['--test',target]],'--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/crate,log,900);report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError(crate+' normal release build failed')
 artifacts={}
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  target=v.get('target',{}).get('name')
  if v.get('reason')=='compiler-artifact'and target in targets and v.get('executable'):artifacts[target]=v
 assert set(artifacts)==set(targets)
 binaries={}
 for target,artifact in artifacts.items():
  binary=A/f'{prefix}-{target}-tests';shutil.copy2(artifact['executable'],binary);binaries[target]=dict(path=str(binary),sha256=digest(binary))
  log=A/f'{prefix}-{target}-list.log';listing=run([str(binary),'--list'],archive/crate,log,30);report['test_listings'][target]=listing;assert listing['returncode']==0
  available={line.removesuffix(': test')for line in log.read_text().splitlines()if line.endswith(': test')}
  assert set(names[target])<=available
  if target not in {'hypersolve','hypercurve'}:names[target]=sorted(available)
 row['binaries']=dict(binaries);save();print(crate,'compiled',round(row['elapsed_seconds'],3),flush=True);return binaries
verify();save();code=0
try:
 binaries=compile_tests('hypersolve',['hypersolve']);binaries.update(compile_tests('hypercurve',[n for n in names if n!='hypersolve']))
 # Reuse only after normal Cargo builds reproduce both original executable hashes.
 for old_build in probe['builds']:
  for target,old_binary in old_build['binaries'].items():
   assert digest(Path(old_binary['path']))==old_binary['sha256']==binaries[target]['sha256'],target
 for case in probe['cases']:
  assert case['passed']and case['returncode']==0 and case['name']in names[case['target']]
  assert 'test result: ok. 1 passed;'in(A/case['log']).read_text()
  report['cases'].append(dict(case,reused_from=f'{probe_prefix}-terminal.json'))
 assert len(report['cases'])==220
 for check in probe['checks']:
  assert check['returncode']==0;report['checks'].append(dict(check,reused_from=f'{probe_prefix}-terminal.json'))
 assert len(report['checks'])==3
 done={(c['target'],c['name'])for c in report['cases']}
 save();print('Reused 220 exact-source/executable-matching cases and three checks; inventories',{k:len(v)for k,v in names.items()},flush=True)
 for target,selected in names.items():
  for name in selected:
   if (target,name)in done:continue
   log=A/f'{prefix}-{name.split("::")[-1]}-{hashlib.sha256((target+name).encode()).hexdigest()[:10]}.log';case=run([binaries[target]['path'],'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,240);case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
   if not case['passed']:print(log.read_text()[-3500:],flush=True);raise RuntimeError('caller regression failed: '+name)
 report['probe_complete']=True;save()
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
  log=A/f'{prefix}-{label}.log';check=run(command,build/crate,log,900);check['label']=label;report['checks'].append(check);save();print(label,check['returncode'],round(check['elapsed_seconds'],3),flush=True)
  if check['returncode']:print(log.read_text()[-4000:],flush=True);raise RuntimeError(label+' failed')
 report['qualification_complete']=True
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
