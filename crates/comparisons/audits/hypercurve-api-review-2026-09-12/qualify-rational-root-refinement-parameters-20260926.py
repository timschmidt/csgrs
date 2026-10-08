from pathlib import Path
import hashlib,json,os,re,subprocess,time,sys
A=Path(__file__).resolve().parent; W=A.parent
prior=json.loads((A/'rational-root-refinement-20260926-v198-terminal.json').read_text())
assert prior['all_processes_reaped'] and prior['all_sources_unchanged']
manifest=json.loads((A/prior['source_manifest']).read_text()); build=Path(prior['build_source_directory']); archive=Path(prior['source_directory'])
prefix='rational-root-refinement-20260926-v198-parameters';assert not (A/(prefix+'-terminal.json')).exists()
report=dict(parents=prior['parents'],source_manifest=prior['source_manifest'],source_directory=str(archive),build_source_directory=str(build),binaries=prior['binaries'],checks=[],cases=[],all_processes_reaped=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));tc=Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin');cargo=str(tc/'cargo')
def verify():
 for n,h in manifest.items():
  for root in [W,build,archive]: assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,n
 for repo,h in report['parents'].items():assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip()==h
 for b in report['binaries'].values():assert hashlib.sha256(Path(b['path']).read_bytes()).hexdigest()==b['sha256']
def save(): (A/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
def run(label,cmd,limit,group):
 report['active']=label;save();log=A/(prefix+'-'+label+'.log');start=time.monotonic()
 with log.open('w') as out:
  try:code=subprocess.run(cmd,cwd=build/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=limit).returncode
  except subprocess.TimeoutExpired:code='timeout'
 row=dict(label=label,repo='hypercurve',command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name);report[group].append(row);report.pop('active');save();print(label,code,round(row['elapsed_seconds'],3),log.read_text()[-2000:] if code else '',flush=True);return row
verify()
binary=prior['binaries']['hypercurve']['path']
names=[l[:-6] for l in subprocess.check_output([binary,'--list'],text=True).splitlines() if l.endswith(': test')]
jobs=[n for n in names if n.startswith('bezier_parameter::conversion_tests::')]
assert 'bezier_parameter::conversion_tests::root_refinement_does_not_embed_irrational_isolator_bounds' in jobs
assert len(jobs)>40
report['selection']=jobs;save();print('selected',len(jobs),'parameter cases',flush=True)
for i,name in enumerate(jobs):
 row=run(f'case-{i:02}',[binary,'--exact',name,'--nocapture','--test-threads=1','--color','never'],180,'cases');row['name']=name;save()
 if row['returncode']:break
 assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;', (A/row['log']).read_text())
if all(not r['returncode'] for r in report['cases']):
 for label,cmd in [
  ('fmt',[str(tc/'rustfmt'),'--edition','2024','--check','src/bezier_parameter.rs']),
  ('clippy-all',[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings']),
  ('clippy-default',[cargo,'clippy','--all-targets','--no-default-features','--locked','--offline','--','-D','warnings']),
 ]:
  if run(label,cmd,1200,'checks')['returncode']:break
verify();report['all_sources_unchanged']=True;report['all_processes_reaped']=True;report['qualification_complete']=len(report['cases'])==len(jobs) and len(report['checks'])==3 and all(not r['returncode'] for r in report['cases']+report['checks']);save();print('all reaped; complete=',report['qualification_complete'],flush=True);sys.exit(0 if report['qualification_complete'] else 1)
