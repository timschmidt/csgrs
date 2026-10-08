from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prior_prefix='public-fillet-families-full-20260926-v236';prefix='public-fillet-families-coverage-20260926-v240'
report=json.loads((A/f'{prior_prefix}-terminal.json').read_text());assert report['all_processes_reaped']
manifest=json.loads((A/report['source_manifest']).read_text());archive=A/'source-archives'/prefix;build=Path(report['build_source_directory']);assert not archive.exists()
for name,sha in manifest.items():
 src=W/name;assert hashlib.sha256(src.read_bytes()).hexdigest()==sha,name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 if (build/name).read_bytes()!=src.read_bytes():shutil.copy2(src,build/name)
 assert dst.stat().st_ino!=(build/name).stat().st_ino
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
report.update(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),all_processes_reaped=False,continued_from=f'{prior_prefix}-terminal.json',prior_outer_session_id=82035,prior_outer_exit_code=1,diagnostic_outer_sessions=[{'id':19623,'exit':0},{'id':58519,'exit':0},{'id':33568,'exit':1}],unsuccessful_trial_reverted=True)
report.pop('qualification_complete',None)
for row in report['cases']:row.setdefault('reused_terminal',f'{prior_prefix}-terminal.json')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
def verify():
 for repo,head in report['parents'].items():assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip()==head
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for binary in report['binaries'].values():assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
def save(): (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
verify();save();complete={(r['target'],r['name'])for r in report['cases']};remaining=[tuple(job)for job in report['selection']if tuple(job)not in complete]
for index,(target,name)in enumerate(remaining):
 label=f'case-{index:03}';report['active']=name;save();log=A/f'{prefix}-{label}.log';start=time.monotonic();binary=report['binaries'][target];cmd=[binary['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never'];limit=300 if 'one_fragment_nonzero_parallel_loop_extends_chamfer'in name else 180
 with log.open('w')as out:
  try:code=subprocess.run(cmd,cwd=build/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=limit).returncode
  except subprocess.TimeoutExpired:code=124
 row=dict(repo='hypercurve',target=target,name=name,label=label,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name);report['cases'].append(row);report.pop('active');save();print(name,code,flush=True)
 if code==0:assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',log.read_text()),name
verify();assert len(report['cases'])==len(report['selection']);report['all_processes_reaped']=True;report['coverage_complete']=True;report['unresolved_failures']=[{k:r[k]for k in ['target','name','returncode','log']}for r in report['cases']if r['returncode']];report['hypercurve_passed']=sum(r['returncode']==0 for r in report['cases']);report['qualification_complete']=not report['unresolved_failures'];save()
print('Coverage complete:',report['hypercurve_passed'],'passed;',len(report['unresolved_failures']),'unresolved failures.',flush=True)
