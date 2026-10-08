from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prior_prefix='public-fillet-families-full-20260926-v234'
prefix='public-fillet-families-full-20260926-v236'
prior=json.loads((A/f'{prior_prefix}-terminal.json').read_text())
assert prior['all_processes_reaped'] and not prior.get('qualification_complete')
old_manifest=json.loads((A/prior['source_manifest']).read_text())
manifest={name:hashlib.sha256((W/name).read_bytes()).hexdigest()for name in old_manifest}
changed=[name for name in manifest if manifest[name]!=old_manifest[name]]
assert changed==['hypercurve/tests/hypercurve_curve_region_promotion.rs'],changed
old=(Path(prior['source_directory'])/changed[0]).read_text();new=(W/changed[0]).read_text()
name='assert_analytic_parallel_support_corners_retain_algebraic_fillet_centers_and_extensions'
def split_test(text):
 start=text.index('fn '+name+'(');end=text.index('\n#[test]',start+1)
 return text[:start],text[start:end],text[end:]
a,b,c=split_test(old);x,y,z=split_test(new);assert a==x and c==z and b!=y
archive=A/'source-archives'/prefix;build=Path(prior['build_source_directory']);assert not archive.exists()
for name in manifest:
 src=W/name;dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
report=dict(parents=prior['parents'],source_manifest=f'{prefix}-sources.json',source_directory=str(archive),build_source_directory=str(build),checks=[],cases=[],selection=prior['selection'],binaries=prior['binaries'].copy(),all_sources_unchanged=True,all_processes_reaped=False,reused_from=f'{prior_prefix}-terminal.json',prior_outer_session_id=3601,prior_outer_exit_code=1,baseline_audit='fillet-normalized-loop-baseline-20260926-v235-terminal.json',baseline_outer_session_id=4356,baseline_outer_exit_code=0,test_only_change=changed,reused_case_reason='Only the named integration test helper changed; every production and dependency input is byte-identical. All cases from that integration target are rerun.')
for binary in report['binaries'].values():binary.setdefault('source_manifest',prior['source_manifest'])
def verify():
 for repo,head in report['parents'].items():assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip()==head
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for binary in report['binaries'].values():assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
def save(): (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(repo,label,cmd,limit,group,extra=None):
 report['active']=label;save();start=time.monotonic();log=A/f'{prefix}-{label}.log'
 with log.open('w')as out:
  try:code=subprocess.run(cmd,cwd=build/repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=limit).returncode
  except subprocess.TimeoutExpired:code=124
 row=dict(repo=repo,label=label,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name,**(extra or {}));report[group].append(row);report.pop('active');save();print(label,code,flush=True)
 if code:
  verify();report['all_processes_reaped']=True;save();print(log.read_text()[-5000:],flush=True);raise SystemExit(1)
 return row
verify();save()
for check in prior['checks']:
 if check['repo']=='hyperbrep':
  report['checks'].append(dict(check,reused_terminal=report['reused_from']));continue
 run(check['repo'],check['label'],check['command'],900,'checks')
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
target='hypercurve_curve_region_promotion'
cmd=[cargo,'test','--test',target,'--release','--all-features','--no-run','--message-format=json','--locked','--offline']
report['builds']=[];row=run('hypercurve','hypercurve-build',cmd,900,'builds');report['hypercurve_build_returncode']=0
rows=[]
for line in (A/row['log']).read_text().splitlines():
 try:rows.append(json.loads(line))
 except ValueError:pass
artifact=next(r for r in rows if r.get('reason')=='compiler-artifact'and r['target']['name']==target and r.get('executable'));assert not artifact['fresh']
binary=A/f'{prefix}-{target}';shutil.copy2(artifact['executable'],binary)
report['binaries'][target]=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),source_manifest=report['source_manifest'])
for row in prior['cases']:
 if row['returncode']==0 and row.get('target')!=target:
  assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;', (A/row['log']).read_text())
  report['cases'].append(dict(row,reused_terminal=report['reused_from']))
complete={(row['target'],row['name'])for row in report['cases']}
remaining=[tuple(job)for job in report['selection']if tuple(job)not in complete]
remaining.sort(key=lambda job:'analytic_parallel_support_corners_retain_algebraic_fillet_' not in job[1])
for index,(target,name)in enumerate(remaining):
 binary=report['binaries'][target];limit=300 if 'one_fragment_nonzero_parallel_loop_extends_chamfer'in name else 180
 row=run('hypercurve',f'case-{index:03}',[binary['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never'],limit,'cases',dict(target=target,name=name))
 assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;', (A/row['log']).read_text()),name
 save()
verify();assert len(report['cases'])==len(report['selection']);report['hypercurve_passed']=len(report['cases']);report['all_processes_reaped']=True;report['qualification_complete']=True;save()
print('Complete:',len(report['cases']),'release cases;',len(remaining),'executed,',len(complete),'reused.',flush=True)
