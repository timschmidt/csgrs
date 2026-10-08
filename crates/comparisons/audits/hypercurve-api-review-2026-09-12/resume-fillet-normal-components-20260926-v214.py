from pathlib import Path
import hashlib, json, os, re, subprocess, time
A=Path(__file__).resolve().parent
W=A.parent
original='fillet-normal-components-full-20260926-v214'
prefix=original+'-resumed'
report=json.loads((A/f'{original}-terminal.json').read_text())
report.update(resumed_from=f'{original}-terminal.json', cases=[], all_processes_reaped=False)
assert all(row['returncode']==0 for row in report['checks'])
assert report['hypercurve_build_returncode']==0
terminal=A/f'{prefix}-terminal.json'
assert not terminal.exists()
manifest=json.loads((A/report['source_manifest']).read_text())
archive=Path(report['source_directory']);build=Path(report['build_source_directory'])
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
def save():terminal.write_text(json.dumps(report,indent=2)+'\n')
def verify():
 for repo,head in report['parents'].items():
  assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip()==head
  assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=W/repo)
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
 for binary in report['binaries'].values():assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
verify()
names={target:[line[:-6] for line in subprocess.check_output([binary['path'],'--list'],text=True).splitlines() if line.endswith(': test')] for target,binary in report['binaries'].items()}
jobs=set(tuple(row) for row in json.loads((A/'corner-source-chart-full-20260926-v208-terminal.json').read_text())['selection'])
def test_bodies(text):
 matches=list(re.finditer(r'#\[test\]\s+fn\s+(\w+)',text))
 return {m.group(1):text[m.start():matches[i+1].start() if i+1<len(matches) else len(text)] for i,m in enumerate(matches)}
changed_tests=set()
for file in ['src/bezier_offset.rs','src/curve.rs']:
 before=test_bodies(subprocess.check_output(['git','show',report['parents']['hypercurve']+':'+file],cwd=W/'hypercurve',text=True))
 after=test_bodies((archive/'hypercurve'/file).read_text())
 changed_tests.update(name for name,body in after.items() if before.get(name)!=body)
report['changed_test_suffixes']=sorted(changed_tests)
for suffix in report['changed_test_suffixes']:
 matches=[name for name in names['hypercurve'] if name.rsplit('::',1)[-1]==suffix]
 assert matches,suffix
 jobs.update(('hypercurve',name) for name in matches)
jobs.update(('hypercurve',name) for name in names['hypercurve'] if name.startswith('bezier_parameter::conversion_tests::'))
assert all(name in names[target] for target,name in jobs)
known_slow={'curve_region_boolean::certified_successor_tests::selected_parallel_arrangement_splits_interior_cusps','algebraic_endpoint_analytic_parallel_chamfers_replay_selected_distance'}
first_cases={
 'bezier_offset::conversion_tests::original_parallel_normal_constraints_clip_finite_and_incident_components',
 'bezier_offset::conversion_tests::domain_component_normal_constraints_follow_swapped_operands',
 'curve::tests::selected_parallel_fillet_clips_a_positive_dimensional_center_component_locally',
 'curve::tests::selected_parallel_fillet_keeps_contacts_beyond_interior_cusps',
 'curve::tests::fillet_center_contacts_keep_source_orientation_across_support_cusps',
 'curve::tests::distinct_parallel_sources_keep_a_shared_fillet_center_family',
}
jobs=sorted(jobs,key=lambda job:(job[1] not in first_cases,job[1] in known_slow,'one_fragment_nonzero_parallel_loop_extends_chamfer' in job[1],job))
report['selection']=jobs
save();print('Running',len(jobs),'source-bound release cases.',flush=True)
for index,(target,name) in enumerate(jobs):
 label=f'case-{index:03}'
 report['active']=dict(target=target,name=name,index=index);save()
 log=A/f'{prefix}-{label}.log'
 start=time.monotonic()
 with log.open('w') as out:
  try:code=subprocess.run([report['binaries'][target]['path'],'--exact',name,'--test-threads=1','--color','never'],cwd=build/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=300 if 'one_fragment_nonzero_parallel_loop_extends_chamfer' in name else 180).returncode
  except subprocess.TimeoutExpired:code='timeout'
 report['cases'].append(dict(target=target,name=name,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name))
 report.pop('active');save()
 if code:
  print(name,code,log.read_text()[-2500:],flush=True)
  verify();report['all_processes_reaped']=True;save();raise SystemExit(1)
 assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',log.read_text()),name
 if index<6 or (index+1)%25==0:print(index+1,'passed;',name,flush=True)
verify()
report.update(all_processes_reaped=True,all_sources_unchanged=True,qualification_complete=True,hypercurve_passed=len(jobs))
save();print('Complete:',len(jobs),'release cases passed; all child processes reaped.',flush=True)
