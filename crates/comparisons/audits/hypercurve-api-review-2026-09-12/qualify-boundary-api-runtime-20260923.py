from pathlib import Path
import concurrent.futures,hashlib,json,os,shutil,subprocess,time

a=Path(__file__).resolve().parent
w=a.parent
r=Path('/tmp/hypercurve-boundary-api-2026-09-23')
parent=Path('/tmp/hypercurve-closure-2026-09-23/hypercurve')
prefix='boundary-api-20260923-runtime1'
binding=json.loads((a/'boundary-api-20260923-check5-sources.json').read_text())
checks=json.loads((a/'boundary-api-20260923-check5-runs.json').read_text())
assert len(checks)==3 and all(x['returncode']==0 for x in checks),checks
changes=json.loads((a/'boundary-api-20260923-changed-files.json').read_text())
targets=[Path(n).stem for n in changes['hypercurve'] if n.startswith('tests/')]
libnames=[
 'bezier_region::tests::boundary_curve_views_share_retained_domains_and_endpoint_evidence',
 'bezier_region::tests::selected_corner_candidates_reenter_normalization_with_retained_contacts',
 'bezier_region::tests::material_component_reentry_shares_single_region_evidence',
 'bezier_region::tests::boundary_path_construction_obeys_selected_terminal_policy',
 'bezier_offset::conversion_tests::mapped_point_inversion_maps_every_structural_overlap_carrier_directly',
]
held_out=['radical_parallel_cusp_offsets_exactly_under_both_policies','retained_rational_arc_and_analytic_parallel_fillet_exactly']
env=dict(os.environ,**json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'

def verify():
 for name,h in binding['working'].items(): assert hashlib.sha256((w/name).read_bytes()).hexdigest()==h,name
 for name,h in binding['isolated'].items(): assert hashlib.sha256((r/name).read_bytes()).hexdigest()==h,name
verify()
builds=[]
def build(repo,root,args,label,fresh):
 cmd=[cargo,'test','--release','--all-features','--no-run','--message-format=json','--locked','--offline',*args]
 start=time.monotonic(); log=a/(prefix+'-'+label+'-build.log'); listing=a/(prefix+'-'+label+'-build.jsonl')
 with listing.open('w') as out,log.open('w') as err:
  code=subprocess.run(cmd,cwd=root,env=env,stdout=out,stderr=err,timeout=1200).returncode
 row=dict(label=label,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name); builds.append(row); (a/(prefix+'-builds.json')).write_text(json.dumps(builds,indent=2)+'\n'); verify(); print(json.dumps(row),flush=True)
 assert code==0,log.read_text()[-6000:]
 binaries={}
 for line in listing.read_text().splitlines():
  x=json.loads(line)
  if x.get('reason')!='compiler-artifact' or not x.get('executable') or not x['profile']['test']: continue
  if fresh: assert not x['fresh'],(label,x['target']['name'])
  name=x['target']['name']; binary=a/(prefix+'-'+label+'-'+name); shutil.copy2(x['executable'],binary); binaries[name]=binary
 return binaries

def names(binary,*flags):
 text=subprocess.check_output([str(binary),*flags,'--list'],text=True)
 return [line.removesuffix(': test') for line in text.splitlines() if line.endswith(': test')]

def run(job):
 label,target,binary,name=job; cmd=[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never']; start=time.monotonic(); log=a/(prefix+'-'+label+'-'+target+'-'+name.replace('::','_')+'.log')
 with log.open('w') as out:
  try: code=subprocess.run(cmd,cwd=r/('hyperbrep' if label=='hyperbrep' else 'hypercurve'),env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
  except subprocess.TimeoutExpired: code='timeout'
 text=log.read_text(); row=dict(label=label,target=target,name=name,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start,passed=code==0 and '1 passed;' in text,ignored=code==0 and '1 ignored;' in text,log=log.name,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
 if not row['passed'] and not row['ignored']: print('Not passed:',label,target,name,code,'\n'+text[-1800:],flush=True)
 return row
args=[]
for target in targets: args+=['--test',target]
# The parent snapshot is the exact dfa3913 core source with pinned dependencies.
parent_binding=json.loads((a/'winding-contact-reuse-20260923-final1-sources.json').read_text())
for x in parent_binding['isolated']: assert hashlib.sha256((parent.parent/x['file']).read_bytes()).hexdigest()==x['sha256'],x['file']
parents=build('hypercurve',parent,args,'parent',False)
candidates=build('hypercurve',r/'hypercurve',['--lib',*args],'candidate',True)
assert set(candidates)==set(targets)|{'hypercurve'},set(candidates)
jobs=[]; exclusions=[]
for target,binary in candidates.items():
 listed=names(binary)
 if target=='hypercurve': assert all(n in listed for n in libnames); listed=libnames
 for name in listed:
  if name in held_out: exclusions.append((target,name)); continue
  jobs.append(('candidate',target,binary,name))
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 for row in pool.map(run,jobs):
  rows.append(row); (a/(prefix+'-candidate-cases.json')).write_text(json.dumps(rows,indent=2)+'\n')
  if len(rows)%100==0: print('Candidate cases completed:',len(rows),'/',len(jobs),flush=True)
verify()
failed=[x for x in rows if not x['passed'] and not x['ignored']]
assert all(x['target']!='hypercurve' for x in failed),failed
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 replays=list(pool.map(run,[('parent',x['target'],parents[x['target']],x['name']) for x in failed]))
new=[x['name'] for x in replays if x['passed']]
report=dict(cases=rows,parent_replays=replays,new_failures=new,excluded=exclusions)
(a/(prefix+'-hypercurve.json')).write_text(json.dumps(report,indent=2)+'\n'); verify()
print('Hypercurve:',sum(x['passed'] for x in rows),'passed;',len(failed),'not passed; new:',new,flush=True)
assert not new,new
breps=build('hyperbrep',r/'hyperbrep',[],'hyperbrep',True)
rows=[]
jobs=[('hyperbrep',target,binary,name) for target,binary in breps.items() for name in names(binary)]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 for row in pool.map(run,jobs):
  rows.append(row); (a/(prefix+'-hyperbrep.json')).write_text(json.dumps(rows,indent=2)+'\n')
verify()
print('HyperBREP:',sum(x['passed'] for x in rows),'passed; not passed:',[x['name'] for x in rows if not x['passed'] and not x['ignored']],flush=True)
(a/(prefix+'-terminal.json')).write_text(json.dumps(dict(builds=builds,sources_unchanged=True,hypercurve_passed=sum(x['passed'] for x in report['cases']),hypercurve_new_failures=new,hyperbrep_passed=sum(x['passed'] for x in rows),hyperbrep_failures=[x['name'] for x in rows if not x['passed'] and not x['ignored']]),indent=2)+'\n')
print('Runtime qualification terminal; all bound sources unchanged.',flush=True)
