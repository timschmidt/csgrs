from pathlib import Path
import hashlib,json,shutil,subprocess
root=Path('/home/tim/Documents/GitHub/workspace');a=root/'hypercurve-api-review-2026-09-12';prefix='finite-region-self-domain'
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
rows=json.loads((a/(prefix+'-full-results.json')).read_text());assert len(rows)==49,len(rows)
baseline=json.loads((a/(prefix+'-baseline-failures.json')).read_text())
known=sorted(n for row in rows for n in row['known_failures'])
assert known==sorted(n for names in baseline.values() for n in names),known
for row in rows:
 assert not row['new_failures'] and not row['timed_out'],row
 assert row['returncode']==(101 if row['known_failures'] else 0),row
 assert row['summaries'] and sha(row['binary'])==row['sha256'],row['target']
manifest=json.loads((a/(prefix+'-source-before-tests.json')).read_text())
changed=[x['file'] for x in manifest if sha(root/x['file'])!=x['sha256']];assert not changed,changed
verification={'files_checked':len(manifest),'changed':changed,'source_manifest':prefix+'-source-before-tests.json','all_sources_unchanged_during_builds_and_tests':True}
(a/(prefix+'-source-verification.json')).write_text(json.dumps(verification,indent=2)+'\n')
formatted={}
for name in ['hypercurve','hyperbrep']:
 files=[n for n in subprocess.check_output(['git','diff','--name-only'],cwd=root/name,text=True).splitlines() if n.endswith('.rs')];formatted[name]=files
 if files:subprocess.run(['rustfmt','--edition','2024','--config','skip_children=true','--check',*files],cwd=root/name,check=True)
 subprocess.run(['git','diff','--check'],cwd=root/name,check=True)
checks={s:int((a/(prefix+s+'.exit')).read_text()) for s in ['-check','-test-build','-hyperbrep-check','-hyperbrep-test-build']};assert all(x==0 for x in checks.values()),checks
artifacts=[json.loads(l) for l in (a/(prefix+'-test-build.jsonl')).read_text().splitlines()]
lib=Path(next(f for x in artifacts if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')));lib_sha=sha(lib)
callers=json.loads((a/(prefix+'-caller-checks.json')).read_text());assert len(callers)==3 and all(x['returncode']==0 for x in callers)
probes=[]
for suffix in ['public-replay','affine-parallel-replay','exterior-completed','tangent-matrix','finite-circle-probe','family-matrix','completed','unit-replay','retained-chord-matrix','finite-matrix','cubic-replay','baseline-recheck','replay-matrix-run','analytic-probe','endpoint-probe','completed-probe','probe-final','circle-matrix','region-carrier-matrix','general-trim-matrix','constant-image-trim-matrix','region-public','region-explicit','bounds-public','winding-public','trim-public','self-public','retrace-public']:
 name=prefix+'-'+suffix;d=json.loads((a/(name+'.json')).read_text())
 assert d['returncode']==0 and d['normal_library_sha256']==lib_sha,d
 assert d['source_sha256']==sha(a/d['source']) and d['executable_sha256']==sha(a/name),name
 if suffix in ['region-public','region-explicit']:assert d['counts']==dict(queries=48,point_replays=96,topologies=48,splits=96,child_queries=192,region_queries=12,boolean_reentries=12)
 probes.append(name+'.json')
focused=json.loads((a/(prefix+'-focused.json')).read_text());assert len(focused)==7
for row in focused:
 assert row['returncode']==0 and sha(row['binary'])==row['sha256'],row
parent=json.loads((a/'finite-region-preparation-qualification.json').read_text())
discovery=json.loads((a/'finite-region-self-domain-ownership-baseline2.json').read_text());assert discovery['returncode']==101
assert discovery['normal_library_sha256']==parent['normal_library_sha256']
assert 'failures=4' in (a/discovery['log']).read_text()
assert discovery['source_sha256']==sha(a/'finite-region-self-domain-public.rs')
assert 'failures=0' in (a/(prefix+'-self-public.log')).read_text()
assert 'valid=false' not in (a/(prefix+'-self-public.log')).read_text()
retrace_discovery=json.loads((a/'finite-region-self-retrace-parent.json').read_text());assert retrace_discovery['returncode']==101
assert retrace_discovery['normal_library_sha256']==parent['normal_library_sha256']
assert retrace_discovery['source_sha256']==sha(a/'finite-region-self-retrace-public.rs')
assert 'failures=12' in (a/retrace_discovery['log']).read_text()
assert 'failures=0' in (a/(prefix+'-retrace-public.log')).read_text()
assert 'queries=24 failures=0' in (a/(prefix+'-trim-public.log')).read_text()
import re
windings = re.findall(r'shift=(-?\d+) x=(-?\d+)/8 policy=CurveContext\((\d)\) location=Ok\(CurveOutcome \{ value: Decided\((\w+)\), certainty: (\w+) \}\)', (a/(prefix+'-winding-public.log')).read_text())
assert len(windings)==42,len(windings)
expected={-5:'Outside',-4:'Boundary',-3:'Inside',-2:'Boundary',-1:'Outside',0:'Outside',1:'Outside'}
assert all(where==expected[int(x)] and certainty=='Certified' for shift,x,policy,where,certainty in windings),windings
archive=a/(prefix+'-libraries');archive.mkdir(exist_ok=True);shutil.copy2(lib,archive/lib.name)
result=dict(status='qualified against the documented baseline; commit pending',goal_status='active',previous_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root/'hypercurve',text=True).strip(),commit=None,targets=len(rows),passed=sum(len(x['passed']) for x in rows),passed_by_repo={name:sum(len(x['passed']) for x in rows if x['repo']==name) for name in ['hypercurve','hyperbrep']},known_failed=known,new_failures=[],ignored=sum(len(x['ignored']) for x in rows),expensive_previously_unqualified_exclusions=8,timeouts=0,builds_and_checks=checks,formatting_and_whitespace='passed on final frozen sources',formatted_files=formatted,additional_caller_checks=callers,source_verification=prefix+'-source-verification.json',focused_regression_checks=prefix+'-focused.json',normal_library=str(lib),normal_library_sha256=lib_sha,archived_library=str(archive/lib.name),independent_probes_and_matrices=probes,discovery_probes=['finite-region-self-domain-baseline.json','finite-region-self-domain-ownership-baseline2.json','finite-region-self-retrace-parent.json'],failed_qualification_attempts=['finite-region-self-domain-attempt2-check.log','finite-region-self-domain-attempt3-regression/archive.json','finite-region-self-domain-attempt5-retrace/archive.json'], prequalification_probe='finite-region-self-domain-attempt1/record.json',evidence_audit=prefix+'-worklog.md',performance_claim=None,remaining='Parallel self-domain ownership, remaining control-hull and analytic-axis premises, analytic point incidence; normalized public region construction; independent circle inverse replay and finite analytic pairs; five existing promotion failures; eight unqualified expensive tests; original algebraic and computational architecture goal.')
(a/(prefix+'-qualification.json')).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ['status','targets','passed','passed_by_repo','known_failed','new_failures','ignored','timeouts','normal_library_sha256']},indent=2))
