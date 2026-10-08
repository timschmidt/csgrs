from pathlib import Path
import hashlib, json, subprocess
root=Path('/home/tim/Documents/GitHub/workspace')
audit=root/'hypercurve-api-review-2026-09-12'
prefix='finite-circle-rational-domains'
rows=json.loads((audit/(prefix+'-full-results.json')).read_text())
assert len(rows)==49, len(rows)
new=[name for row in rows for name in row['new_failures']]
timeouts=[row['target'] for row in rows if row['timed_out']]
assert not new, new
assert not timeouts, timeouts
known=sorted(name for row in rows for name in row['known_failures'])
baseline=json.loads((audit/(prefix+'-baseline-failures.json')).read_text())
assert known==sorted(name for names in baseline.values() for name in names),known
for row in rows:
    assert row['returncode']==(101 if row['known_failures'] else 0),row['target']
    assert row['summaries'],row['target']
    assert hashlib.sha256(Path(row['binary']).read_bytes()).hexdigest()==row['sha256'],row['target']
before=json.loads((audit/(prefix+'-source-before-tests.json')).read_text())
changed=[row['file'] for row in before if hashlib.sha256((root/row['file']).read_bytes()).hexdigest()!=row['sha256']]
assert not changed,changed
verification={'files_checked':len(before),'changed':changed,'source_manifest':prefix+'-source-before-tests.json','all_sources_unchanged_during_builds_and_tests':True}
(audit/(prefix+'-source-verification.json')).write_text(json.dumps(verification,indent=2)+'\n')
formatted = {}
for repo in ['hypercurve', 'hyperbrep']:
    files = [name for name in subprocess.check_output(['git','diff','--name-only'],cwd=root/repo,text=True).splitlines() if name.endswith('.rs')]
    formatted[repo] = files
    if files:
        subprocess.run(['rustfmt','--edition','2024','--config','skip_children=true','--check',*files],cwd=root/repo,check=True)
    subprocess.run(['git','diff','--check'],cwd=root/repo,check=True)
checks={suffix:int((audit/(prefix+suffix+'.exit')).read_text()) for suffix in ['-check','-test-build','-hyperbrep-check','-hyperbrep-test-build']}
assert all(code==0 for code in checks.values()),checks
lib=root/'hypercurve/target/release/deps/libhypercurve-2c7ff6f3fbe07653.rlib'
sha=hashlib.sha256(lib.read_bytes()).hexdigest()
caller_checks=json.loads((audit/(prefix+'-caller-checks.json')).read_text())
assert all(row['returncode']==0 for row in caller_checks),caller_checks
probes=[]
for name in ['public-replay','affine-parallel-replay','exterior-completed','tangent-matrix','finite-circle-probe','family-matrix','completed','unit-replay','retained-chord-matrix','finite-matrix','cubic-replay','baseline-recheck','replay-matrix-run','analytic-probe','endpoint-probe','completed-probe','probe-final','circle-matrix','region-carrier-matrix','general-trim-matrix','constant-image-trim-matrix','curves-run']:
    path=prefix+'-'+name+'.json'
    data=json.loads((audit/path).read_text())
    assert data['returncode']==0,data
    assert data['normal_library_sha256']==sha,data
    assert data['source_sha256']==hashlib.sha256((audit/data['source']).read_bytes()).hexdigest()
    assert data['executable_sha256']==hashlib.sha256((audit/(prefix+'-'+name)).read_bytes()).hexdigest()
    if name == 'curves-run':
        assert data['counts'] == {'queries':48,'point_replays':96,'topologies':48,'splits':96,'child_queries':192}
    probes.append(path)
focused=json.loads((audit/(prefix+'-final-focused.json')).read_text())
for row in focused:
    assert row['returncode']==0,row
    assert hashlib.sha256(Path(row['binary']).read_bytes()).hexdigest()==row['sha256'],row
region=json.loads((audit/(prefix+'-region-recheck.json')).read_text())
control=json.loads((audit/(prefix+'-control-boolean.json')).read_text())
assert region['returncode']==control['returncode']==101
assert region['normal_library_sha256']==sha
assert region['source_sha256']==control['source_sha256']==hashlib.sha256((audit/region['source']).read_bytes()).hexdigest()
assert hashlib.sha256((audit/(prefix+'-region-recheck')).read_bytes()).hexdigest()==region['executable_sha256']
assert hashlib.sha256(Path(control['archived_library']).read_bytes()).hexdigest()==control['normal_library_sha256']
for discovery in [region,control]:
    assert 'operation: Boolean, family: Line, reason: Unsupported' in (audit/discovery['log']).read_text()
result={
 'status':'qualified against the documented baseline; commit pending',
 'goal_status':'active',
 'previous_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root/'hypercurve',text=True).strip(),
 'commit':None,
 'targets':len(rows),
 'reused_identical_executables':sum(bool(row.get('reused_identical_executable')) for row in rows),
 'passed':sum(len(row['passed']) for row in rows),
 'passed_by_repo':{name:sum(len(row['passed']) for row in rows if row['repo']==name) for name in ['hypercurve','hyperbrep']},
 'known_failed':known,
 'new_failures':new,
 'ignored':sum(len(row['ignored']) for row in rows),
 'expensive_previously_unqualified_exclusions':8,
 'timeouts':len(timeouts),
 'builds_and_checks':checks,
 'formatting_and_whitespace':'passed on final frozen qualification sources',
 'formatted_files':formatted,
 'additional_caller_checks':caller_checks,
 'source_verification':prefix+'-source-verification.json',
 'focused_regression_checks':prefix+'-final-focused.json',
 'discovery_probe':'finite-circle-nonlinear-domain-baseline.json',
 'failed_qualification_attempt':'finite-circle-rational-domains-attempt7-failure/finite-circle-rational-domains-full-results.json',
 'repair_focused_checks':'finite-circle-domain-attempt11-focused.json',
 'additional_preexisting_region_failure':{
     'control':'finite-circle-rational-domains-control-boolean.json',
     'initial_discovery':'finite-circle-nonlinear-domain-attempt6-public-matrix.json',
     'current_discovery':'finite-circle-rational-domains-region-recheck.json',
     'description':'The same unit-chart region intersection returns Blocked(Boolean, Line, Unsupported) on parent 5c47fb1, attempt 6, and the final frozen sources; this precedes the explicit Boolean call and is separate from the five suite failures.'
 },
 'normal_library_sha256':sha,
 'independent_probes_and_matrices':probes,
 'regression_baseline':'circle-closed-clipping-qualification.json',
 'affected_regression_results':prefix+'-full-hypercurve-lib.json',
 'evidence_audit':prefix+'-evidence-audit.md',
 'performance_claim':None,
 'remaining':'Cold finite circle inverse replay between independent mapped-point/tangent authorities; circle/parallel, parallel/rational and parallel/parallel finite discovery; the separately reproduced region intersection failure; normalized public region construction; five promotion failures; eight unqualified expensive tests; broader algebraic consolidation, computational measurements, and original architecture goal.'
}
(audit/(prefix+'-qualification.json')).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
