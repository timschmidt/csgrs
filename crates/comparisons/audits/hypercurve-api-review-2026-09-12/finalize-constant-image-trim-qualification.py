from pathlib import Path
import hashlib,json
root=Path('/home/tim/Documents/GitHub/workspace')
p=root/'hypercurve-api-review-2026-09-12'
prefix='constant-image-trim'
read=lambda name: json.loads((p/name).read_text())
rows=read(prefix+'-full-results.json')
assert len(rows)==49,len(rows)
assert not any(r['new_failures'] or r['timed_out'] for r in rows)
assert all(r['summaries'] for r in rows)
assert all(r['returncode'] in (0,101) for r in rows)
assert sum(len(r['failed']) for r in rows)==5
assert sum(len(r['ignored']) for r in rows)==9
focused=read(prefix+'-focused-results.json')
assert all(r['returncode']==0 for r in focused)
manifest=read(prefix+'-source-before-tests.json')
changed=[r['file'] for r in manifest if hashlib.sha256((root/r['file']).read_bytes()).hexdigest()!=r['sha256']]
assert not changed,changed
binary_changes=[r['binary'] for r in rows if hashlib.sha256(Path(r['binary']).read_bytes()).hexdigest()!=r['sha256']]
assert not binary_changes,binary_changes
verification={'source_files':len(manifest),'changed_sources':changed,'changed_test_binaries':binary_changes,'passed':True}
(p/(prefix+'-source-verification.json')).write_text(json.dumps(verification,indent=2)+'\n')
checks={kind:int((p/(prefix+kind+'.exit')).read_text()) for kind in ['-check','-test-build','-hyperbrep-check','-hyperbrep-test-build']}
assert all(v==0 for v in checks.values())
matrix=read(prefix+'-matrix.json')
regression=read(prefix+'-regression-matrix.json')
probe=read(prefix+'-degenerate-probe.json')
assert matrix['returncode']==regression['returncode']==probe['returncode']==0
assert matrix['sha256']==regression['sha256']==probe['sha256']
assert hashlib.sha256(Path(matrix['library']).read_bytes()).hexdigest()==matrix['sha256']
summary={
    'status':'qualified against the documented baseline; ready for commit',
    'goal_status':'active',
    'previous_commit':'678414f0c2e2f18987f5fe682c3fafa4cdd27930',
    'commit':None,
    'targets':len(rows),
    'passed':sum(len(r['passed']) for r in rows),
    'passed_by_repo':{repo:sum(len(r['passed']) for r in rows if r['repo']==repo) for repo in ['hypercurve','hyperbrep']},
    'known_failed':[t for r in rows for t in r['known_failures']],
    'new_failures':[],
    'ignored':sum(len(r['ignored']) for r in rows),
    'expensive_previously_unqualified_exclusions':8,
    'timeouts':0,
    'focused_passed':sum(len(r['passed']) for r in focused),
    'builds_and_checks':checks,
    'formatting_and_whitespace':'passed on unchanged qualification sources',
    'source_verification':prefix+'-source-verification.json',
    'normal_library_sha256':matrix['sha256'],
    'independent_matrix':prefix+'-matrix.json',
    'independent_regression_matrix':prefix+'-regression-matrix.json',
    'degeneracy_probe':prefix+'-degenerate-probe.json',
    'initial_fixture_failures':prefix+'-initial-tests.log',
    'evidence_audit':prefix+'-evidence-audit.md',
    'performance_claim':None,
    'remaining':'Selected-circle/parallel common curve-pair dispatch, five promotion failures, eight expensive unqualified cases, normalized public-region construction and broader architecture work remain open.',
}
(p/(prefix+'-qualification.json')).write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
