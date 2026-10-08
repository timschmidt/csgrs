from pathlib import Path
import hashlib, json, subprocess
root=Path('/home/tim/Documents/GitHub/workspace')
audit=root/'hypercurve-api-review-2026-09-12'
prefix='selected-circle-local-frame'
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
before=json.loads((audit/(prefix+'-source-before-tests.json')).read_text())
changed=[row['file'] for row in before if hashlib.sha256((root/row['file']).read_bytes()).hexdigest()!=row['sha256']]
assert not changed,changed
verification={'files_checked':len(before),'changed':changed,'source_manifest':prefix+'-source-before-tests.json','all_sources_unchanged_during_builds_and_tests':True}
(audit/(prefix+'-source-verification.json')).write_text(json.dumps(verification,indent=2)+'\n')
files=['src/bezier_offset.rs','src/bezier_split.rs','src/curve_intersection.rs','src/curve_region_boolean.rs','src/curve_support_intersection.rs','tests/hypercurve_curve_intersection.rs']
for cmd in [['rustfmt','--edition','2024','--config','skip_children=true','--check',*files],['git','diff','--check']]:
    subprocess.run(cmd,cwd=root/'hypercurve',check=True)
checks={suffix:int((audit/(prefix+suffix+'.exit')).read_text()) for suffix in ['-check','-test-build','-hyperbrep-check','-hyperbrep-test-build']}
assert all(code==0 for code in checks.values()),checks
lib=root/'hypercurve/target/release/deps/libhypercurve-2c7ff6f3fbe07653.rlib'
sha=hashlib.sha256(lib.read_bytes()).hexdigest()
matrices=[prefix+'-probe-final.json',prefix+'-matrix.json',prefix+'-general-trim-matrix.json',prefix+'-constant-image-trim-matrix.json']
for name in matrices:
    matrix=json.loads((audit/name).read_text())
    assert matrix['returncode']==0,name
    assert matrix['normal_library_sha256']==sha,name
result={
 'status':'qualified against the documented baseline; commit pending',
 'goal_status':'active',
 'previous_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root/'hypercurve',text=True).strip(),
 'commit':None,
 'targets':len(rows),
 'passed':sum(len(row['passed']) for row in rows),
 'passed_by_repo':{name:sum(len(row['passed']) for row in rows if row['repo']==name) for name in ['hypercurve','hyperbrep']},
 'known_failed':known,
 'new_failures':new,
 'ignored':sum(len(row['ignored']) for row in rows),
 'expensive_previously_unqualified_exclusions':8,
 'timeouts':len(timeouts),
 'builds_and_checks':checks,
 'formatting_and_whitespace':'passed on unchanged qualification sources',
 'source_verification':prefix+'-source-verification.json',
 'normal_library_sha256':sha,
 'independent_probes_and_matrices':matrices,
 'regression_baseline':prefix+'-baseline.json',
 'regression_fixed':prefix+'-fixed.json',
 'evidence_audit':prefix+'-evidence-audit.md',
 'performance_claim':None,
 'remaining':'Parallel/rational, parallel/chord and parallel/parallel common dispatch; publicly replayable region carrier charts; finite exterior and retraced domains; normalized public region construction; five promotion failures; eight unqualified expensive cases; broader original architecture goal.'
}
(audit/(prefix+'-qualification.json')).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
