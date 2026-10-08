from pathlib import Path
import hashlib, json, subprocess
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root/'hypercurve-api-review-2026-09-12'
prefix = 'native-line-arc-endpoint'
rows = json.loads((audit/(prefix+'-full-results.json')).read_text())
assert len(rows) == 49
assert not any(r['new_failures'] or r['timed_out'] for r in rows)
assert all(r['summaries'] and r['returncode'] in [0,101] for r in rows)
before = json.loads((audit/(prefix+'-source-before-tests.json')).read_text())
after = []
for name in ['hypercurve','hyperbrep']:
    paths = subprocess.check_output(['git','ls-files','-c','-o','--exclude-standard','-z'],cwd=root/name).decode().split('\0')
    for relative in sorted(set(filter(None,paths))):
        path=root/name/relative
        if path.is_file():after.append({'file':str(path.relative_to(root)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
assert before == after, 'sources changed during qualification'
checks = []
for name,command in [
    ('format',['rustfmt','--edition','2024','--config','skip_children=true','--check','src/intersect.rs','src/segment.rs','src/curve_region_boolean.rs','tests/hypercurve_curve_region_promotion.rs']),
    ('whitespace',['git','diff','--check']),
]:
    result=subprocess.run(command,cwd=root/'hypercurve',capture_output=True,text=True)
    checks.append({'check':name,'exit_code':result.returncode,'output':result.stdout+result.stderr})
    assert result.returncode==0
for suffix in ['', '-hyperbrep']:
    assert (audit/(prefix+suffix+'-check.exit')).read_text().strip()=='0'
    assert (audit/(prefix+suffix+'-test-build.exit')).read_text().strip()=='0'
(audit/(prefix+'-source-checks.json')).write_text(json.dumps({'source_files':len(before),'immutable_sources':True,'checks':checks},indent=2)+'\n')
lib=max((root/'hypercurve/target/release/deps').glob('libhypercurve-*.rlib'),key=lambda p:p.stat().st_mtime)
q={
 'status':'qualified, pending commit', 'goal_status':'active',
 'base_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root/'hypercurve',text=True).strip(),
 'changes':[
  'Deflate certified line/circle endpoint contacts, retaining their exact point/parameter identity and deriving the other root without a new radical.',
  'Use the root order already certified by the circle relation; remove two redundant scalar sorts and duplicate point evaluation during finite arc filtering.',
  'Consolidate the duplicate arc-endpoint membership predicates; strict geometric equality handles independently represented contacts.',
  'Exhaust certified finite-domain exclusions before terminal approximation.',
  'Require strict proof from optional shared-image chord-endpoint and contact-box distinctness shortcuts before falling through to the common exact kernel.'
 ],
 'regressions':{
  'new_unit_tests':['line_arc_reentry_preserves_certified_algebraic_endpoints','arc_endpoint_membership_survives_an_unresolved_other_endpoint','line_arc_domain_exclusion_does_not_consume_unneeded_ordering'],
  'repaired_public_test':'line_parabola_fillet_extends_the_regular_incident_cell_exactly',
  'independent_matrix':'native-line-arc-endpoint-matrix-final.json',
  'independent_curve_pair_queries':900,'certified_region_intersection_queries':12,'certified_empty_xors':4,
  'policies':['STRICT','APPROXIMATE_512'],'traversals':['forward','reversed']
 },
 'qualification':{
  'results':prefix+'-full-results.json','targets':len(rows),
  'passed':sum(len(r['passed']) for r in rows),'failed':sum(len(r['failed']) for r in rows),
  'ignored':sum(len(r['ignored']) for r in rows),'new_failures':0,'timeouts':0,
  'hypercurve_passed':sum(len(r['passed']) for r in rows if r['repo']=='hypercurve'),
  'hyperbrep_passed':sum(len(r['passed']) for r in rows if r['repo']=='hyperbrep'),
  'remaining_failures':sorted(n for r in rows for n in r['failed']),
  'all_target_checks':True,'immutable_sources':True,'source_files':len(before),'formatting_and_whitespace':True,
  'previously_unqualified_expensive_cases':8,'full_suite_passing':False,
  'normal_library_sha256':hashlib.sha256(lib.read_bytes()).hexdigest()
 },
 'evidence_audit':prefix+'-evidence-audit.md', 'performance_claim':None,
 'remaining':['Known promotion failures and eight expensive previously unqualified cases.','Full-family common intersection dispatch, general trimming, normalized public region admission and the broader architecture goal.']
}
(audit/(prefix+'-qualification.json')).write_text(json.dumps(q,indent=2)+'\n')
print(json.dumps(q['qualification'],indent=2))
