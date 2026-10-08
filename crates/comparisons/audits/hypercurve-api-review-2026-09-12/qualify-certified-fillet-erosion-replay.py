from pathlib import Path
import subprocess,json,hashlib,re
root=Path('/home/tim/Documents/GitHub/workspace')
audit=root/'hypercurve-api-review-2026-09-12'
prefix='certified-fillet-erosion-replay'
base=json.loads((audit/'native-line-arc-endpoint-qualification.json').read_text())
log=(audit/(prefix+'-promotion-tests.log')).read_text()
statuses=re.findall(r'^test ([^\n]+?) \.\.\. (ok|FAILED|ignored[^\n]*)$',log,re.M)
passed=[n for n,s in statuses if s=='ok']
failed=[n for n,s in statuses if s=='FAILED']
new=sorted(set(failed)-set(base['qualification']['remaining_failures']))
assert not new and len(passed)==96 and len(failed)==5
assert 'line_parabola_fillet_extends_the_regular_incident_cell_exactly' in passed
assert 'unified_region_non_miter_erosions_split_after_neck_collapse' in passed
assert (audit/(prefix+'-check.exit')).read_text().strip()=='0'
assert (audit/(prefix+'-promotion-tests.exit')).read_text().strip()=='101'
before=json.loads((audit/(prefix+'-source-before-tests.json')).read_text())
after=[]
for repo in ['hypercurve','hyperbrep']:
    paths=subprocess.check_output(['git','ls-files','-c','-o','--exclude-standard','-z'],cwd=root/repo).decode().split('\0')
    for relative in sorted(set(filter(None,paths))):
        path=root/repo/relative
        if path.is_file():after.append({'file':str(path.relative_to(root)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
assert before==after
prior=json.loads((audit/'native-line-arc-endpoint-source-before-tests.json').read_text())
changed=[r['file'] for r,s in zip(prior,after) if r!=s]
assert len(prior)==len(after) and changed==['hypercurve/tests/hypercurve_curve_region_promotion.rs']
checks=[]
for name,command in [('format',['rustfmt','--edition','2024','--config','skip_children=true','--check','tests/hypercurve_curve_region_promotion.rs']),('whitespace',['git','diff','--check'])]:
    result=subprocess.run(command,cwd=root/'hypercurve',capture_output=True,text=True)
    checks.append({'check':name,'exit_code':result.returncode,'output':result.stdout+result.stderr})
    assert result.returncode==0
lib=max((root/'hypercurve/target/release/deps').glob('libhypercurve-*.rlib'),key=lambda p:p.stat().st_mtime)
lib_hash=hashlib.sha256(lib.read_bytes()).hexdigest()
assert lib_hash==base['qualification']['normal_library_sha256']
q={
 'status':'qualified, pending commit','goal_status':'active','base_commit':base['commit'],
 'changes':['Require certified fixture reversal, region construction and both represented/selected fillet solutions before composition.','Require certified inside/outside classification after bevel, round and limited-miter neck collapse under both policies; remove the obsolete mandatory-approximation expectation.'],
 'production_sources_unchanged':True,'normal_library_sha256':lib_hash,
 'independent_probe':'neck-erosion-certified-locations-probe.json','independent_offset_queries':6,'independent_location_queries':18,
 'qualification':{'target':'hypercurve_curve_region_promotion','passed':len(passed),'failed':len(failed),'remaining_failures':failed,'new_failures':new,'all_target_check':True,'immutable_sources':True,'source_files':len(after),'formatting_and_whitespace':True,'checks':checks,'unchanged_production_qualification':'native-line-arc-endpoint-qualification.json','updated_total_passed':base['qualification']['passed']+len(passed)-95,'full_suite_passing':False,'previously_unqualified_expensive_cases':8},
 'remaining':['Five promotion failures and eight expensive previously unqualified cases.','General trimming, full-family common dispatch and the broader original architecture goal.']
}
(audit/(prefix+'-qualification.json')).write_text(json.dumps(q,indent=2)+'\n')
print(json.dumps(q['qualification'],indent=2))
