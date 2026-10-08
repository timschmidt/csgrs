from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent
repo=Path('/home/tim/Documents/GitHub/workspace/hypercurve')
root=Path('/tmp/hypercurve-selected-overlap-cache-v3-2026-09-23')
bindings=json.loads((A/'selected-overlap-cache-20260923-v3-sources.json').read_text())
focused=json.loads((A/'selected-overlap-cache-20260923-v2-terminal.json').read_text())
broad=json.loads((A/'selected-overlap-cache-20260923-regression-terminal.json').read_text())
assert focused['all_processes_reaped'] and broad['all_processes_reaped']
assert focused['all_sources_unchanged'] and broad['all_sources_unchanged']
assert len(broad['failed'])==1 and broad['failed'][0]['name'].endswith('::selected_fiber_mapped_cut_inverts_on_analytic_overlap')
final=json.loads((A/'selected-overlap-cache-20260923-v3-terminal.json').read_text())
assert final['all_processes_reaped'] and final['all_sources_unchanged']
assert len(final['cases'])==1 and final['cases'][0]['passed']
assert final['cases'][0]['name']==broad['failed'][0]['name']
old=Path('/tmp/hypercurve-selected-overlap-cache-v2-2026-09-23/hypercurve/src/bezier_offset.rs').read_text()
new=(root/'hypercurve/src/bezier_offset.rs').read_text()
def excluding_test(s):
    a=s.index('    fn selected_fiber_mapped_cut_inverts_on_analytic_overlap()')
    b=s.index('\n    #[test]',a)
    return s[:a]+s[b:]
assert excluding_test(old)==excluding_test(new)
assert all(check['returncode']==0 and 'warning:' not in (A/check['log']).read_text() for check in final['checks'])
assert hashlib.sha256((A/'selected-overlap-cache-20260923-v3-libtest').read_bytes()).hexdigest()==final['binary_sha256']
assert len(focused['cases'])==9 and sum(row['passed'] for row in focused['cases'])==8
assert all(check['returncode']==0 and 'warning:' not in (A/check['log']).read_text() for check in focused['checks'])
main_count=0
for name,sha in bindings.items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
    if name.startswith('hypercurve/'):
        assert hashlib.sha256((repo/name[len('hypercurve/'):]).read_bytes()).hexdigest()==sha,name
        main_count+=1
binary=A/'selected-overlap-cache-20260923-v2-libtest'
assert hashlib.sha256(binary.read_bytes()).hexdigest()==focused['binary_sha256']==broad['binary_sha256']
status=subprocess.check_output(['git','status','--short'],cwd=repo,text=True)
assert status==' M src/bezier_offset.rs\n',status
subprocess.run(['git','diff','--check'],cwd=repo,check=True)
report=dict(parent=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),main_files_verified=main_count,offset_sha256=bindings['hypercurve/src/bezier_offset.rs'],focused_cases=focused['cases'],migrated_fallback_case=final['cases'][0],final_binary_sha256=final['binary_sha256'],unchanged_source_excluding_one_test=True,regression_attempted=broad['attempted'],regression_passed=broad['passed'],regression_ignored=broad['ignored'],existing_nonpasses_not_repeated=[row['name'] for row in broad['not_repeated_existing_nonpasses']],binary_sha256=focused['binary_sha256'],all_owned_processes_reaped=True,full_goal_complete=False)
(A/'selected-overlap-cache-20260923-qualification.json').write_text(json.dumps(report,indent=2)+'\n')
print('Qualified',main_count,'Hypercurve input files;',broad['attempted'],'regression cases, 9 focused cases and migrated fallback; separate known chamfer Predicate remains')
