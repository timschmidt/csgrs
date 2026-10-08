from pathlib import Path
import hashlib, json, subprocess

A=Path(__file__).resolve().parent
workspace=Path('/home/tim/Documents/GitHub/workspace')
repo=workspace/'hypercurve'
root=Path('/tmp/hypercurve-recognized-circle-evidence-v5-2026-09-23')
prefix='recognized-circle-evidence-20260923'
bindings=json.loads((A/f'{prefix}-v5-sources.json').read_text())
focused=json.loads((A/f'{prefix}-v5-terminal.json').read_text())
broad=json.loads((A/f'{prefix}-v5-regression-terminal.json').read_text())
assert focused['all_processes_reaped'] and focused['all_sources_unchanged']
assert broad['all_processes_reaped'] and broad['all_sources_unchanged']
assert len(focused['cases'])==6 and all(row['passed'] for row in focused['cases'])
assert not broad['failed']
assert all(check['returncode']==0 and 'warning:' not in (A/check['log']).read_text() for check in focused['checks'])
binary=A/f'{prefix}-v5-libtest'
assert hashlib.sha256(binary.read_bytes()).hexdigest()==focused['binary_sha256']==broad['binary_sha256']
main_count=0
for name,sha in bindings.items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
    if name.startswith('hypercurve/'):
        assert hashlib.sha256((workspace/name).read_bytes()).hexdigest()==sha,name
        main_count+=1
files=['src/bezier_region.rs','src/curve_support_intersection.rs','src/rational_bezier_general.rs']
status=subprocess.check_output(['git','status','--short'],cwd=repo,text=True)
assert status==''.join(' M '+name+'\n' for name in files),status
subprocess.run(['git','diff','--check'],cwd=repo,check=True)
report=dict(parent=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),files={name:bindings['hypercurve/'+name] for name in files},main_files_verified=main_count,snapshot_files_verified=len(bindings),focused_cases=focused['cases'],regression_attempted=broad['attempted'],regression_passed=broad['passed'],regression_ignored=broad['ignored'],existing_nonpasses_not_repeated=[row['name'] for row in broad['not_repeated_existing_nonpasses']],binary_sha256=focused['binary_sha256'],all_owned_processes_reaped=True,full_goal_complete=False)
(A/f'{prefix}-qualification.json').write_text(json.dumps(report,indent=2)+'\n')
print('Qualified',main_count,'Hypercurve files;',broad['passed']+6,'passing cases and',broad['ignored'],'ignored; existing nonpasses remain')
