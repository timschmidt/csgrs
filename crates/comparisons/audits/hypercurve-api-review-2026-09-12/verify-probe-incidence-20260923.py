from pathlib import Path
import hashlib,json,subprocess
p=Path(__file__).resolve().parent;main=p.parent/'hypercurve'
report=json.loads((p/'probe-incidence-20260923-final1-terminal.json').read_text())
assert report['all_processes_reaped'] and report['all_sources_unchanged']
assert not report['new_failures'] and not report['changed_existing_nonpasses'], report
assert report['passed']==1193 and report['ignored']==6 and len(report['failed'])==18
assert all(r['returncode']==0 for r in report['checks'])
assert hashlib.sha256((p/'probe-incidence-20260923-final1-libtest').read_bytes()).hexdigest()==report['binary_sha256']
expected=json.loads((p/'probe-incidence-20260923-final1-candidate.json').read_text())
files={name:sha for name,sha in expected.items() if name!='src/bezier_region.rs'}
for name,sha in files.items():assert hashlib.sha256((main/name).read_bytes()).hexdigest()==sha,name
region=hashlib.sha256((main/'src/bezier_region.rs').read_bytes()).hexdigest()
assert region=='1bc26463270fbe520873708bc6909db19a7af36356367f8bbc366b92696ed720'
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=main,text=True).strip()=='d00fa186b720f4224e512d9185bc859c624b4608'
out=dict(base_head='d00fa186b720f4224e512d9185bc859c624b4608',files=files,preserved_unqualified_region_sha256=region,library_cases=1217,passed=report['passed'],ignored=report['ignored'],existing_nonpasses=18,new_nonpasses=0,changed_existing_nonpasses=0,all_target_checks=report['checks'],qualified_binary_sha256=report['binary_sha256'],normalized_focused_cases=json.loads((p/'retained-fragment-witness-20260923-focused3-terminal.json').read_text()),all_owned_processes_reaped=True,hyperreal_inputs='Pinned immutable snapshots; other-session worktree untouched and unused')
(p/'probe-incidence-20260923-qualification.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
