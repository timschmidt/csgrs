from pathlib import Path
import hashlib, json, re, subprocess

A=Path(__file__).resolve().parent
W=A.parent
root=Path('/tmp/hypercurve-tower-reuse-v6-2026-09-24')
prefix='tower-reuse-20260924-v6'
def read(name): return json.loads((A/name).read_text())
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
bindings=read(f'{prefix}-sources.json')
counts={}
for name,digest in bindings.items():
    assert sha(root/name)==digest,name
    assert sha(W/name)==digest,name
    crate=name.split('/')[0]
    counts[crate]=counts.get(crate,0)+1
hr=read(f'{prefix}-terminal.json')
dep=read(f'{prefix}-dependents-terminal.json')
focus=read(f'{prefix}-hypercurve-terminal.json')
broad=read(f'{prefix}-regression-terminal.json')
for report in [hr,dep,focus,broad]:
    assert report['all_processes_reaped'] and report['all_sources_unchanged']
assert all(row['returncode']==0 for row in hr['checks']+hr['tests']+focus['checks'])
assert all(row['passed'] for row in focus['cases'])
for row in dep['crates']:
    assert row['returncode']==0 and all(check['returncode']==0 for check in row['checks'])
prior=read('biquadratic-basis-20260923-qualification.json')
known={row['name'] if isinstance(row,dict) else row for row in prior['hypercurve_existing_nonpasses_not_repeated']}
known.add(prior['hypercurve_existing_nonpass']['name'])
failed={row['name'] for row in broad['failed']}
assert failed<=known,('new regressions',sorted(failed-known))
assert not broad['not_repeated_existing_nonpasses']
rows=read(f'{prefix}-regression-cases.json')
assert len(rows)==len({row['name'] for row in rows})
focus_names={row['name'] for row in focus['cases']}
assert focus_names.isdisjoint({row['name'] for row in rows})
binary=A/f'{prefix}-hypercurve-libtest'
assert sha(binary)==focus['binary_sha256']==broad['binary_sha256']
names={line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')}
assert names==focus_names|{row['name'] for row in rows}
files=['src/computable/node/bounds.rs','src/computable/node/cache_rescale_tests.rs','src/computable/node/quadratic_tower.rs','src/computable/node/representation.rs','src/real/arithmetic/quadratic_tower_sign.rs']
changed=set(subprocess.check_output(['git','diff','--name-only'],cwd=W/'hyperreal',text=True).splitlines())
assert changed==set(files),changed
subprocess.run(['git','diff','--check'],cwd=W/'hyperreal',check=True)
def passed(log):
    match=re.search(r'test result: ok\. (\d+) passed;', (A/log).read_text())
    assert match,log
    return int(match.group(1))
report=dict(
    parents={crate:subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/crate,text=True).strip() for crate in counts},
    files={name:sha(W/'hyperreal'/name) for name in files},
    main_and_snapshot_files_verified=counts,
    hyperreal_tests_passed=passed(f'{prefix}-all.log'),
    dependent_tests_passed={row['crate']:passed(row['log']) for row in dep['crates']},
    hypercurve_cases_attempted=len(names),
    hypercurve_passed=broad['passed']+len(focus['cases']),
    hypercurve_ignored=broad['ignored'],
    remaining_nonpasses=broad['failed'],
    newly_passing_previous_nonpasses=sorted(known-failed),
    hypercurve_production_and_tests_unchanged=True,
    binaries={'hyperreal':hr['binary_sha256'],'hypercurve':focus['binary_sha256'],**{row['crate']:row['binary_sha256'] for row in dep['crates']}},
    all_owned_processes_reaped=True,
    full_goal_complete=False,
)
(A/'tower-reuse-20260924-qualification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({key:report[key] for key in ['hyperreal_tests_passed','dependent_tests_passed','hypercurve_cases_attempted','hypercurve_passed','hypercurve_ignored','newly_passing_previous_nonpasses']},indent=2))
print('Remaining existing nonpasses:',len(failed))
