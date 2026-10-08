from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent
root=Path('/tmp/hypercurve-direct-endpoints-v3-2026-09-24');prefix='direct-endpoints-20260924-v3'
def read(name):return json.loads((A/name).read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
bindings=read(f'{prefix}-sources.json')
for name,digest in bindings.items():
    assert sha(root/name)==digest,name
    assert sha(W/name)==digest,name
focus=read(f'{prefix}-terminal.json');sweep=read(f'{prefix}-regression-terminal.json');integration=read(f'{prefix}-integrations-terminal.json')
for report in [focus,sweep,integration]:assert report['all_sources_unchanged'] and report['all_processes_reaped']
assert all(row['returncode']==0 for row in focus['checks'])
assert all(row['passed'] for row in focus['cases'])
assert not sweep['failed'],sweep['failed']
assert all(row['passed'] for row in integration['cases'])
v2=read('direct-endpoints-20260924-v2-regression-terminal.json')
parent=read('normalized-straight-corners-20260924-qualification.json')
known={row['name'] for row in parent['remaining_nonpasses']}
assert known=={row['name'] for row in v2['failed']}=={row['name'] for row in sweep['not_repeated_existing_nonpasses']}
cases=read(f'{prefix}-regression-cases.json');assert len(cases)==len({row['name'] for row in cases})
focused={row['name'] for row in focus['cases']};other={row['name'] for row in cases}
assert focused.isdisjoint(other) and focused.isdisjoint(known) and other.isdisjoint(known)
binary=A/f'{prefix}-libtest';assert sha(binary)==focus['binary_sha256']==sweep['binary_sha256']
names={line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')}
assert names==focused|other|known
control=read('direct-endpoints-20260924-admission-control-terminal.json');assert control['all_processes_reaped']
assert [(row['candidate'],row['returncode']) for row in control['runs']]==[('parent',0),('v2',101)]
parent_path=read('direct-endpoints-20260924-parent-path-terminal.json');assert parent_path['all_processes_reaped'] and parent_path['all_sources_unchanged']
assert len(parent_path['cases'])==1 and parent_path['cases'][0]['returncode']=='timeout'
assert len(integration['existing_nonpasses_not_repeated'])==1
assert integration['existing_nonpasses_not_repeated'][0]['name']==parent_path['cases'][0]['name']
files=['src/bezier_offset.rs','src/bezier_region.rs','src/curve_point.rs']
changed=set(subprocess.check_output(['git','diff','--name-only'],cwd=W/'hypercurve',text=True).splitlines());assert changed==set(files),changed
subprocess.run(['git','diff','--check'],cwd=W/'hypercurve',check=True)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check',*files],cwd=W/'hypercurve',check=True)
removed=['RetainedEndpointEvidence','RetainedEndpointEquality','retained_fragment_endpoint_evidence','retained_endpoint_equality']
source=(W/'hypercurve/src/bezier_region.rs').read_text();assert all(name not in source for name in removed)
report=dict(parent=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip(),files={name:sha(W/'hypercurve'/name) for name in files},main_and_snapshot_files_verified=len(bindings),library_inventory=len(names),library_unique_passes=sweep['passed']+len(focused),library_ignored=sweep['ignored'],existing_library_nonpasses_not_repeated=sweep['not_repeated_existing_nonpasses'],public_integrations_passed=len(integration['cases']),existing_public_integration_nonpasses_not_repeated=integration['existing_nonpasses_not_repeated'],binary_sha256=focus['binary_sha256'],integration_binaries=integration['binaries'],admission_counterexample='direct-endpoints-20260924-admission-control-terminal.json',all_owned_processes_reaped=True,full_goal_complete=False)
(A/'direct-endpoints-20260924-qualification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['library_inventory','library_unique_passes','library_ignored','public_integrations_passed','main_and_snapshot_files_verified']},indent=2))
