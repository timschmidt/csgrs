from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = 'v233'
prefix = f'public-fillet-families-full-20260926-{version}'
guard_name = f'{prefix}-sources.json'
guard = json.loads((A/guard_name).read_text())
manifest = guard
archive = A/'source-archives'/prefix
build = A/'build-workspace-20260925'
assert not (A/f'{prefix}-terminal.json').exists()
parents = {name: subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/name,text=True).strip()
           for name in ['hypercurve','hypersolve','hyperreal']}
for repo in parents:
    assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=W/repo)
for name,sha in manifest.items():
    source,destination = archive/name,build/name
    assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
    assert hashlib.sha256(source.read_bytes()).hexdigest()==sha,name
    if not destination.exists() or source.read_bytes()!=destination.read_bytes():
        shutil.copy2(source,destination)
        os.utime(destination,None)
    assert (source.stat().st_dev,source.stat().st_ino)!=(destination.stat().st_dev,destination.stat().st_ino)
env = dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
cargo = str(toolchain/'cargo')
report = dict(parents=parents, workspace_guard=guard_name, source_manifest=guard_name,
              source_directory=str(archive), build_source_directory=str(build),
              checks=[], cases=[], binaries={}, all_processes_reaped=False)

def verify():
    for repo, head in parents.items():
        assert subprocess.check_output(['git','rev-parse','HEAD'], cwd=W/repo, text=True).strip() == head
    for name, sha in guard.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((archive/name).read_bytes()).hexdigest() == manifest[name], name
        assert hashlib.sha256((build/name).read_bytes()).hexdigest() == manifest[name], name

def save():
    report['all_sources_unchanged'] = True
    (A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')

def run(repo, label, command, limit, group):
    report['active'] = label
    save()
    log = A/f'{prefix}-{label}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command, cwd=build/repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    row = dict(repo=repo, label=label, command=command, returncode=code,
               elapsed_seconds=time.monotonic()-start, log=log.name)
    report[group].append(row)
    report.pop('active')
    save()
    print(label, code, log.read_text()[-2200:] if code else '', flush=True)
    if code:
        report['all_processes_reaped'] = True
        verify()
        save()
        raise SystemExit(1)
    return row

def compile_tests(repo, targets):
    os.utime(build/repo/'src/lib.rs', None)
    command = [cargo,'test','--lib']
    for target in targets:
        if target != repo:
            os.utime(build/repo/'tests'/f'{target}.rs', None)
            command += ['--test',target]
    command += ['--release','--all-features','--no-run','--message-format=json','--locked','--offline']
    report['active'] = repo+'-build'
    save()
    with (A/f'{prefix}-{repo}-build.jsonl').open('w') as out, (A/f'{prefix}-{repo}-build.log').open('w') as err:
        code = subprocess.run(command, cwd=build/repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
    report.pop('active')
    report[repo+'_build_returncode'] = code
    if code:
        report['all_processes_reaped'] = True
        verify()
        save()
        print((A/f'{prefix}-{repo}-build.log').read_text()[-3000:], flush=True)
        raise SystemExit(code)
    rows = [json.loads(line) for line in (A/f'{prefix}-{repo}-build.jsonl').read_text().splitlines()]
    names = {}
    for target in targets:
        artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == target and row.get('executable'))
        assert not artifact['fresh'], target
        binary = A/f'{prefix}-{target}'
        shutil.copy2(artifact['executable'], binary)
        report['binaries'][target] = dict(path=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
        names[target] = [line[:-6] for line in subprocess.check_output([str(binary),'--list'], text=True).splitlines() if line.endswith(': test')]
    save()
    return names

verify()
prior_checks=json.loads((A/'retained-fillet-components-full-20260926-v219-terminal.json').read_text())
assert prior_checks['all_processes_reaped'] and prior_checks['qualification_complete']
assert len(prior_checks['checks'])==4 and all(row['returncode']==0 for row in prior_checks['checks'])
qualified_clippy_prefix='public-fillet-families-callers-20260926-v232'
qualified_clippy=json.loads((A/(qualified_clippy_prefix+'-terminal.json')).read_text())
assert qualified_clippy['returncode']==0 and qualified_clippy['all_processes_reaped'] and qualified_clippy['all_sources_unchanged']
assert json.loads((A/(qualified_clippy_prefix+'-sources.json')).read_text())==manifest
changed_files=subprocess.check_output(['git','diff','--name-only'],cwd=W/'hypercurve',text=True).splitlines()+['src/curve_fillet.rs']
for check in prior_checks['checks']:
    if check['label']=='hypercurve-clippy-all':
        report['checks'].append(dict(repo='hypercurve',label='hypercurve-clippy-all',command=qualified_clippy['command'],returncode=0,elapsed_seconds=qualified_clippy['elapsed_seconds'],log=qualified_clippy_prefix+'.log',reused_terminal=qualified_clippy_prefix+'-terminal.json'))
        continue
    command=check['command'].copy()
    if 'rustfmt' in command[0]:
        command=command[:4]+[file for file in changed_files if file.endswith('.rs')]
    run(check['repo'],check['label'],command,900,'checks')
verify()

jobs=set(tuple(row) for row in prior_checks['selection'])
changed_tests=set()
def test_bodies(text):
    matches=list(re.finditer(r'#\[test\]\s+fn\s+(\w+)',text))
    return {m.group(1):text[m.start():matches[i+1].start() if i+1<len(matches) else len(text)] for i,m in enumerate(matches)}
changes=[]
for file in changed_files:
    if not file.endswith('.rs'):continue
    target=Path(file).stem if file.startswith('tests/') else 'hypercurve'
    if file.startswith('benches/'):continue
    old=subprocess.run(['git','show',parents['hypercurve']+':'+file],cwd=W/'hypercurve',text=True,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
    before=test_bodies(old.stdout if old.returncode==0 else '')
    after=test_bodies((archive/'hypercurve'/file).read_text())
    changed_tests.update((target,name) for name,body in after.items() if before.get(name)!=body)
    changes.append((target,after))
targets=sorted({target for target,_ in jobs}|{target for target,_ in changes})
names=compile_tests('hypercurve',targets)
for target,suffix in changed_tests:
    matches=[name for name in names[target] if name.rsplit('::',1)[-1]==suffix]
    assert matches,(target,suffix)
    jobs.update((target,name) for name in matches)
# A shared test assertion changed even when individual fillet bodies did not.
for target,bodies in changes:
    for suffix,body in bodies.items():
        if 'fillet_' not in body:continue
        jobs.update((target,name) for name in names[target] if name.rsplit('::',1)[-1]==suffix)
jobs.update(('hypercurve',name) for name in names['hypercurve'] if name.startswith('bezier_parameter::conversion_tests::'))
jobs.update((row.get('target','hypercurve'),row['name']) for row in prior_checks['cases'])
verify()
known_slow = {
    'curve_region_boolean::certified_successor_tests::selected_parallel_arrangement_splits_interior_cusps',
    'algebraic_endpoint_analytic_parallel_chamfers_replay_selected_distance',
}
first_cases={
    'bezier_offset::conversion_tests::parameter_component_domains_own_selected_events_once',
    'bezier_offset::parameter_component::tests::incident_charts_keep_poles_and_finite_ownership_excluded',
    'bezier_offset::parameter_component::tests::product_components_keep_fixed_axes_and_operand_roles',
    'bezier_offset::conversion_tests::complete_component_queries_keep_split_cells_and_residual_contacts',
    'bezier_offset::conversion_tests::axis_components_keep_exact_clipped_ends_and_open_selector_events',
    'bezier_offset::conversion_tests::original_parallel_normal_constraints_clip_finite_and_incident_components',
    'bezier_offset::conversion_tests::domain_component_normal_constraints_follow_swapped_operands',
    'curve::tests::selected_parallel_fillet_clips_a_positive_dimensional_center_component_locally',
    'curve::tests::selected_parallel_fillet_keeps_contacts_beyond_interior_cusps',
    'curve::tests::fillet_center_contacts_keep_source_orientation_across_support_cusps',
    'curve::tests::distinct_parallel_sources_keep_a_shared_fillet_center_family',
    'curve::curve_fillet::tests::joined_path_selects_and_replays_a_continuous_fillet_family',
    'curve::curve_fillet::tests::normalized_region_selects_and_reuses_a_continuous_fillet_family',
}
jobs=sorted(jobs,key=lambda job:(job[1] not in first_cases, job[1] in known_slow, 'one_fragment_nonzero_parallel_loop_extends_chamfer' in job[1], job))
report['selection']=jobs
report['changed_test_suffixes']=sorted(changed_tests)
assert len(jobs) >= 560, len(jobs)
assert all(name in names[target] for target,name in jobs)
print('fresh Hypercurve binaries; selected',len(jobs),'cases',flush=True)
for index,(target,name) in enumerate(jobs):
    assert name in names[target],name
    binary=report['binaries'][target]
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
    limit=300 if 'one_fragment_nonzero_parallel_loop_extends_chamfer' in name or 'curve_fillet::tests::' in name else 180
    row=run('hypercurve',f'case-{index:03}',[binary['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never'],limit,'cases')
    row.update(target=target,name=name)
    assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',(A/row['log']).read_text()),name
    save()
report['hypercurve_passed'] = len(jobs)
for binary in report['binaries'].values():
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
verify()
report['all_processes_reaped'] = True
save()
report['qualification_complete'] = True
save()
print('Complete:',len(jobs),'Hypercurve cases passed.',flush=True)
