from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = 'v191'
prefix = f'corner-source-chart-full-20260926-{version}'
guard_name = f'local-chord-complete-replay-20260924-{version}-sources.json'
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
for repo, files in [('hypercurve',['src/bezier_offset.rs','src/bezier_region.rs','src/curve.rs','src/curve_corner_chain.rs','src/curve_region_boolean.rs'])]:
    run(repo,repo+'-fmt',[str(toolchain/'rustfmt'),'--edition','2024','--check',*files],60,'checks')
    for index,feature in enumerate(['--all-features','--no-default-features']):
        run(repo,repo+f'-clippy-{index}',[cargo,'clippy','--all-targets',feature,'--locked','--offline','--','-D','warnings'],1200,'checks')
run('hyperbrep','hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],1200,'checks')

jobs=set()
for receipt in ['retained-conic-contacts-isolated-20260926-v130-terminal.json',
                'corner-source-chart-followup-20260925-v115-terminal.json',
                'retained-contact-replay-20260926-v126-terminal.json',
                'packed-integer-product-20260926-v138-terminal.json']:
    prior=json.loads((A/receipt).read_text())
    for row in prior['cases']:
        if row.get('repo')=='hypersolve': continue
        target=row.get('target','hypercurve')
        name=row.get('name')
        if name is None: continue
        name=name.replace('selected_affine_tangent_source_retains_incident_endpoint_and_contacts_locally','selected_source_range_retains_incident_endpoint_and_contacts_locally')
        jobs.add((target,name))
fillet = json.loads((A/'fillet-branches-isolated-20260926-v190-resumed-terminal.json').read_text())
jobs.update(tuple(job) for job in fillet['selection'])
jobs.add(('hypercurve_analytic_parallel_region','algebraic_endpoint_analytic_parallel_chamfers_replay_selected_distance'))
changed_tests=set()
def test_bodies(text):
    matches=list(re.finditer(r'#\[test\]\s+fn\s+(\w+)',text))
    return {m.group(1):text[m.start():matches[i+1].start() if i+1<len(matches) else len(text)] for i,m in enumerate(matches)}
for file in ['src/bezier_offset.rs','src/bezier_region.rs','src/curve.rs','src/curve_corner_chain.rs','src/curve_region_boolean.rs']:
    before=test_bodies(subprocess.check_output(['git','show',parents['hypercurve']+':'+file],cwd=W/'hypercurve',text=True))
    after=test_bodies((archive/'hypercurve'/file).read_text())
    changed_tests.update(name for name,body in after.items() if before.get(name)!=body)
names=compile_tests('hypercurve',sorted({target for target,_ in jobs}))
for suffix in changed_tests:
    matches=[name for name in names['hypercurve'] if name.rsplit('::',1)[-1]==suffix]
    assert matches,suffix
    jobs.update(('hypercurve',name) for name in matches)
verify()
known_slow = {
    'curve_region_boolean::certified_successor_tests::selected_parallel_arrangement_splits_interior_cusps',
    'algebraic_endpoint_analytic_parallel_chamfers_replay_selected_distance',
}
jobs=sorted(jobs,key=lambda job:(job[1] in known_slow, 'one_fragment_nonzero_parallel_loop_extends_chamfer' in job[1], job))
report['selection']=jobs
report['changed_test_suffixes']=sorted(changed_tests)
print('fresh Hypercurve binaries; selected',len(jobs),'cases',flush=True)
for index,(target,name) in enumerate(jobs):
    assert name in names[target],name
    binary=report['binaries'][target]
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
    limit=300 if 'one_fragment_nonzero_parallel_loop_extends_chamfer' in name else 180
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
