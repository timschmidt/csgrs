from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = 'v185'
assert version.startswith('v') and version[1:].isdigit()
prefix = f'selected-cusp-ranges-isolated-20260926-{version}'
guard_name = 'local-chord-complete-replay-20260924-v184-sources.json'
guard = json.loads((A / guard_name).read_text())
archive = A / 'source-archives' / prefix
build = A / 'build-workspace-20260925'
assert not archive.exists()
assert not (A / f'{prefix}-terminal.json').exists()
parents = {name: subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=W/name, text=True).strip()
           for name in ['hypercurve', 'hypersolve', 'hyperreal']}
assert parents['hypercurve'].startswith('7941ee50')
assert parents['hypersolve'].startswith('41855c3')
changed = {repo: subprocess.check_output(['git','diff','--name-only'], cwd=W/repo, text=True).splitlines() for repo in parents}
for repo in parents:
    assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=W/repo)
for name, sha in guard.items():
    source, destination = W/name, archive/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha, name
    destination.parent.mkdir(parents=True, exist_ok=True)
    repo, local = name.split('/', 1)
    if repo in changed and local in changed[repo]:
        destination.write_bytes(subprocess.check_output(['git','show',parents[repo]+':'+local], cwd=W/repo))
    else:
        shutil.copy2(source, destination)
    assert (source.stat().st_dev, source.stat().st_ino) != (destination.stat().st_dev, destination.stat().st_ino)
patch = A / f'{prefix}.patch'
subprocess.run(['git','apply','--check',str(patch)], cwd=archive/'hypercurve', check=True)
subprocess.run(['git','apply',str(patch)], cwd=archive/'hypercurve', check=True)
manifest = {name: hashlib.sha256((archive/name).read_bytes()).hexdigest() for name in guard}
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest, indent=2)+'\n')
for name in manifest:
    source, destination = archive/name, build/name
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists() or source.read_bytes() != destination.read_bytes():
        shutil.copy2(source, destination)
        os.utime(destination, None)
    assert (source.stat().st_dev, source.stat().st_ino) != (destination.stat().st_dev, destination.stat().st_ino)
env = dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
cargo = str(toolchain/'cargo')
report = dict(parents=parents, workspace_guard=guard_name, source_manifest=f'{prefix}-sources.json',
              source_directory=str(archive), build_source_directory=str(build), patch=patch.name,
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
for repo, files in [('hypercurve',['src/bezier_offset.rs','src/bezier_region.rs'])]:
    run(repo,repo+'-fmt',[str(toolchain/'rustfmt'),'--edition','2024','--check',*files],60,'checks')
    for index, feature in enumerate(['--all-features','--no-default-features']):
        run(repo,repo+f'-clippy-{index}',[cargo,'clippy','--all-targets',feature,'--locked','--offline','--','-D','warnings'],1200,'checks')
run('hyperbrep','hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],1200,'checks')

prior = json.loads((A/'retained-circle-parallel-isolated-20260926-v180-terminal.json').read_text())
jobs = [(row['target'],row['name']) for row in prior['cases'] if 'target' in row]
extra = [
    'selected_parallel_offset_partitions_existing_cusps_before_composition',
    'selected_parallel_coalescing_requires_regular_cells',
    'selected_parallel_endpoint_tangents_use_the_incident_branch',
    'selected_parallel_offset_preserves_stationary_branch_frames',
    'retained_parallel_offset_composition_respects_traversal_orientation',
    'retained_parallel_offset_preserves_algebraic_cusp_partition',
    'retained_parallel_offset_composition_splits_new_cusps_in_traversal_order',
    'retained_parallel_offset_coalesces_non_cusp_algebraic_arrangement_partitions',
    'retained_stationary_endpoint_composition_keeps_source_and_tangent_anchor',
    'resource_blocked_selected_offset_span_completes_its_cold_projection',
    'even_multiplicity_stationary_source_coalesces_one_offset_span',
]
names = compile_tests('hypercurve', sorted({target for target,_ in jobs}))
for suffix in extra:
    matching = [name for name in names['hypercurve'] if name.rsplit('::',1)[-1] == suffix]
    assert len(matching) == 1, suffix
    if ('hypercurve',matching[0]) not in jobs:
        jobs.append(('hypercurve',matching[0]))
analytic = [
    'retained_arc_fillet_preserves_past_center_tangent_orientation',
    'retained_rational_arc_and_analytic_parallel_fillet_exactly',
    'retained_rational_arc_and_analytic_parallel_fillet_extends_exactly',
]
for name in analytic:
    job=('hypercurve_analytic_parallel_region', name)
    if job not in jobs: jobs.append(job)
extra += analytic
for target, listed in names.items():
    for name in listed:
        if target == 'hypercurve_analytic_parallel_region' or (
            name.startswith('bezier_region::tests::') and
            any(word in name for word in ['offset', 'stroke', 'stationary', 'boundary_tangent', 'source_cusp'])
        ):
            if (target,name) not in jobs: jobs.append((target,name))
verify()
jobs.sort(key=lambda job: (job[1].rsplit('::',1)[-1] not in extra, job))
report['selection'] = jobs
print('fresh Hypercurve binaries; selected',len(jobs),'cases',flush=True)
for index,(target,name) in enumerate(jobs):
    assert name in names[target], name
    binary = report['binaries'][target]
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
    row = run('hypercurve',f'case-{index:03}',[binary['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never'],180,'cases')
    row.update(target=target,name=name)
    output = (A/row['log']).read_text()
    assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',output), name
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
