from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'retained-contact-factor-20260925-{version}'
guard_name = f'local-chord-complete-replay-20260924-{version}-sources.json'
guard = json.loads((A / guard_name).read_text())
archive = A / 'source-archives' / prefix
build = A / 'build-workspace-20260925'
assert not archive.exists()
assert not (A / f'{prefix}-terminal.json').exists()
parents = {name: subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=W/name, text=True).strip()
           for name in ['hypercurve', 'hypersolve']}
assert parents['hypercurve'].startswith('5915c26b')
assert parents['hypersolve'].startswith('a6307b5')
hc_changed = subprocess.check_output(['git', 'diff', '--name-only'], cwd=W/'hypercurve', text=True).splitlines()
for repo in parents:
    assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=W/repo)
for name, sha in guard.items():
    source, destination = W/name, archive/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha, name
    destination.parent.mkdir(parents=True, exist_ok=True)
    local = name.removeprefix('hypercurve/')
    if name.startswith('hypercurve/') and local in hc_changed:
        destination.write_bytes(subprocess.check_output(['git','show',parents['hypercurve']+':'+local], cwd=W/'hypercurve'))
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
    verify()
    report['all_sources_unchanged'] = True
    (A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')

def run(repo, label, command, limit, group):
    verify()
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

for repo, files in [('hypersolve',['src/ordered_field_roots.rs','src/root_sign.rs','src/algebraic_fiber.rs']),
                    ('hypercurve',['src/bezier_offset.rs'])]:
    run(repo,repo+'-fmt',[str(toolchain/'rustfmt'),'--edition','2024','--check',*files],60,'checks')
    for index, feature in enumerate(['--all-features','--no-default-features']):
        run(repo,repo+f'-clippy-{index}',[cargo,'clippy','--all-targets',feature,'--locked','--offline','--','-D','warnings'],1200,'checks')
run('hyperbrep','hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],1200,'checks')

names = compile_tests('hypersolve',['hypersolve'])
assert 'ordered_field_roots::tests::linear_quotient_retains_a_root_in_its_coefficient_field' in names['hypersolve']
binary = report['binaries']['hypersolve']['path']
row = run('hypersolve','hypersolve-full',[binary,'--test-threads=2','--color','never'],600,'cases')
output = (A/row['log']).read_text()
match = re.search(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;.*? (\d+) filtered out;', output)
assert match
passed, failed, ignored, filtered = map(int, match.groups())
assert not failed and not ignored and not filtered and passed == len(names['hypersolve'])
report['hypersolve_passed'] = passed

prior = json.loads((A/'selected-frame-import-20260925-v94-candidate-terminal.json').read_text())
jobs = [(row['target'],row['name']) for row in prior['cases']]
extra = [
    'retained_parameter_import_keeps_original_axes_and_tower',
    'exterior_chord_contact_deflation_preserves_other_contacts',
    'recursive_quadratic_endpoint_roots_keep_the_original_field',
    'certified_circle_chord_endpoint_factor_preserves_roots_and_field',
    'algebraic_chord_source_incidence_deflates_only_the_retained_contact',
    'recursive_polynomial_isolators_keep_the_selected_root_after_endpoint_deflation',
    'recursive_ordered_field_isolation_does_not_guess_polynomial_degree',
    'local_parallel_endpoint_clipping_reuses_a_coefficient_root',
]
names = compile_tests('hypercurve',sorted({target for target,_ in jobs}))
for suffix in extra:
    matching = [name for name in names['hypercurve'] if name.rsplit('::',1)[-1] == suffix]
    assert len(matching) == 1, suffix
    if ('hypercurve',matching[0]) not in jobs:
        jobs.append(('hypercurve',matching[0]))
jobs.sort(key=lambda job: (job[1].rsplit('::',1)[-1] not in extra, job))
report['selection'] = jobs
print('fresh Hypercurve binaries; selected',len(jobs),'cases',flush=True)
for index,(target,name) in enumerate(jobs):
    assert name in names[target], name
    binary = report['binaries'][target]
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
    row = run('hypercurve',f'case-{index:03}',[binary['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never'],75,'cases')
    row.update(target=target,name=name)
    output = (A/row['log']).read_text()
    assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',output), name
    save()
report['hypercurve_passed'] = len(jobs)
for binary in report['binaries'].values():
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
report['all_processes_reaped'] = True
save()
print('Complete:',passed,'Hypersolve and',len(jobs),'Hypercurve cases passed.',flush=True)
