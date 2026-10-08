from pathlib import Path
import concurrent.futures, hashlib, json, os, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
root = Path(f'/tmp/hypercurve-represented-circle-incidence-{version}-20260924')
focused_prefix = f'represented-circle-incidence-20260924-{version}'
prefix = focused_prefix + '-full'
bindings = json.loads((A / f'{focused_prefix}-sources.json').read_text())
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
verify()
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
focused = json.loads((A/f'{focused_prefix}-terminal.json').read_text())
assert focused['all_processes_reaped'] and focused['all_sources_unchanged']
assert all(case['returncode'] == 0 for case in focused['cases'])
checks = focused['checks']
assert all(check['returncode'] == 0 for check in checks)
targets = ['hypercurve_path_closure', 'hypercurve_curve_region_boolean', 'hypercurve_bezier_arrangement', 'hypercurve_curve_region_boolean_fuzz', 'hypercurve_pcb_boolean_regressions', 'hypercurve_curve', 'hypercurve_curve_intersection', 'hypercurve_curve_point', 'hypercurve_curve_range']
command = [cargo, 'test', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
for target in targets:
    command.extend(['--test', target])
with (A / f'{prefix}-build.jsonl').open('w') as out, (A / f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=root / 'hypercurve', env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0, (A / f'{prefix}-build.log').read_text()[-4000:]
assert 'warning:' not in (A / f'{prefix}-build.log').read_text()
libtest = A/f'{focused_prefix}-libtest'
assert hashlib.sha256(libtest.read_bytes()).hexdigest() == focused['binary_sha256']
artifacts = {'hypercurve': dict(path=str(libtest), sha256=focused['binary_sha256'])}
for line in (A / f'{prefix}-build.jsonl').read_text().splitlines():
    row = json.loads(line)
    if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] in targets:
        target = row['target']['name']
        assert not row['fresh']
        binary = A / f'{prefix}-{target}'
        shutil.copy2(row['executable'], binary)
        artifacts[target] = dict(path=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
assert set(artifacts) == {'hypercurve', *targets}
jobs = []
for target, data in artifacts.items():
    names = [line[:-6] for line in subprocess.check_output([data['path'], '--list'], text=True).splitlines() if line.endswith(': test')]
    if target == 'hypercurve':
        names = [name for name in names if any(word in name for word in ['fillet', 'circle', 'cusp', 'chord', 'retained_point', 'corner']) or name.rsplit('::', 1)[-1] in {row['label'] for row in focused['cases']}]
    jobs.extend((target, name) for name in names)
reused = []
for case in focused['cases']:
    matches = [name for target, name in jobs if target == 'hypercurve' and name.rsplit('::', 1)[-1] == case['label']]
    assert len(matches) == 1, case['label']
    output = (A / case['log']).read_text()
    assert 'running 1 test' in output and '1 passed;' in output
    reused.append(dict(target='hypercurve', name=matches[0], returncode=0, passed=True, ignored=False, elapsed_seconds=case['elapsed_seconds'], limit_seconds=75, log=case['log'], reused_from=f'{focused_prefix}-terminal.json'))
reused_keys = {(row['target'], row['name']) for row in reused}
remaining = [(index, job) for index, job in enumerate(jobs) if job not in reused_keys]
slow_suffixes = {'recursive_selected_radial_projective_chamfer_reenters_corner_kernel', 'nonrepresented_chord_and_selected_circle_complete_the_fillet_kernel', 'extended_fillet_region_classifies_both_sides_of_its_companion'}
isolated = [job for job in remaining if job[1][1].rsplit('::', 1)[-1] in slow_suffixes]
concurrent_jobs = [job for job in remaining if job not in isolated]
(A / f'{prefix}-selection.json').write_text(json.dumps(dict(binaries=artifacts, jobs=jobs, reused_focused=reused, isolated_jobs=isolated, remaining_workers=2), indent=2)+'\n')
print('Affected qualification built:', len(jobs), 'cases,', len(reused), 'already focused,', len(isolated), 'long cases isolated', flush=True)
def run(job):
    index, (target, name) = job
    limit = 180 if name.endswith('::nonrepresented_chord_and_selected_circle_complete_the_fillet_kernel') else 75
    if target != 'hypercurve':
        limit = 180
    log = A / f'{prefix}-case-{index:04d}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run([artifacts[target]['path'], '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], cwd=root / 'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    output = log.read_text()
    assert 'running 1 test' in output
    return dict(target=target, name=name, returncode=code, passed=code == 0 and '1 passed;' in output, ignored=code == 0 and '1 ignored;' in output, elapsed_seconds=time.monotonic()-start, limit_seconds=limit, log=log.name)
rows = reused.copy()
for job in isolated:
    row = run(job)
    rows.append(row)
    (A / f'{prefix}-cases.json').write_text(json.dumps(rows, indent=2)+'\n')
    print('Isolated:', row['name'], row['returncode'], row['elapsed_seconds'], flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    futures = [pool.submit(run, job) for job in concurrent_jobs]
    for future in concurrent.futures.as_completed(futures):
        row = future.result()
        rows.append(row)
        (A / f'{prefix}-cases.json').write_text(json.dumps(rows, indent=2)+'\n')
        if not row['passed'] and not row['ignored']:
            print('Not passed:', row['target'], row['name'], row['returncode'], flush=True)
        if len(rows) % 100 == 0:
            print('Completed', len(rows), 'of', len(jobs), flush=True)
verify()
for artifact in artifacts.values():
    assert hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest() == artifact['sha256']
report = dict(reused_focused=reused, isolated_jobs=isolated, remaining_workers=2, checks=checks, command=command, binaries=artifacts, attempted=len(rows), passed=sum(row['passed'] for row in rows), ignored=sum(row['ignored'] for row in rows), nonpasses=[row for row in rows if not row['passed'] and not row['ignored']], all_sources_unchanged=True, all_processes_reaped=True)
(A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Terminal:', {key: report[key] for key in ['attempted', 'passed', 'ignored', 'nonpasses']}, flush=True)
