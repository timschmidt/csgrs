from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'incident-ray-sectors-parent-20260925-{version}'
root = A / 'source-archives' / prefix
assert not root.exists()
guard_name = f'local-chord-complete-replay-20260924-{version}-sources.json'
guard = json.loads((A / guard_name).read_text())
hc = W / 'hypercurve'
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=hc, text=True).strip()
changed = subprocess.check_output(['git', 'diff', '--name-only'], cwd=hc, text=True).splitlines()
assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=hc)
selected = ['src/curve_region_boolean.rs']
assert all(name in changed for name in selected)
manifest = {}
for name, expected in guard.items():
    source = W / name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == expected, name
    destination = root / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    local = name.removeprefix('hypercurve/')
    if name.startswith('hypercurve/') and local in changed:
        parent_bytes = subprocess.check_output(['git', 'show', head + ':' + local], cwd=hc)
        if local == selected[0]:
            current = source.read_text()
            start = '    #[test]\n    fn regularization_orders_all_branches_at_a_pinched_algebraic_corner()'
            end = '    #[test]\n    fn curved_face_windings_preserve_crossings_tangencies_overlaps_and_nested_holes()'
            regression = current[current.index(start):current.index(end)]
            parent = parent_bytes.decode()
            assert parent.count(end) == 1
            parent_bytes = parent.replace(end, regression + end).encode()
        destination.write_bytes(parent_bytes)
    else:
        shutil.copy2(source, destination)
    assert (source.stat().st_dev, source.stat().st_ino) != (destination.stat().st_dev, destination.stat().st_ino)
    manifest[name] = hashlib.sha256(destination.read_bytes()).hexdigest()
(A / f'{prefix}-sources.json').write_text(json.dumps(manifest, indent=2) + '\n')

def verify():
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=hc, text=True).strip() == head
    for name, expected in guard.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == expected, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == manifest[name], name

env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
repo = root / 'hypercurve'
report = dict(diagnostic_only=True, parent_control=True, parent=head, selected_files=selected, restored_to_parent=[name for name in changed if name not in selected],
              workspace_guard=guard_name, source_manifest=f'{prefix}-sources.json', checks=[], binaries={}, cases=[],
              all_processes_reaped=False)

def save():
    verify()
    report['all_sources_unchanged'] = True
    (A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2) + '\n')

def run(label, command, limit):
    log = A / f'{prefix}-{label}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    verify()
    return dict(label=label, command=command, returncode=code, elapsed_seconds=time.monotonic()-start,
                limit_seconds=limit, log=log.name, ignored='1 ignored' in log.read_text())

checks = [('fmt', [str(toolchain / 'rustfmt'), '--edition', '2024', '--check', *selected], 60)]
checks.extend((f'clippy-{index}', [str(toolchain / 'cargo'), 'clippy', '--all-targets', feature,
                                '--locked', '--offline', '--', '-D', 'warnings'], 1200)
              for index, feature in enumerate(['--all-features', '--no-default-features']))
for label, command, limit in []:
    row = run(label, command, limit)
    report['checks'].append(row)
    save()
    print(label, row['returncode'], flush=True)
    if row['returncode'] != 0:
        report['all_processes_reaped'] = True
        save()
        print((A / row['log']).read_text()[-3000:], flush=True)
        raise SystemExit(1)

targets = ['hypercurve']
command = [str(toolchain / 'cargo'), 'test', '--release', '--all-features', '--lib', '--no-run',
           '--message-format=json', '--locked', '--offline']
for target in targets[1:]:
    command.extend(['--test', target])
os.utime(repo / 'src/lib.rs', None)
with (A / f'{prefix}-build.jsonl').open('w') as out, (A / f'{prefix}-build.log').open('w') as err:
    try:
        code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
    except subprocess.TimeoutExpired:
        code = 'timeout'
report['build_returncode'] = code
if code != 0:
    report['all_processes_reaped'] = True
    save()
    print('build failed', code, flush=True)
    raise SystemExit(1)
artifacts = [json.loads(line) for line in (A / f'{prefix}-build.jsonl').read_text().splitlines()]
jobs = []
for target in targets:
    artifact = next(row for row in artifacts if row.get('reason') == 'compiler-artifact'
                    and row['target']['name'] == target and row.get('executable'))
    assert not artifact['fresh']
    binary = A / f'{prefix}-{target}'
    shutil.copy2(artifact['executable'], binary)
    report['binaries'][target] = dict(path=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    names = [line[:-6] for line in subprocess.check_output([str(binary), '--list'], text=True).splitlines()
             if line.endswith(': test')]
    jobs.extend((target, name) for name in names if target != 'hypercurve' or name.startswith('curve_region_boolean::'))
regression = 'regularization_orders_all_branches_at_a_pinched_algebraic_corner'
assert sum(name.endswith('::' + regression) for _, name in jobs) == 1
jobs = [job for job in jobs if job[1].rsplit('::', 1)[-1] in {regression,
        'independent_field_collinear_chord_overlap_enters_all_boolean_topology',
        'source_related_algebraic_chord_contact_enters_split_topology'}]
assert len(jobs) == 3
jobs.sort(key=lambda job: not job[1].endswith('::' + regression))
report['selection'] = jobs
print('fresh binaries; selected cases', len(jobs), flush=True)
for index, (target, name) in enumerate(jobs):
    report['active_case'] = [target, name]
    save()
    binary = report['binaries'][target]
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
    row = run(f'case-{index:03}', [binary['path'], '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], 75)
    row.update(target=target, name=name)
    report['cases'].append(row)
    save()
    if row['returncode'] != 0:
        print(name, row['returncode'], (A / row['log']).read_text()[-2000:], flush=True)
    elif index == 0 or (index + 1) % 20 == 0:
        print('completed', index + 1, 'of', len(jobs), flush=True)
report.pop('active_case', None)
report['all_processes_reaped'] = True
save()
passed = len(report['cases']) == len(jobs) and all(row['returncode'] == 0 for row in report['cases'])
print('terminal:', 'passed' if passed else 'failed', '; every child reaped', flush=True)
raise SystemExit(0 if len(report['cases']) == len(jobs) else 1)
