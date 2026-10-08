from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'fillet-normal-components-20260926-v212'
archive = A / 'source-archives' / prefix
build = A / 'build-workspace-20260925'
workspace_name = f'{prefix}-sources.json'
workspace = json.loads((A / workspace_name).read_text())
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
parents = {name: subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=W/name, text=True).strip()
           for name in ['hypercurve', 'hypersolve', 'hyperreal']}
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
env = dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
report = dict(parents=parents, workspace_guard=workspace_name,
              source_manifest=f'{prefix}-sources.json', source_directory=str(archive),
              build_source_directory=str(build), cases=[], all_processes_reaped=False)
terminal = A / f'{prefix}-terminal.json'
assert not terminal.exists()

def save():
    terminal.write_text(json.dumps(report, indent=2)+'\n')

def verify():
    for name, head in parents.items():
        assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=W/name, text=True).strip() == head
        assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=W/name)
    for name, sha in manifest.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == workspace[name], name
        assert hashlib.sha256((archive/name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((build/name).read_bytes()).hexdigest() == sha, name

for name, sha in manifest.items():
    source, destination = archive/name, build/name
    if not destination.exists() or source.read_bytes() != destination.read_bytes():
        shutil.copy2(source, destination)
        os.utime(destination, None)
    assert (source.stat().st_dev, source.stat().st_ino) != (destination.stat().st_dev, destination.stat().st_ino)
verify()
report['checks'] = []
checks = [
    [str(toolchain/'rustfmt'), '--edition', '2024', '--check', 'src/bezier_offset.rs', 'src/curve.rs'],
    [str(toolchain/'cargo'), 'clippy', '--all-targets', '--all-features', '--locked', '--offline', '--', '-D', 'warnings'],
    [str(toolchain/'cargo'), 'clippy', '--all-targets', '--no-default-features', '--locked', '--offline', '--', '-D', 'warnings'],
]
for index, command in enumerate(checks):
    report['active'] = f'check-{index}'
    save()
    log=A/f'{prefix}-check-{index}.log'
    with log.open('w') as out:
        code=subprocess.run(command, cwd=build/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
    report['checks'].append(dict(command=command,returncode=code,log=log.name))
    print('check',index,code,log.read_text()[-4500:] if code else '',flush=True)
    if code:
        report.pop('active')
        report['all_processes_reaped']=True
        verify()
        save()
        raise SystemExit(code)
report['active'] = 'build' 
save()
os.utime(build/'hypercurve/src/lib.rs', None)
command = [str(toolchain/'cargo'), 'test', '--lib', '--release', '--all-features', '--no-run',
           '--message-format=json', '--locked', '--offline']
with (A/f'{prefix}-build.jsonl').open('w') as out, (A/f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=build/'hypercurve', env=env, stdout=out, stderr=err, timeout=1200).returncode
report['build_returncode'] = code
report.pop('active')
if code:
    report['all_processes_reaped'] = True
    save()
    print((A/f'{prefix}-build.log').read_text()[-3500:], flush=True)
    raise SystemExit(code)
rows = [json.loads(line) for line in (A/f'{prefix}-build.jsonl').read_text().splitlines()]
artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact'
                and row['target']['name'] == 'hypercurve' and row.get('executable'))
assert not artifact['fresh']
binary = A/f'{prefix}-hypercurve'
shutil.copy2(artifact['executable'], binary)
report['binary'] = dict(path=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
names = [
    'bezier_offset::conversion_tests::original_parallel_normal_constraints_clip_finite_and_incident_components',
    'curve::tests::selected_parallel_fillet_clips_a_positive_dimensional_center_component_locally',
    'curve::tests::selected_parallel_fillet_keeps_contacts_beyond_interior_cusps',
    'curve::tests::fillet_center_contacts_keep_source_orientation_across_support_cusps',
    'curve::tests::distinct_parallel_sources_keep_a_shared_fillet_center_family',
]
report['selection'] = names
save()
for index, name in enumerate(names):
    report['active'] = name
    save()
    log = A/f'{prefix}-case-{index:03}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run([str(binary), '--exact', name, '--test-threads=1', '--color', 'never'],
                                  cwd=build/'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT,
                                  timeout=180).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    if not code:
        assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;', log.read_text()), name
    report['cases'].append(dict(name=name, returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name))
    report.pop('active')
    save()
    print(name, code, log.read_text()[-2200:] if code else '', flush=True)
verify()
assert hashlib.sha256(binary.read_bytes()).hexdigest() == report['binary']['sha256']
report['all_processes_reaped'] = True
report['all_sources_unchanged'] = True
save()
print('Probe complete; all child processes reaped.', flush=True)
