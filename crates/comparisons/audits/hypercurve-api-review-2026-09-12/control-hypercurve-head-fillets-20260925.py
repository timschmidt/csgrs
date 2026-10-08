from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'hypercurve-head-fillets-20260925-{version}'
root = A / 'source-archives' / prefix
assert not root.exists()
workspace_manifest_name = f'local-chord-complete-replay-20260924-{version}-sources.json'
workspace_manifest = json.loads((A/workspace_manifest_name).read_text())
hc = W/'hypercurve'
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=hc, text=True).strip()
hs_head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=W/'hypersolve', text=True).strip()
assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=hc)
changed = subprocess.check_output(['git', 'diff', '--name-only'], cwd=hc, text=True).splitlines()
assert changed and all('hypercurve/'+name in workspace_manifest for name in changed)
manifest = {}
for name, expected in workspace_manifest.items():
    source = W/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == expected, name
    destination = root/name
    destination.parent.mkdir(parents=True, exist_ok=True)
    if name.startswith('hypercurve/') and name[len('hypercurve/'):] in changed:
        destination.write_bytes(subprocess.check_output(['git', 'show', head+':'+name[len('hypercurve/'):]], cwd=hc))
    else:
        shutil.copy2(source, destination)
    assert (source.stat().st_dev, source.stat().st_ino) != (destination.stat().st_dev, destination.stat().st_ino)
    manifest[name] = hashlib.sha256(destination.read_bytes()).hexdigest()
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest, indent=2)+'\n')

def verify():
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=hc, text=True).strip() == head
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=W/'hypersolve', text=True).strip() == hs_head
    for name, expected in workspace_manifest.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == expected, name
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == manifest[name], name

env = dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
targets = ['hypercurve_analytic_parallel_region', 'hypercurve_curve']
command = [cargo, 'test', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
for target in targets:
    command.extend(['--test', target])
report = dict(diagnostic_only=True, hypercurve_head=head, hypersolve_head=hs_head,
              workspace_guard_manifest=workspace_manifest_name,
              source_manifest=f'{prefix}-sources.json', restored_hypercurve_files=changed,
              command=command, binaries={}, cases=[], all_processes_reaped=False)
def save():
    verify()
    report['all_sources_unchanged'] = True
    (A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Control snapshot:', head, 'with current shared dependencies;', len(manifest), 'physical copies.', flush=True)
with (A/f'{prefix}-build.jsonl').open('w') as out, (A/f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=root/'hypercurve', env=env, stdout=out, stderr=err, timeout=1200).returncode
report['build_returncode'] = code
if code:
    report['all_processes_reaped'] = True
    save()
    print((A/f'{prefix}-build.log').read_text()[-2000:])
    raise SystemExit(1)
artifacts = [json.loads(line) for line in (A/f'{prefix}-build.jsonl').read_text().splitlines()]
for target in targets:
    artifact = next(row for row in artifacts if row.get('reason') == 'compiler-artifact' and row['target']['name'] == target and row.get('executable'))
    assert not artifact['fresh']
    binary = A/f'{prefix}-{target}'
    shutil.copy2(artifact['executable'], binary)
    report['binaries'][target] = dict(path=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
for target, name in [
    ('hypercurve_analytic_parallel_region', 'retained_arc_fillet_preserves_past_center_tangent_orientation'),
    ('hypercurve_analytic_parallel_region', 'retained_rational_arc_and_analytic_parallel_fillet_exactly'),
    ('hypercurve_analytic_parallel_region', 'retained_rational_arc_and_analytic_parallel_fillet_extends_exactly'),
    ('hypercurve_curve', 'direct_bezier_pair_fillet_materializes_both_incident_extensions'),
]:
    verify()
    binary = report['binaries'][target]
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
    command = [binary['path'], '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never']
    log = A/f'{prefix}-{name}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command, cwd=root/'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=30).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    report['cases'].append(dict(target=target, name=name, command=command, returncode=code,
                               elapsed_seconds=time.monotonic()-start, limit_seconds=30, log=log.name))
    save()
    print(name, code, log.read_text()[-1000:], flush=True)
report['all_processes_reaped'] = True
save()
print('Control diagnostic completed; all children reaped.', flush=True)
