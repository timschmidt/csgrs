from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
root = Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
prefix = f'local-fraction-coefficients-20260925-{version}'
manifest_name = f'local-chord-complete-replay-20260924-{version}-sources.json'
manifest = json.loads((A / manifest_name).read_text())
build_root = A / 'build-workspace-20260925'
for name in manifest:
    source, destination = root / name, build_root / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists() or destination.read_bytes() != source.read_bytes():
        shutil.copy2(source, destination)
    assert (source.stat().st_dev, source.stat().st_ino) != (destination.stat().st_dev, destination.stat().st_ino)
def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((build_root / name).read_bytes()).hexdigest() == sha, name

env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
repo = build_root / 'hypercurve'
report = dict(hypersolve_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypersolve',text=True).strip(), diagnostic_only=True, parent=subprocess.check_output(['git','rev-parse','HEAD'], cwd=W/'hypercurve', text=True).strip(), source_manifest=manifest_name, source_directory=str(root.resolve()), build_source_directory=str(build_root), checks=[], binaries={}, cases=[], all_processes_reaped=False)
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
    row = dict(label=label, command=command, returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name)
    print(label, code, log.read_text()[-3200:], flush=True)
    return row

verify()
repo = build_root / 'hypersolve'
for label, command in [
    ('hypersolve-fmt', [str(toolchain/'rustfmt'), '--edition', '2024', '--check', 'src/algebraic_fiber.rs']),
    ('hypersolve-clippy-all', [str(toolchain/'cargo'), 'clippy', '--all-targets', '--all-features', '--locked', '--offline', '--', '-D', 'warnings']),
    ('hypersolve-clippy-minimal', [str(toolchain/'cargo'), 'clippy', '--all-targets', '--no-default-features', '--locked', '--offline', '--', '-D', 'warnings']),
]:
    row = run(label, command, 1200)
    report['checks'].append(row)
    if row['returncode']:
        report['all_processes_reaped'] = True
        save()
        raise SystemExit(1)
os.utime(repo/'src/lib.rs', None)
command = [str(toolchain/'cargo'), 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (A/f'{prefix}-hypersolve-build.jsonl').open('w') as out, (A/f'{prefix}-hypersolve-build.log').open('w') as err:
    code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
report['hypersolve_build_returncode'] = code
if code:
    report['all_processes_reaped'] = True
    save()
    print((A/f'{prefix}-hypersolve-build.log').read_text()[-3000:],flush=True)
    raise SystemExit(1)
artifacts = [json.loads(line) for line in (A/f'{prefix}-hypersolve-build.jsonl').read_text().splitlines()]
artifact = next(row for row in artifacts if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hypersolve' and row.get('executable'))
assert not artifact['fresh']
binary = A/f'{prefix}-hypersolve'
shutil.copy2(artifact['executable'], binary)
report['binaries']['hypersolve'] = dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
for suffix in ['nonrational_local_coefficients_preserve_repeated_fiber_roots', 'local_division_preserves_arbitrary_exact_base_coefficients', 'local_division_preserves_wide_exact_quotients_beyond_modular_reconstruction', 'local_division_removes_only_foreign_factors_and_reuses_older_values']:
    name = 'algebraic_fiber::tests::'+suffix
    row = run(suffix, [str(binary), '--exact', name, '--test-threads=1', '--nocapture'], 75)
    row.update(target='hypersolve',name=name)
    report['cases'].append(row)
    save()
    if row['returncode']:
        report['all_processes_reaped'] = True
        save()
        raise SystemExit(1)
row = run('hypersolve-full', [str(binary), '--test-threads=2', '--color', 'never'], 600)
row.update(target='hypersolve',name='full-library')
report['cases'].append(row)
save()
if row['returncode']:
    report['all_processes_reaped'] = True
    save()
    raise SystemExit(1)
repo = build_root / 'hypercurve'
os.utime(repo / 'src/lib.rs', None)
command = [str(toolchain / 'cargo'), 'test', '--test', 'hypercurve_curve', '--test', 'hypercurve_analytic_parallel_region', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (A / f'{prefix}-build.jsonl').open('w') as out, (A / f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
report['build_returncode'] = code
if code:
    report['all_processes_reaped'] = True
    save()
    print((A / f'{prefix}-build.log').read_text()[-3000:], flush=True)
    raise SystemExit(1)
artifacts = [json.loads(line) for line in (A / f'{prefix}-build.jsonl').read_text().splitlines()]
for target in ['hypercurve_curve', 'hypercurve_analytic_parallel_region']:
    artifact = next(row for row in artifacts if row.get('reason') == 'compiler-artifact' and row['target']['name'] == target and row.get('executable'))
    assert not artifact['fresh']
    binary = A / f'{prefix}-{target}'
    shutil.copy2(artifact['executable'], binary)
    report['binaries'][target] = dict(path=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest())

jobs = []
for target, binary in report['binaries'].items():
    if target == 'hypersolve':
        continue
    names = [line[:-6] for line in subprocess.check_output([binary['path'], '--list'], text=True).splitlines() if line.endswith(': test')]
    for name in names:
        suffix = name.rsplit('::', 1)[-1]
        selected = target == 'hypercurve_curve' and suffix in {
            'direct_bezier_pair_fillet_retains_both_incident_extensions',
            'line_parabola_mixed_exact_algebraic_fillet_is_an_exact_open_path',
            'one_curve_closed_spline_extensions_materialize_each_cell_once',
            'spline_chamfer_materializes_only_the_incident_extension_cell',
            'spline_line_fillet_preserves_the_authored_spline_and_adds_its_incident_cell',
            'polynomial_chamfer_materializes_represented_incident_extension',
            'rational_chamfer_materializes_the_incident_projective_cell'}
        selected |= target == 'hypercurve_analytic_parallel_region' and suffix in {
            'retained_arc_fillet_preserves_past_center_tangent_orientation',
            'retained_rational_arc_and_analytic_parallel_fillet_exactly',
            'retained_rational_arc_and_analytic_parallel_fillet_extends_exactly'}
        if selected:
            jobs.append((target, name))
jobs.sort(key=lambda job: (job[0] != 'hypercurve_curve', job[0] != 'hypercurve', job[1]))
assert len(jobs) == 10
report['selection'] = jobs
print('fresh binaries; selected',len(jobs),'cases',flush=True)
for target, name in jobs:
    report['active_case'] = [target, name]
    save()
    binary = report['binaries'][target]
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
    row = run(name.rsplit('::',1)[-1], [binary['path'], '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], 75)
    row.update(target=target, name=name)
    report['cases'].append(row)
    save()
report.pop('active_case', None)
report['all_processes_reaped'] = True
save()
print('Expanded diagnostic terminal; every child process reaped.', flush=True)
raise SystemExit(int(any(row['returncode'] != 0 for row in report['cases'])))
