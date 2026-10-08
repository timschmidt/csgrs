from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
root = Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
prefix = f'native-root-witness-integrated-20260925-{version}'
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
report = dict(diagnostic_only=True, parent=subprocess.check_output(['git','rev-parse','HEAD'], cwd=W/'hypercurve', text=True).strip(), source_manifest=manifest_name, source_directory=str(root.resolve()), build_source_directory=str(build_root), checks=[], binaries={}, cases=[], all_processes_reaped=False)
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
if len(sys.argv) > 2 and sys.argv[2] == 'checked':
    changed = ['src/bezier_algebraic_image.rs', 'src/bezier_split.rs', 'src/bezier_offset.rs', 'src/bezier_parameter.rs', 'src/bezier_region.rs', 'src/curve.rs', 'src/curve_corner_chain.rs', 'src/curve_region_boolean.rs', 'tests/hypercurve_curve.rs']
    checks = [('fmt', [str(toolchain/'rustfmt'), '--edition', '2024', '--check', *changed], 60)]
    checks.extend((f'clippy-{index}', [str(toolchain/'cargo'), 'clippy', '--all-targets', feature, '--locked', '--offline', '--', '-D', 'warnings'], 1200) for index, feature in enumerate(['--all-features', '--no-default-features']))
    checks.append(('hyperbrep-check-all-targets', [str(toolchain/'cargo'), 'check', '--manifest-path', str(build_root/'hyperbrep/Cargo.toml'), '--all-targets', '--all-features', '--locked', '--offline'], 1200))
    for label, command, limit in checks:
        row = run(label, command, limit)
        report['checks'].append(row)
        if row['returncode']:
            report['all_processes_reaped'] = True
            save()
            raise SystemExit(1)
os.utime(repo / 'src/lib.rs', None)
command = [str(toolchain / 'cargo'), 'test', '--lib', '--test', 'hypercurve_curve', '--test', 'hypercurve_analytic_parallel_region', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
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
for target in ['hypercurve', 'hypercurve_curve', 'hypercurve_analytic_parallel_region']:
    artifact = next(row for row in artifacts if row.get('reason') == 'compiler-artifact' and row['target']['name'] == target and row.get('executable'))
    assert not artifact['fresh']
    binary = A / f'{prefix}-{target}'
    shutil.copy2(artifact['executable'], binary)
    report['binaries'][target] = dict(path=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest())

jobs = []
for target, binary in report['binaries'].items():
    names = [line[:-6] for line in subprocess.check_output([binary['path'], '--list'], text=True).splitlines() if line.endswith(': test')]
    for name in names:
        suffix = name.rsplit('::', 1)[-1]
        selected = (target == 'hypercurve' and (
            name.startswith(('bezier_parameter::', 'bezier_algebraic_image::', 'bezier_split::')) or
            suffix in {'selected_norm_carriers_exclude_foreign_endpoint_roots',
                       'selected_norm_carriers_preserve_repeated_interior_roots',
                       'bounded_point_import_reuses_certified_native_fiber_parameters',
                       'bounded_point_import_preserves_a_cached_local_field',
                       'selected_fiber_points_reuse_projective_roots_without_global_promotion'})) or (
            target == 'hypercurve_curve' and suffix in {
                'direct_bezier_pair_fillet_retains_both_incident_extensions',
                'line_parabola_mixed_exact_algebraic_fillet_is_an_exact_open_path',
                'spline_chamfer_materializes_only_the_incident_extension_cell',
                'one_curve_closed_spline_extensions_materialize_each_cell_once',
                'spline_line_fillet_preserves_the_authored_spline_and_adds_its_incident_cell'} ) or (
            target == 'hypercurve_analytic_parallel_region' and suffix in {
                'retained_arc_fillet_preserves_past_center_tangent_orientation',
                'retained_rational_arc_and_analytic_parallel_fillet_exactly',
                'retained_rational_arc_and_analytic_parallel_fillet_extends_exactly'})
        if selected:
            jobs.append((target, name))
jobs.sort(key=lambda job: (job[0] != 'hypercurve_curve', job[0] != 'hypercurve', job[1]))
assert len([job for job in jobs if job[0] == 'hypercurve_curve']) == 5
assert len([job for job in jobs if job[0] == 'hypercurve_analytic_parallel_region']) == 3
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
