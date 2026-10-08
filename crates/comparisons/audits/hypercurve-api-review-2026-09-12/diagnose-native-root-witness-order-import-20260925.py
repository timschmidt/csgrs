from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'native-root-witness-order-20260925-{version}'
root = A / 'source-archives' / prefix
assert not root.exists()
guard_name = 'local-chord-complete-replay-20260924-v89-sources.json'
guard = json.loads((A / guard_name).read_text())
for name, sha in guard.items():
    source, destination = W / name, root / name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha, name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
p = root / 'hypercurve/src/bezier_offset.rs'
s = p.read_text()
a = s.index('fn recursive_projective_incident_point_order(')
b = s.index('fn recursive_projective_point_evidence_circle_residual_sign(', a)
block = s[a:b]
for needle, label in [
    ('    let first = match recursive_projective_point_source(first, policy)? {', 'first-point'),
    ('    let second = match recursive_projective_point_source(second, policy)? {', 'second-point'),
    ('    let authority = match semicircle.recursive_circle_frame_authority(policy)? {', 'circle-authority'),
    ('    let (authority, first) =', 'embed-first'),
    ('    let (authority, second) =', 'embed-second'),
    ('    let Some(first) = first.lifted_to(&authority.field) else {', 'lift-first'),
    ('    let cross = if let Some(sign) = cross.bounded_interval_sign(0..=512) {', 'cross-sign')]:
    assert block.count(needle) == 1, label
    block = block.replace(needle, f'    eprintln!("circle incident: {label}");\n' + needle)
s = s[:a] + block + s[b:]
a = s.index('    fn recursive_projective_parameter(')
b = s.index('    fn represented_retained_field_rational_value(', a)
block = s[a:b]
needle = '        if !(1..=2).contains(&degree) {'
assert block.count(needle) == 1
block = block.replace(needle, '        eprintln!("fiber import: degree={degree} exact={} native={}", self.represented_value().is_some(), self.data.representations.bezier.get().is_some());\n' + needle)
needle = '        let Ok(Some(parameter)) = attempt else {'
assert block.count(needle) == 1
block = block.replace(needle, '        eprintln!("fiber local import: success={}", matches!(&attempt, Ok(Some(_))));\n' + needle)
s = s[:a] + block + s[b:]
needle = """        if let Some(Classification::Decided(order)) = policy
            .bounded_exact_predicate_pass(|| self.recursive_projective_order(other, policy))?
        {"""
extra = """        let recursive_order = policy.bounded_exact_predicate_pass(|| self.recursive_projective_order(other, policy))?;
        eprintln!("circle order result: {recursive_order:?}");
        if let Some(Classification::Decided(order)) = recursive_order {"""
assert s.count(needle) == 1
s = s.replace(needle, extra)
a = s.index('fn embed_recursive_projective_point_source(')
b = s.index('fn recursive_projective_point_source_in_field(', a)
block = s[a:b]
needle = '    match authority.clone().joined_with_point(point.clone(), policy)? {'
assert block.count(needle) == 1
extra = """    for (label, field) in [("authority", authority.field.clone()), ("incoming", point.denominator.field())] {
        let (base, path) = field.base_and_extension_path();
        eprintln!("embed point: {label} sources={} extensions={}", base.sources.len(), path.len());
        for source in &base.sources {
            eprintln!("embed root: degree={} witness={}", source.polynomial_coefficients.len().saturating_sub(1), source.exact_point_witness().is_some());
        }
    }
    let joined = authority.clone().joined_with_point(point.clone(), policy)?;
    eprintln!("embed joined: {}", match &joined { Classification::Decided(Some(_)) => "some", Classification::Decided(None) => "none", Classification::Uncertain(_) => "uncertain" });
    match joined {"""
block = block.replace(needle, extra)
s = s[:a] + block + s[b:]
p.write_text(s)
manifest = {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in guard}
(A / f'{prefix}-sources.json').write_text(json.dumps(manifest, indent=2) + '\n')
build_root = A / 'build-workspace-20260925'
for name in guard:
    source, destination = root / name, build_root / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists() or source.read_bytes() != destination.read_bytes():
        shutil.copy2(source, destination)
    assert source.stat().st_ino != destination.stat().st_ino

def verify():
    for name, sha in guard.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == manifest[name], name
        assert hashlib.sha256((build_root / name).read_bytes()).hexdigest() == manifest[name], name

env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
repo = build_root / 'hypercurve'
env['HYPERCURVE_DIAG_CIRCLE_ORDER']='1'
report = dict(diagnostic_only=True, workspace_guard=guard_name, source_manifest=f'{prefix}-sources.json', source_directory=str(root), build_source_directory=str(build_root), checks=[], binaries={}, cases=[], all_processes_reaped=False)
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
    for label, command, limit in checks:
        row = run(label, command, limit)
        report['checks'].append(row)
        if row['returncode']:
            report['all_processes_reaped'] = True
            save()
            raise SystemExit(1)
os.utime(repo / 'src/lib.rs', None)
command = [str(toolchain / 'cargo'), 'test', '--test', 'hypercurve_curve', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
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
for target in ['hypercurve_curve']:
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
        selected = suffix == 'direct_bezier_pair_fillet_retains_both_incident_extensions'
        if selected:
            jobs.append((target, name))
jobs.sort(key=lambda job: (job[0] != 'hypercurve_curve', job[0] != 'hypercurve', job[1]))
assert len(jobs) == 1
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
