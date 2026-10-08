from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'selected-fiber-normalization-20260925-{version}'
root = A / 'source-archives' / prefix
assert not root.exists()
guard_name = 'local-chord-complete-replay-20260924-v99-sources.json'
guard = json.loads((A / guard_name).read_text())
for name, sha in guard.items():
    source, destination = W / name, root / name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha, name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
p = root / 'hypercurve/src/bezier_offset.rs'
s = p.read_text()
helper = """
fn diagnostic_selected_fiber_polynomial(label: &str, polynomial: &BivariatePolynomial) {
    static COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    if COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) >= 200 { return; }
    let count = polynomial.coefficients.iter().map(Vec::len).sum::<usize>();
    let rationals = polynomial.coefficients.iter().flatten().filter_map(Real::exact_rational_ref).collect::<Vec<_>>();
    let bits = rationals.iter().map(|r| r.numerator().bits().max(r.denominator().bits())).max().unwrap_or(0);
    eprintln!("fiber polynomial: {label} rows={} columns={} coefficients={count} rational={} max-bits={bits}", polynomial.coefficients.len(), polynomial.coefficients.iter().map(Vec::len).max().unwrap_or(0), rationals.len());
}

"""
a = s.index('fn algebraic_selected_fiber_root_predicate_sign(')
s = s[:a] + helper + s[a:]
a = s.index('    fn selected_parallel_normal_rational_selected_fiber_intersections(')
b = s.index('    fn selected_parallel_normal_replay_rational_circle_component(',a)
block = s[a:b]
needle = '        let tangent_candidates = candidate.is_some();'
assert block.count(needle) == 1
block = block.replace(needle, """        eprintln!("fiber kernel: target-degree={} retained-circle={} center-degree={} center-rational={}/{} tangent={}", other.degree(), other.retained_circular_conic().is_some(), center_parameter.polynomial().degree(), center_parameter.polynomial().coefficients().iter().filter(|c| c.exact_rational_ref().is_some()).count(), center_parameter.polynomial().coefficients().len(), candidate.is_some());
        diagnostic_selected_fiber_polynomial("incidence", &system.incidence);
""" + needle)
block = block.replace('        let mut retained = Vec::with_capacity(parameters.len());', '        eprintln!("fiber roots: {}", parameters.len());\n        let mut retained = Vec::with_capacity(parameters.len());')
block = block.replace('        for other_parameter in parameters {', '        for other_parameter in parameters {\n            eprintln!("fiber candidate: exact={} native={} projective={}", other_parameter.represented_value().is_some(), other_parameter.data.representations.bezier.get().is_some(), other_parameter.data.representations.projective.get().is_some());')
for needle,label in [
    ('                match other_parameter.radical_sum_sign(\n                    &system.circle,','circle'),
    ('            let selected =','half-plane'),
    ('            let tangent_cross_sign =','tangent-cross')]:
    assert block.count(needle) == 1,label
    block = block.replace(needle, f'            eprintln!("fiber stage: {label}");\n'+needle)
s = s[:a]+block+s[b:]
a = s.index('fn algebraic_selected_fiber_root_radical_sum_sign(')
b = s.index('fn algebraic_selected_fiber_root_square_root_sum_sign(',a)
block = s[a:b]
needle = '    let incidence = &authority.data.incidence;'
assert block.count(needle) == 1
block = block.replace(needle, """    diagnostic_selected_fiber_polynomial("radical-sum A", &expression.rational);
    diagnostic_selected_fiber_polynomial("radical-sum B", &expression.radical);
""" + needle)
s = s[:a]+block+s[b:]
a = s.index('fn algebraic_selected_fiber_root_predicate_sign(')
b = s.index('fn validate_selected_fiber_pair_base(',a)
block = s[a:b]
needle = '            let zero_report = count_bivariate_common_fiber_roots_at_algebraic_parameter('
assert block.count(needle) == 1
block = block.replace(needle, """            diagnostic_selected_fiber_polynomial("common-root incidence", incidence);
            diagnostic_selected_fiber_polynomial("common-root predicate", predicate);
            eprintln!("fiber common-root: steps={refinement_steps} base-degree={} base-rational={}/{}", retained_root.polynomial_coefficients.len()-1, retained_root.polynomial_coefficients.iter().filter(|c| c.exact_rational_ref().is_some()).count(), retained_root.polynomial_coefficients.len());
""" + needle)
s = s[:a]+block+s[b:]
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
command = [str(toolchain / 'cargo'), 'test', '--test', 'hypercurve_analytic_parallel_region', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
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
for target in ['hypercurve_analytic_parallel_region']:
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
        selected = suffix == 'retained_arc_fillet_preserves_past_center_tangent_orientation'
        if selected:
            jobs.append((target, name))
jobs.sort(key=lambda job: (job[0] != 'hypercurve_analytic_parallel_region', job[0] != 'hypercurve', job[1]))
assert len(jobs) == 1
report['selection'] = jobs
print('fresh binaries; selected',len(jobs),'cases',flush=True)
for target, name in jobs:
    report['active_case'] = [target, name]
    save()
    binary = report['binaries'][target]
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
    row = run(name.rsplit('::',1)[-1], [binary['path'], '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], 25)
    row.update(target=target, name=name)
    report['cases'].append(row)
    save()
report.pop('active_case', None)
report['all_processes_reaped'] = True
save()
print('Expanded diagnostic terminal; every child process reaped.', flush=True)
raise SystemExit(int(any(row['returncode'] != 0 for row in report['cases'])))
