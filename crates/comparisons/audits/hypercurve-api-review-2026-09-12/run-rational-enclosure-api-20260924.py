from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'rational-enclosure-api-20260924-{version}'
root = Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
assert not (A / f'{prefix}-terminal.json').exists()
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
cargo = str(toolchain / 'cargo')
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
env['PATH'] = str(toolchain) + os.pathsep + env['PATH']
report = dict(checks=[], cases=[], binaries={}, all_processes_reaped=False)
def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
def save():
    verify()
    report['all_sources_unchanged'] = True
    (A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
def run(repo, label, command, limit=1200, category='checks'):
    log = A / f'{prefix}-{label}.log'
    assert not log.exists()
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command, cwd=root/repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    verify()
    row = dict(label=label, repo=repo, command=command, returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name)
    report[category].append(row)
    print(label, code, log.read_text()[-2500:] if code else '', flush=True)
    if code:
        report['all_processes_reaped'] = True
        save()
        raise SystemExit(1)
    return row
verify()
run('hyperreal', 'fmt', [cargo, 'fmt', '--all', '--', '--check'], 60)
for feature in ['--all-features', '--no-default-features']:
    label = feature[2:]
    run('hyperreal', f'clippy-{label}', [cargo, 'clippy', '--all-targets', feature, '--locked', '--offline', '--', '-D', 'warnings'])
    targets = ['hyperreal', 'gmp_api_coverage', 'real_representations'] if feature == '--all-features' else ['hyperreal']
    command = [cargo, 'test', '--release', feature, '--no-run', '--message-format=json', '--locked', '--offline', '--lib']
    for target in targets[1:]:
        command.extend(['--test', target])
    build = run('hyperreal', f'build-{label}', command)
    artifacts = {}
    for line in (A/build['log']).read_text().splitlines():
        if not line.startswith('{'): continue
        row = json.loads(line)
        if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] in targets:
            target = row['target']['name']
            binary = A / f'{prefix}-{label}-{target}'
            shutil.copy2(row['executable'], binary)
            artifacts[target] = binary
            report['binaries'][f'{label}-{target}'] = dict(path=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    assert set(artifacts) == set(targets)
    binary = artifacts['hyperreal']
    names = [line[:-6] for line in subprocess.check_output([str(binary), '--list'], text=True).splitlines() if line.endswith(': test')]
    required = ['dyadic_enclosure_rounds_outward_and_is_tight_on_its_grid', 'certified_dyadic_interval_encloses_rational_points_and_scaled_computables', 'certified_rational_interval_is_exact_for_rationals_and_bounds_symbolic_values']
    for suffix in required:
        matches = [name for name in names if name.rsplit('::', 1)[-1] == suffix]
        assert len(matches) == 1, suffix
        run('hyperreal', f'{label}-{suffix}', [str(binary), '--exact', matches[0], '--nocapture', '--test-threads=1'], 75, 'cases')
    for target in targets[1:]:
        run('hyperreal', f'{label}-{target}-tests', [str(artifacts[target]), '--nocapture', '--test-threads=2'], 240, 'cases')
for repo in ['hyperlattice', 'hyperlimit', 'hypersolve', 'hypermesh', 'hyperpath']:
    run(repo, f'{repo}-callers', [cargo, 'check', '--all-targets', '--all-features', '--locked', '--offline'])
run('alumina-interface', 'alumina-core-callers', [cargo, 'check', '-p', 'alumina-interface-core', '--all-targets', '--all-features', '--locked', '--offline'])
for repo, targets in {
    'hyperreal': ['real_exact', 'structural_representations'],
    'hyperlattice': ['hyperreal_representations'],
    'hypersolve': ['hyperreal_representations'],
    'hypermesh': ['boolean_hyperreal_representations'],
    'hyperpath': ['hyperreal_representations'],
    'hypercurve': ['hyperreal_representations'],
}.items():
    command = [cargo, 'check', '--manifest-path', 'fuzz/Cargo.toml', '--locked', '--offline']
    for target in targets: command.extend(['--bin', target])
    run(repo, f'{repo}-fuzz-callers', command)
report['all_processes_reaped'] = True
save()
print('Scalar enclosure and caller checks complete; all owned processes reaped.', flush=True)
