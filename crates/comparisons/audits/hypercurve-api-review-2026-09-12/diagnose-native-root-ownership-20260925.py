from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
root = Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
prefix = f'native-root-ownership-20260925-{version}'
manifest = json.loads((A / f'local-chord-complete-replay-20260924-{version}-sources.json').read_text())
def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name

env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
repo = root / 'hypercurve'
report = dict(diagnostic_only=True, checks=[], binaries={}, cases=[], all_processes_reaped=False)
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
    changed = ['src/bezier_offset.rs', 'src/bezier_parameter.rs', 'src/bezier_region.rs', 'src/curve.rs', 'src/curve_corner_chain.rs', 'src/curve_region_boolean.rs', 'tests/hypercurve_curve.rs']
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
command = [str(toolchain / 'cargo'), 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
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
for target in ['hypercurve']:
    artifact = next(row for row in artifacts if row.get('reason') == 'compiler-artifact' and row['target']['name'] == target and row.get('executable'))
    assert not artifact['fresh']
    binary = A / f'{prefix}-{target}'
    shutil.copy2(artifact['executable'], binary)
    report['binaries'][target] = dict(path=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest())

suffixes = [
    ('hypercurve', 'imported_root_comparisons_preserve_owned_and_excluded_endpoints'),
    ('hypercurve', 'decreasing_root_charts_preserve_owned_and_excluded_endpoints'),
    ('hypercurve', 'solver_owned_root_refinement_excludes_repeated_lower_endpoints'),
    ('hypercurve', 'collapsed_exact_real_representation_imports_as_an_exact_parameter'),
    ('hypercurve', 'isolation_retains_large_rational_roots_until_projection_is_requested'),
    ('hypercurve', 'projective_images_preserve_exterior_singletons_across_unused_poles'),
    ('hypercurve', 'independent_singleton_charts_reuse_nested_root_identity'),
]
if 'minimal' in sys.argv:
    suffixes = suffixes[:2]
for target, suffix in suffixes:
    binary = report['binaries'][target]['path']
    names = [line[:-6] for line in subprocess.check_output([binary, '--list'], text=True).splitlines() if line.endswith(': test')]
    matches = [name for name in names if name.rsplit('::', 1)[-1] == suffix]
    assert len(matches) == 1, suffix
    report['cases'].append(run(suffix, [binary, '--exact', matches[0], '--nocapture', '--test-threads=1', '--color', 'never'], 75 if target == 'hypercurve' else 90))
    save()
report['all_processes_reaped'] = True
save()
print('Diagnostic terminal; every child process reaped.', flush=True)
raise SystemExit(int(any(row['returncode'] != 0 for row in report['cases'])))
