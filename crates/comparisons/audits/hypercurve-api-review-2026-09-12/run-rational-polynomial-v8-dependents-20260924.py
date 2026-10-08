from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
root = Path(f'/tmp/hypercurve-rational-polynomial-{version}-20260924')
prefix = f'rational-polynomial-20260924-{version}-dependents'
bindings = json.loads((A / f'rational-polynomial-20260924-{version}-sources.json').read_text())
parent = json.loads((A / f'rational-polynomial-20260924-{version}-terminal.json').read_text())
assert parent['all_processes_reaped'] and all(row['returncode'] == 0 for row in parent['checks'] + parent['tests'])
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report = dict(crates=[], all_processes_reaped=False)
def save():
    verify()
    report['all_sources_unchanged'] = True
    (A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2) + '\n')
for crate in ['hyperlattice', 'hyperlimit', 'hypersolve', 'hypercurve']:
    verify()
    repo = root / crate
    os.utime(repo / 'src/lib.rs', None)
    record = dict(crate=crate, checks=[], cases=[])
    report['crates'].append(record)
    for feature in ['--all-features', '--no-default-features']:
        log = A / f'{prefix}-{crate}-clippy-{len(record["checks"])}.log'
        command = [cargo, 'clippy', '--all-targets', feature, '--locked', '--offline', '--', '-D', 'warnings']
        with log.open('w') as out:
            code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=1200).returncode
        verify()
        record['checks'].append(dict(command=command, returncode=code, log=log.name))
        print(crate, feature, code, flush=True)
        if code:
            print(log.read_text()[-3000:], flush=True)
            report['all_processes_reaped'] = True
            save()
            raise SystemExit(1)
    command = [cargo, 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
    with (A / f'{prefix}-{crate}-build.jsonl').open('w') as out, (A / f'{prefix}-{crate}-build.log').open('w') as err:
        code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
    verify()
    assert code == 0
    rows = [json.loads(line) for line in (A / f'{prefix}-{crate}-build.jsonl').read_text().splitlines()]
    libraries = [row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hyperreal']
    assert libraries and all(str(root / 'hyperreal') in row['package_id'] for row in libraries)
    record['hyperreal_artifacts'] = libraries
    artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == crate and row.get('executable'))
    assert not artifact['fresh']
    binary = A / f'{prefix}-{crate}-libtest'
    shutil.copy2(artifact['executable'], binary)
    record['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
    if crate != 'hypercurve':
        selected = [('all', ['--test-threads=2'], 180)]
    else:
        names = [line[:-6] for line in subprocess.check_output([str(binary), '--list'], text=True).splitlines() if line.endswith(': test')]
        suffixes = [
            'recursive_bounds_refine_each_shared_source_once',
            'nonlinear_selected_corner_chamfers_through_retained_fixed_distance_image',
            'recursive_selected_radial_projective_chamfer_reenters_corner_kernel',
            'extended_fillet_region_classifies_both_sides_of_its_companion',
            'nonrepresented_chord_and_selected_circle_complete_the_fillet_kernel',
            'selected_parallel_companion_fillets_without_range_promotion',
            'pair_native_boolean_algebraic_chord_corner_publishes_a_third_generation_fillet',
        ]
        selected = []
        for suffix in suffixes:
            matches = [name for name in names if name.rsplit('::', 1)[-1] == suffix]
            assert len(matches) == 1
            selected.append((suffix, ['--exact', matches[0], '--nocapture', '--test-threads=1'], 180 if suffix.startswith('nonrepresented_chord') else 75))
    for index, (name, args, limit) in enumerate(selected):
        log = A / f'{prefix}-{crate}-case-{index}.log'
        start = time.monotonic()
        with log.open('w') as out:
            try:
                code = subprocess.run([str(binary), *args, '--color', 'never'], cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=limit).returncode
            except subprocess.TimeoutExpired:
                code = 'timeout'
        verify()
        row = dict(name=name, returncode=code, elapsed_seconds=time.monotonic()-start, limit_seconds=limit, log=log.name)
        record['cases'].append(row)
        print(crate, row, log.read_text()[-700:], flush=True)
        save()
        if crate != 'hypercurve' and code:
            report['all_processes_reaped'] = True
            save()
            raise SystemExit(1)
report['all_processes_reaped'] = True
save()
print('Terminal; all dependent processes reaped.', flush=True)
