from pathlib import Path
import difflib, hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version, mode = sys.argv[1:]
assert version.startswith('v') and version[1:].isdigit()
assert mode == 'candidate'
prefix = f'native-norm-ownership-20260925-{version}-{mode}'
root = A / 'source-archives' / prefix
assert not root.exists()
guard_name = f'local-chord-complete-replay-20260924-{version}-sources.json'
guard = json.loads((A / guard_name).read_text())
baseline = A / 'source-archives' / 'hypercurve-local-chord-complete-replay-v67-20260924'
baseline_manifest = json.loads((A / 'local-chord-complete-replay-20260924-v67-sources.json').read_text())
hc = W / 'hypercurve'
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=hc, text=True).strip()
assert head.startswith('9d5d782b')
changed = subprocess.check_output(['git', 'diff', '--name-only'], cwd=hc, text=True).splitlines()
assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=hc)
selected = ['src/bezier_offset.rs', 'src/bezier_parameter.rs']
delta_files = []
patch = ''
for name, expected in guard.items():
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == expected, name
    old = (baseline / name).read_bytes()
    assert hashlib.sha256(old).hexdigest() == baseline_manifest[name], name
    if old != (W / name).read_bytes():
        assert name.startswith('hypercurve/')
        local = name.removeprefix('hypercurve/')
        delta_files.append(local)
        before, after = old.decode(), (W / name).read_text()
        if local == 'src/bezier_parameter.rs':
            # The old half-open refinement workaround exists only in the
            # larger pending migration, not HEAD. Its removal stays with that
            # migration; this milestone publishes the constructor correction.
            begin = "impl<'a> RefinedParameter<'a> {"
            end = '\nfn compare_distinct_parameters('
            first, last = before.index(begin), before.index(end, before.index(begin))
            new_first, new_last = after.index(begin), after.index(end, after.index(begin))
            original = before[first:last]
            revised = after[new_first:new_last]
            if original != revised:
                branch = '                        // Imported solver isolators own'
                b = original.index(branch)
                e = original.index('                    }', original.index('.ok_or(CurveError::InvalidBezierAlgebraicParameter)?', b))
                expected = original[:b] + '                        return Err(CurveError::InvalidBezierAlgebraicParameter);\n' + original[e:]
                assert revised == expected
                parent_text = subprocess.check_output(['git', 'show', head + ':' + local], cwd=hc, text=True)
                assert branch not in parent_text
                after = after[:new_first] + original + after[new_last:]
        patch += f'diff --git a/{local} b/{local}\n'
        patch += ''.join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                             fromfile='a/' + local, tofile='b/' + local))
assert sorted(delta_files) == selected, delta_files
patch_path = A / f'native-norm-ownership-20260925-{version}.patch'
if patch_path.exists():
    assert patch_path.read_text() == patch
else:
    patch_path.write_text(patch)

for name in guard:
    source = W / name
    destination = root / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    local = name.removeprefix('hypercurve/')
    if name.startswith('hypercurve/') and local in changed:
        destination.write_bytes(subprocess.check_output(['git', 'show', head + ':' + local], cwd=hc))
    else:
        shutil.copy2(source, destination)
    assert (source.stat().st_dev, source.stat().st_ino) != (destination.stat().st_dev, destination.stat().st_ino)

repo = root / 'hypercurve'
regressions = ['selected_norm_carriers_exclude_foreign_endpoint_roots',
               'selected_norm_carriers_preserve_repeated_interior_roots']
controls = ['selected_norm_isolation_preserves_exterior_exact_and_algebraic_roots',
            'selected_fiber_norm_isolation_refines_past_the_old_limit']
prior_failures = []
subprocess.run(['git', 'apply', '--check', str(patch_path)], cwd=repo, check=True)
subprocess.run(['git', 'apply', str(patch_path)], cwd=repo, check=True)

manifest = {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in guard}
(A / f'{prefix}-sources.json').write_text(json.dumps(manifest, indent=2) + '\n')
def verify():
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=hc, text=True).strip() == head
    for name, expected in guard.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == expected, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == manifest[name], name

env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
report = dict(mode=mode, parent=head, selected_files=selected, workspace_changed=changed,
              workspace_guard=guard_name, source_manifest=f'{prefix}-sources.json', patch=patch_path.name,
              checks=[], binaries={}, cases=[], all_processes_reaped=False)
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
checks.extend((f'clippy-{i}', [str(toolchain / 'cargo'), 'clippy', '--all-targets', feature,
                            '--locked', '--offline', '--', '-D', 'warnings'], 1200)
              for i, feature in enumerate(['--all-features', '--no-default-features']))
for label, command, limit in checks if mode == 'candidate' else []:
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
if mode == 'candidate':
    targets += ['hypercurve_curve_region_boolean', 'hypercurve_curve_region_boolean_fuzz',
                'hypercurve_pcb_boolean_regressions', 'hypercurve_bezier_arrangement']
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
    for name in names:
        suffix = name.rsplit('::', 1)[-1]
        include = suffix in regressions + prior_failures if mode == 'parent' else (
            target != 'hypercurve' or name.startswith(('curve_region_boolean::', 'bezier_parameter::')) or
            name.startswith('bezier_region::tests::') and ('ray' in suffix or 'winding' in suffix) or
            suffix == 'denominator_sign_tracks_the_requested_range_and_keeps_unit_cache_scope' or
            suffix in regressions + controls)
        if include:
            jobs.append((target, name))
assert all(sum(name.rsplit('::', 1)[-1] == regression for _, name in jobs) == 1 for regression in regressions)
jobs.sort(key=lambda job: (0 if job[1].rsplit('::', 1)[-1] in prior_failures else
                          1 if job[1].rsplit('::', 1)[-1] in regressions else 2))
report['selection'] = jobs
print('fresh binaries; selected cases', len(jobs), flush=True)
for index, (target, name) in enumerate(jobs):
    report['active_case'] = [target, name]
    save()
    binary = report['binaries'][target]
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
    # This corpus completed in 70.45s in V62. Give the correctness run headroom;
    # preserve actual timings instead of making load variation a test assertion.
    limit = 120 if name == 'easyduino_uno_scale_process_image_with_holes_corpus' else 75
    row = run(f'case-{index:03}', [binary['path'], '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], limit)
    row.update(target=target, name=name)
    report['cases'].append(row)
    save()
    if row['returncode'] != 0:
        print(name, row['returncode'], (A / row['log']).read_text()[-2000:], flush=True)
    elif index < 2 or (index + 1) % 20 == 0:
        print('completed', index + 1, 'of', len(jobs), flush=True)
report.pop('active_case', None)
report['all_processes_reaped'] = True
save()
passed = all(row['returncode'] == 0 for row in report['cases'])
print('terminal:', 'passed' if passed else 'failed', '; every child reaped', flush=True)
raise SystemExit(0 if passed else 1)
