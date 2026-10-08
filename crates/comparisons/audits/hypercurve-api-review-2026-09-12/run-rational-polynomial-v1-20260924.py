from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
root = Path('/tmp/hypercurve-rational-polynomial-v1-20260924')
prefix = 'rational-polynomial-20260924-v1'
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
probe = A / 'rational-polynomial-probe-20260924.rs'
probe_sha = hashlib.sha256(probe.read_bytes()).hexdigest()
def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256(probe.read_bytes()).hexdigest() == probe_sha
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
cargo = str(toolchain / 'cargo')
repo = root / 'hyperreal'
report = dict(checks=[], tests=[], probe_source_sha256=probe_sha, all_processes_reaped=False)
def save():
    verify()
    report['all_sources_unchanged'] = True
    (A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2) + '\n')
def execute(label, command, limit=1200):
    log = A / f'{prefix}-{label}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    verify()
    row = dict(label=label, command=command, returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name)
    print(label, code, log.read_text()[-1600:] if code else '', flush=True)
    return row
verify()
os.utime(repo / 'src/lib.rs', None)
for feature in ['--all-features', '--no-default-features']:
    row = execute(f'clippy-{len(report["checks"])}', [cargo, 'clippy', '--all-targets', feature, '--locked', '--offline', '--', '-D', 'warnings'])
    report['checks'].append(row)
    if row['returncode']:
        report['all_processes_reaped'] = True
        save()
        raise SystemExit(1)
command = [cargo, 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (A / f'{prefix}-build.jsonl').open('w') as out, (A / f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0
rows = [json.loads(line) for line in (A / f'{prefix}-build.jsonl').read_text().splitlines()]
artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hyperreal' and row.get('executable'))
assert not artifact['fresh']
binary = A / f'{prefix}-libtest'
shutil.copy2(artifact['executable'], binary)
report['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
for label, args, limit in [('focused', ['polynomial_evaluation', '--test-threads=1'], 120), ('all', ['--test-threads=2'], 180)]:
    row = execute(label, [str(binary), *args, '--color', 'never'], limit)
    report['tests'].append(row)
    if row['returncode']:
        report['all_processes_reaped'] = True
        save()
        raise SystemExit(1)
command = [cargo, 'build', '--lib', '--release', '--no-default-features', '--message-format=json', '--locked', '--offline']
with (A / f'{prefix}-probe-build.jsonl').open('w') as out, (A / f'{prefix}-probe-build.log').open('w') as err:
    code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0
rows = [json.loads(line) for line in (A / f'{prefix}-probe-build.jsonl').read_text().splitlines()]
externs = {}
for name in ['hyperreal', 'num']:
    row = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == name and not row.get('executable'))
    if name == 'hyperreal':
        assert not row['fresh']
        assert str(repo) in row['package_id']
    externs[name] = next(path for path in row['filenames'] if path.endswith('.rlib'))
probe_binary = A / f'{prefix}-probe'
command = [str(toolchain / 'rustc'), '--edition', '2024', '-O', str(probe), '-o', str(probe_binary), '-L', 'dependency=' + str(Path(env['CARGO_TARGET_DIR']) / 'release/deps')]
for name, path in externs.items():
    command += ['--extern', name + '=' + path]
row = execute('probe-compile', command)
assert row['returncode'] == 0
report['probe_binary_sha256'] = hashlib.sha256(probe_binary.read_bytes()).hexdigest()
measurements = []
for name in subprocess.check_output([str(probe_binary), 'list'], text=True).splitlines():
    row = execute('probe-' + name, [str(probe_binary), name], 30)
    if row['returncode'] == 0:
        row.update(json.loads((A / row['log']).read_text()))
    measurements.append(row)
    (A / f'{prefix}-measurements.json').write_text(json.dumps(measurements, indent=2) + '\n')
report['measurements'] = measurements
report['all_processes_reaped'] = True
save()
print('Terminal; all candidate processes reaped.', flush=True)
