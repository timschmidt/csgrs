from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
root = Path('/tmp/hypercurve-rational-polynomial-v5-20260924')
prefix = 'rational-polynomial-20260924-v5-public-scalars'
manifest = json.loads((A / 'rational-polynomial-20260924-v5-sources.json').read_text())
def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report = dict(crates=[], all_processes_reaped=False)
def save():
    verify()
    report['all_sources_unchanged'] = True
    (A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2) + '\n')
for crate in ['hyperreal', 'hypersolve']:
    verify()
    command = [cargo, 'test', '--tests', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
    with (A / f'{prefix}-{crate}-build.jsonl').open('w') as out, (A / f'{prefix}-{crate}-build.log').open('w') as err:
        code = subprocess.run(command, cwd=root / crate, env=env, stdout=out, stderr=err, timeout=1200).returncode
    verify()
    assert code == 0
    record = dict(crate=crate, command=command, targets=[])
    report['crates'].append(record)
    rows = [json.loads(line) for line in (A / f'{prefix}-{crate}-build.jsonl').read_text().splitlines()]
    artifacts = [row for row in rows if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['kind'] == ['test'] and str(root / crate) in row['package_id']]
    assert artifacts
    for row in artifacts:
        assert not row['fresh']
        name = row['target']['name']
        binary = A / f'{prefix}-{crate}-{name}'
        shutil.copy2(row['executable'], binary)
        log = A / f'{prefix}-{crate}-{name}.log'
        start = time.monotonic()
        with log.open('w') as out:
            try:
                code = subprocess.run([str(binary), '--test-threads=2', '--color', 'never'], cwd=root / crate, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=180).returncode
            except subprocess.TimeoutExpired:
                code = 'timeout'
        record['targets'].append(dict(name=name, returncode=code, elapsed_seconds=time.monotonic()-start, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), log=log.name))
        print(crate, name, code, log.read_text()[-400:], flush=True)
        save()
report['all_processes_reaped'] = True
save()
print('Terminal; all scalar integration processes reaped.', flush=True)
