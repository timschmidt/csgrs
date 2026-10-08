from pathlib import Path
import hashlib, json, os, shutil, subprocess

A = Path(__file__).resolve().parent
W = A.parent
root = Path('/tmp/hypercurve-rational-polynomial-v6-20260924')
prefix = 'rational-polynomial-20260924-v6-inventory'
manifest = json.loads((A / 'rational-polynomial-20260924-v6-sources.json').read_text())
old = json.loads((A / 'rational-polynomial-20260924-v5-sources.json').read_text())
assert [name for name in manifest if manifest[name] != old[name]] == ['hyperreal/tests/gmp_api_coverage.rs']
def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
cargo = str(toolchain / 'cargo')
repo = root / 'hyperreal'
report = dict(checks=[], all_processes_reaped=False)
verify()
subprocess.run([cargo, 'fmt', '--all', '--', '--check'], cwd=repo, env=env, check=True, timeout=60)
for feature in ['--all-features', '--no-default-features']:
    command = [cargo, 'clippy', '--all-targets', feature, '--locked', '--offline', '--', '-D', 'warnings']
    log = A / f'{prefix}-clippy-{len(report["checks"])}.log'
    with log.open('w') as out:
        code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=1200).returncode
    verify()
    report['checks'].append(dict(command=command, returncode=code, log=log.name))
    assert code == 0
    print('Repaired inventory Clippy passed:', feature, flush=True)
os.utime(repo / 'src/lib.rs', None)
command = [cargo, 'test', '--test', 'gmp_api_coverage', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (A / f'{prefix}-build.jsonl').open('w') as out, (A / f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0
rows = [json.loads(line) for line in (A / f'{prefix}-build.jsonl').read_text().splitlines()]
artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] == 'gmp_api_coverage')
assert not artifact['fresh']
binary = A / f'{prefix}-test'
shutil.copy2(artifact['executable'], binary)
log = A / f'{prefix}-test.log'
with log.open('w') as out:
    code = subprocess.run([str(binary), '--test-threads=1', '--color', 'never'], cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=60).returncode
verify()
report.update(returncode=code, log=log.name, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), all_processes_reaped=True, all_sources_unchanged=True)
(A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2) + '\n')
print(log.read_text(), flush=True)
assert code == 0
