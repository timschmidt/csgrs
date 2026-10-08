from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, time, tomllib

workspace = Path('/home/tim/Documents/GitHub/workspace')
audit = workspace / 'hypercurve-api-review-2026-09-12'
root = Path('/tmp/hypercurve-region-admission-qualification')
prefix = 'normalized-path-admission-csgrs'
settings = json.loads((audit / 'normalized-path-admission-full2-build-settings.json').read_text())
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
features = tomllib.loads((root / 'hypercircuit/Cargo.toml').read_text())['dependencies']['csgrs']['features']
command = [str(toolchain / 'cargo'), 'test', '--release', '--no-default-features', '--features', ','.join(features), '--locked', '--offline', '--lib', '--no-run', '--message-format=json']
start = time.monotonic()
with (audit / (prefix + '-build.log')).open('w') as err, (audit / (prefix + '-build.jsonl')).open('w') as out:
    code = subprocess.run(command, cwd=root / 'csgrs', env=dict(os.environ, **settings), stdout=out, stderr=err, timeout=900).returncode
record = dict(build_command=command, build_returncode=code, build_elapsed_seconds=time.monotonic()-start)
(audit / (prefix + '-qualification.json')).write_text(json.dumps(record, indent=2) + '\n')
print('build', code, record['build_elapsed_seconds'], flush=True)
assert code == 0
artifacts = [json.loads(line) for line in (audit / (prefix + '-build.jsonl')).read_text().splitlines()]
binary = Path(next(item['executable'] for item in artifacts if item.get('reason') == 'compiler-artifact' and item.get('executable') and item['target']['name'] == 'csgrs' and item['profile']['test']))
archive = audit / (prefix + '-libtest')
shutil.copy2(binary, archive)
record['binary_sha256'] = hashlib.sha256(archive.read_bytes()).hexdigest()
record['source_sha256'] = hashlib.sha256((root / 'csgrs/src/curve/native.rs').read_bytes()).hexdigest()
record['test_command'] = [str(archive), 'higher_order_curve_constructors_and_flat_triangulation_are_usable', '--test-threads=1', '--color', 'never']
start = time.monotonic()
with (audit / (prefix + '-test.log')).open('w') as output:
    record['test_returncode'] = subprocess.run(record['test_command'], cwd=root / 'csgrs', stdout=output, stderr=subprocess.STDOUT, timeout=120).returncode
record['test_elapsed_seconds'] = time.monotonic()-start
record['passed'] = re.findall(r'^test ([^\n]+?) \.\.\. ok$', (audit / (prefix + '-test.log')).read_text(), re.M)
(audit / (prefix + '-qualification.json')).write_text(json.dumps(record, indent=2) + '\n')
print('test', record['test_returncode'], record['passed'], record['test_elapsed_seconds'], flush=True)
assert record['test_returncode'] == 0 and len(record['passed']) == 1
assert (workspace / 'csgrs/src/curve/native.rs').read_bytes() == (root / 'csgrs/src/curve/native.rs').read_bytes()
