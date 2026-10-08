from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, time, tomllib

workspace = Path('/home/tim/Documents/GitHub/workspace')
audit = workspace / 'hypercurve-api-review-2026-09-12'
root = Path('/tmp/hypercurve-region-admission-qualification')
prefix = 'normalized-path-admission-csgrs-candidate2'
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
record['queries'] = []
for name in ['higher_order_curve_constructors_and_flat_triangulation_are_usable', 'cycloidal_rack_alternates_face_and_root_flanks']:
    command = [str(archive), name, '--test-threads=1', '--color', 'never']
    log = audit / (prefix + '-' + name + '.log')
    start = time.monotonic()
    with log.open('w') as output:
        code = subprocess.run(command, cwd=root / 'csgrs', stdout=output, stderr=subprocess.STDOUT, timeout=120).returncode
    row = dict(command=command, returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name, passed=re.findall(r'^test ([^\n]+?) \.\.\. ok$', log.read_text(), re.M))
    record['queries'].append(row)
    (audit / (prefix + '-qualification.json')).write_text(json.dumps(record, indent=2) + '\n')
    print('test', name, code, row['passed'], row['elapsed_seconds'], flush=True)
assert all(row['returncode'] == 0 and len(row['passed']) == 1 for row in record['queries'])
assert (workspace / 'csgrs/src/curve/native.rs').read_bytes() == (root / 'csgrs/src/curve/native.rs').read_bytes()
