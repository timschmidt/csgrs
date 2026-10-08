from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, time, tomllib

workspace = Path('/home/tim/Documents/GitHub/workspace')
audit = workspace / 'hypercurve-api-review-2026-09-12'
root = Path('/tmp/hypercurve-region-admission-qualification')
prefix = 'normalized-path-admission-csgrs-candidate3'
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
test_filter = 'curve::native::tests::'
listing = subprocess.check_output([str(archive), test_filter, '--list'], cwd=root / 'csgrs', text=True)
record['expected_tests'] = [line.removesuffix(': test') for line in listing.splitlines() if line.endswith(': test')]
assert len(record['expected_tests']) >= 14
command = [str(archive), test_filter, '--test-threads=2', '--color', 'never']
log = audit / (prefix + '-tests.log')
start = time.monotonic()
with log.open('w') as output:
    try:
        code = subprocess.run(command, cwd=root / 'csgrs', stdout=output, stderr=subprocess.STDOUT, timeout=300).returncode
    except subprocess.TimeoutExpired:
        code = 124
statuses = re.findall(r'^test ([^\n]+?) \.\.\. (ok|FAILED|ignored[^\n]*)$', log.read_text(), re.M)
record['test_command'] = command
record['test_returncode'] = code
record['test_elapsed_seconds'] = time.monotonic()-start
record['passed'] = [name for name, status in statuses if status == 'ok']
record['failed'] = [name for name, status in statuses if status == 'FAILED']
record['ignored'] = [name for name, status in statuses if status.startswith('ignored')]
(audit / (prefix + '-qualification.json')).write_text(json.dumps(record, indent=2) + '\n')
print('tests', code, len(record['passed']), record['failed'], record['test_elapsed_seconds'], flush=True)
assert code == 0 and sorted(record['passed']) == sorted(record['expected_tests'])
assert (workspace / 'csgrs/src/curve/native.rs').read_bytes() == (root / 'csgrs/src/curve/native.rs').read_bytes()
