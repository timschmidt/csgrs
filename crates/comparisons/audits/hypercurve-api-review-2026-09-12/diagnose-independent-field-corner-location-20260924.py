from pathlib import Path
import hashlib, json, os, shutil, subprocess, time
A = Path(__file__).resolve().parent
W = A.parent
prefix = 'independent-field-corners-20260924-v4'
root = Path('/tmp/hypercurve-independent-field-corners-v4-20260924')
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
verify()
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
env['HYPERCURVE_DEBUG_CORNER_LOCATION'] = '1'
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
command = [cargo, 'test', '--lib', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (A / f'{prefix}-debug-build.jsonl').open('w') as out, (A / f'{prefix}-debug-build.log').open('w') as err:
    code = subprocess.run(command, cwd=root / 'hypercurve', env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0
artifacts = [json.loads(line) for line in (A / f'{prefix}-debug-build.jsonl').read_text().splitlines()]
artifact = next(row for row in artifacts if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hypercurve' and row.get('executable'))
assert not artifact['fresh']
binary = A / f'{prefix}-debug-libtest'
shutil.copy2(artifact['executable'], binary)
sha = hashlib.sha256(binary.read_bytes()).hexdigest()
test_command = [str(binary), '--exact', 'bezier_region::tests::independent_field_corner_edits_preserve_normalized_sets', '--nocapture', '--test-threads=1', '--color', 'never']
start = time.monotonic()
log = A / f'{prefix}-location.log'
with log.open('w') as out:
    try:
        code = subprocess.run(test_command, cwd=root / 'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=75).returncode
    except subprocess.TimeoutExpired:
        code = 'timeout'
verify()
assert hashlib.sha256(binary.read_bytes()).hexdigest() == sha
report = dict(build=command, command=test_command, returncode=code, elapsed_seconds=time.monotonic()-start,
              diagnostic_only=True, binary_sha256=sha, all_sources_unchanged=True, all_processes_reaped=True)
(A / f'{prefix}-diagnostic-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print(log.read_text()[-6500:], flush=True)
