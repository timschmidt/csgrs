from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

A = Path(__file__).resolve().parent
root = Path('/tmp/hypercurve-homogeneous-trace1-2026-09-24')
prefix = 'homogeneous-composition-20260924-trace1'
bindings = json.loads((A / f'{prefix}-sources.json').read_text())
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
verify()
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
os.utime(root / 'hypercurve/src/lib.rs', None)
command = [cargo, 'test', '--test', 'hypercurve_path_closure', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (A / f'{prefix}-build.jsonl').open('w') as out, (A / f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=root / 'hypercurve', env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0, (A / f'{prefix}-build.log').read_text()[-4000:]
rows = [json.loads(line) for line in (A / f'{prefix}-build.jsonl').read_text().splitlines()]
artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hypercurve_path_closure' and row.get('executable'))
assert not artifact['fresh']
binary = A / prefix
shutil.copy2(artifact['executable'], binary)
print('Diagnostic built; starting single composition case', flush=True)
start = time.monotonic()
with (A / f'{prefix}.log').open('w') as out:
    try:
        code = subprocess.run([str(binary), '--exact', 'homogeneous_boundary_closes_through_boolean_corners_and_offset', '--nocapture', '--test-threads=1'], stdout=out, stderr=subprocess.STDOUT, timeout=30).returncode
    except subprocess.TimeoutExpired:
        code = 'timeout'
verify()
report = dict(command=command, returncode=code, elapsed_seconds=time.monotonic()-start, limit_seconds=30, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), all_sources_unchanged=True, all_processes_reaped=True)
(A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report), flush=True)
print((A / f'{prefix}.log').read_text()[-16000:], flush=True)
