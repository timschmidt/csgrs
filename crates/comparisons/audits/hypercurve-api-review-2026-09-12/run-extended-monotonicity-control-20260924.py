from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

A = Path(__file__).resolve().parent
R = Path('/tmp/hypercurve-extended-monotonicity-control-20260924')
prefix = 'extended-monotonicity-20260924-control'
bindings = json.loads((A / f'{prefix}-sources.json').read_text())
workspace = {n: hashlib.sha256((A.parent/n).read_bytes()).hexdigest() for n in bindings}
def verify():
    for n, h in bindings.items():
        assert hashlib.sha256((R/n).read_bytes()).hexdigest() == h, n
        assert hashlib.sha256((A.parent/n).read_bytes()).hexdigest() == workspace[n], n
verify()
env = dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
os.utime(R/'hypercurve/src/lib.rs', None)
command = [cargo, 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (A/f'{prefix}-build.jsonl').open('w') as out, (A/f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=R/'hypercurve', env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0, (A/f'{prefix}-build.log').read_text()[-3000:]
rows = [json.loads(line) for line in (A/f'{prefix}-build.jsonl').read_text().splitlines()]
artifact = next(r for r in rows if r.get('reason') == 'compiler-artifact' and r['target']['name'] == 'hypercurve' and r.get('executable'))
assert not artifact['fresh']
binary = A/f'{prefix}-libtest'
shutil.copy2(artifact['executable'], binary)
name = 'bezier_offset::conversion_tests::chord_parallel_monotonicity_covers_the_retained_parameter_range'
start = time.monotonic()
with (A/f'{prefix}-case.log').open('w') as out:
    try:
        code = subprocess.run([str(binary), '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], cwd=R/'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=75).returncode
    except subprocess.TimeoutExpired:
        code = 'timeout'
verify()
report = dict(command=command, name=name, returncode=code, elapsed_seconds=time.monotonic()-start, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), all_sources_unchanged=True, all_processes_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2), flush=True)
print((A/f'{prefix}-case.log').read_text()[-2500:], flush=True)
