from pathlib import Path
import hashlib, json, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
root = Path('/tmp/hypercurve-homogeneous-v3-2026-09-24')
prefix = 'homogeneous-composition-20260924-final-probe'
bindings = json.loads((A/'homogeneous-composition-20260924-v3-sources.json').read_text())
source = A/'homogeneous-composition-20260924-probe1.rs'
source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
assert source_hash == json.loads((A/'homogeneous-composition-20260924-probe1-terminal.json').read_text())['source_sha256']
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_hash
verify()
assert (A/'homogeneous-composition-20260924-v3-qualification-selection.json').exists()
rows = [json.loads(line) for line in (A/'homogeneous-composition-20260924-v3-qualification-build.jsonl').read_text().splitlines()]
artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hypercurve' and row['target']['kind'] == ['lib'] and not row.get('executable'))
assert not artifact['fresh']
library = Path(next(name for name in artifact['filenames'] if name.endswith('.rlib')))
binary = A/prefix
assert not binary.exists()
command = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc', '--edition=2024', '-O', str(source), '-L', f'dependency={library.parent}', '--extern', f'hypercurve={library}', '-o', str(binary)]
with (A/f'{prefix}-build.log').open('w') as out:
    code = subprocess.run(command, stdout=out, stderr=subprocess.STDOUT, timeout=120).returncode
assert code == 0
start = time.monotonic()
with (A/f'{prefix}.log').open('w') as out:
    try:
        code = subprocess.run([str(binary)], cwd=root/'hypercurve', stdout=out, stderr=subprocess.STDOUT, timeout=150).returncode
    except subprocess.TimeoutExpired:
        code = 'timeout'
verify()
report = dict(command=command, returncode=code, elapsed_seconds=time.monotonic()-start, limit_seconds=150, source_sha256=source_hash, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(), all_sources_unchanged=True, all_processes_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print(report, flush=True)
print((A/f'{prefix}.log').read_text()[-3500:], flush=True)
