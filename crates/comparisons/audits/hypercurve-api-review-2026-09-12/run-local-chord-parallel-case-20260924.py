from pathlib import Path
import hashlib, json, os, subprocess, sys, time
A = Path(__file__).resolve().parent
W = A.parent
version, suffix = sys.argv[1:]
source_prefix = f'local-chord-parallel-roots-20260924-{version}'
prefix = source_prefix + '-isolated-' + suffix
assert not (A / f'{prefix}-terminal.json').exists()
root = Path(f'/tmp/hypercurve-local-chord-parallel-roots-{version}-20260924')
manifest = json.loads((A / f'{source_prefix}-sources.json').read_text())
focused = json.loads((A / f'{source_prefix}-terminal.json').read_text())
assert focused['all_processes_reaped']
binary = A / f'{source_prefix}-libtest'
def verify():
    assert hashlib.sha256(binary.read_bytes()).hexdigest() == focused['binary_sha256']
    for name, sha in manifest.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
verify()
names = [line[:-6] for line in subprocess.check_output([str(binary), '--list'], text=True).splitlines() if line.endswith(': test')]
names = [name for name in names if name.rsplit('::', 1)[-1] == suffix]
assert len(names) == 1
command = [str(binary), '--exact', names[0], '--nocapture', '--test-threads=1']
start = time.monotonic()
log = A / f'{prefix}.log'
with log.open('w') as out:
    try:
        code = subprocess.run(command, cwd=root/'hypercurve', stdout=out, stderr=subprocess.STDOUT, timeout=75).returncode
    except subprocess.TimeoutExpired:
        code = 'timeout'
verify()
report = dict(command=command, returncode=code, elapsed_seconds=time.monotonic()-start, limit_seconds=75,
              source_manifest=f'{source_prefix}-sources.json', binary_sha256=focused['binary_sha256'],
              all_processes_reaped=True, all_sources_unchanged=True)
(A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print(code, log.read_text()[-20000:])
