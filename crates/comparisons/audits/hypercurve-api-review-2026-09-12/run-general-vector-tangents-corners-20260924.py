from pathlib import Path
import hashlib, json, os, subprocess, time
A = Path(__file__).resolve().parent
W = A.parent
root = Path('/tmp/hypercurve-general-vector-tangents-v1-20260924')
prefix = 'general-vector-tangents-20260924-v1-corners'
manifest = json.loads((A / 'general-vector-tangents-20260924-v1-sources.json').read_text())
broad = json.loads((A / 'general-vector-tangents-20260924-v1-broad-resumed-terminal.json').read_text())
selection = json.loads((A / 'general-vector-tangents-20260924-v1-broad-selection.json').read_text())
assert broad['all_processes_reaped'] and broad['all_sources_unchanged']
binary = broad['binaries']['hypercurve']
def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
verify()
done = {name for target, name in selection['jobs'] if target == 'hypercurve'}
names = [row[:-6] for row in subprocess.check_output([binary['path'], '--list'], text=True).splitlines() if row.endswith(': test')]
jobs = [name for name in names if name not in done and name.startswith(('bezier_region::', 'curve::', 'curve_region_trim::')) and any(key in name for key in ['corner', 'endpoint_image'])]
assert len(jobs) == 12
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
rows = []
for index, name in enumerate(jobs):
    log = A / f'{prefix}-{index:02d}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run([binary['path'], '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], cwd=root / 'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=75).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    output = log.read_text()
    assert 'running 1 test' in output
    rows.append(dict(target='hypercurve', name=name, returncode=code, passed=code == 0 and '1 passed;' in output, ignored=code == 0 and '1 ignored;' in output, elapsed_seconds=time.monotonic()-start, limit_seconds=75, log=log.name))
    print(name, code, flush=True)
verify()
report = dict(cases=rows, jobs=[['hypercurve', name] for name in jobs], binary=binary, all_sources_unchanged=True, all_processes_reaped=True)
(A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Terminal; twelve supplementary corner cases reaped.', flush=True)
