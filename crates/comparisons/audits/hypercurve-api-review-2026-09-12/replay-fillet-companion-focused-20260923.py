from pathlib import Path
import hashlib, json, os, subprocess, sys, time

audit = Path(__file__).resolve().parent
label = sys.argv[1]
assert label in ['parent', 'candidate']
root = Path('/tmp/hypercurve-fillet-companion-' + ('parent' if label == 'parent' else 'clean') + '-2026-09-23')
build_prefix = 'fillet-companion-chart-20260923-focused1'
prefix = 'fillet-companion-chart-20260923-focused2'
binary = audit / f'{build_prefix}-{label}-libtest'
bindings = json.loads((audit / f'{build_prefix}-{label}-sources.json').read_text())
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
verify()
listing = subprocess.check_output([str(binary), '--list'], text=True)
available = {line[:-6] for line in listing.splitlines() if line.endswith(': test')}
suffixes = ['extended_fillet_circle_endpoints_obey_the_exact_radius_bound', 'extended_fillet_region_classifies_both_sides_of_its_companion']
names = []
for suffix in suffixes:
    matches = [name for name in available if name.endswith('::' + suffix)]
    assert len(matches) == 1
    names.extend(matches)
env = dict(os.environ, **json.loads((audit / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
rows = []
for index, name in enumerate(names):
    log = audit / f'{prefix}-{label}-case-{index}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run([str(binary), '--exact', name, '--nocapture', '--test-threads=1'], cwd=root / 'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=75).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    output = log.read_text()
    assert 'running 1 test' in output, output
    row = dict(name=name, returncode=code, passed=code==0 and '1 passed;' in output, elapsed_seconds=time.monotonic()-start, log=log.name)
    rows.append(row)
    print(label, row, output[-1800:], flush=True)
verify()
report = dict(label=label, cases=rows, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), sources_unchanged=True, all_processes_reaped=True)
(audit / f'{prefix}-{label}-result.json').write_text(json.dumps(report, indent=2)+'\n')
print('Terminal; exact regression cases reaped.', flush=True)
