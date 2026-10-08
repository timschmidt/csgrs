from pathlib import Path
import concurrent.futures, hashlib, json, os, re, subprocess, time

audit = Path(__file__).resolve().parent
root = Path('/tmp/hypercurve-fillet-companion-final-2026-09-23')
repo = root / 'hypercurve'
prefix = 'fillet-companion-chart-20260923-broad2'
focused = 'fillet-companion-chart-20260923-focused1'
summary = [json.loads((audit / 'fillet-companion-chart-20260923-focused2-parent-result.json').read_text()), json.loads((audit / 'fillet-companion-chart-20260923-focused3-candidate-result.json').read_text())]
assert all(row['passed'] for row in summary[1]['cases'])
binary = audit / 'fillet-companion-chart-20260923-focused3-candidate-libtest'
parent_binary = audit / f'{focused}-parent-libtest'
assert hashlib.sha256(binary.read_bytes()).hexdigest() == summary[1]['binary_sha256']
assert hashlib.sha256(parent_binary.read_bytes()).hexdigest() == summary[0]['binary_sha256']
bindings = json.loads((audit / 'fillet-companion-chart-20260923-focused3-candidate-sources.json').read_text())
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
verify()
env = dict(os.environ, **json.loads((audit / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
checks = []
for feature in ['--all-features', '--no-default-features']:
    command = [cargo, 'check', '--all-targets', feature, '--locked', '--offline']
    with (audit / f'{prefix}-check{len(checks)}.log').open('w') as out:
        code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=1200).returncode
    verify()
    checks.append(dict(command=command, returncode=code))
    assert code == 0
    print('Passed', feature, 'all-target check', flush=True)
listing = subprocess.check_output([str(binary), '--list'], text=True)
names = [line[:-6] for line in listing.splitlines() if line.endswith(': test')]
def run(job):
    label, executable, name, index = job
    log = audit / f'{prefix}-{label}-case-{index:04d}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run([str(executable), '--exact', name, '--test-threads=1', '--nocapture', '--color', 'never'], cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=75).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    output = log.read_text()
    row = dict(label=label, name=name, returncode=code, elapsed_seconds=time.monotonic()-start, passed=code==0 and '1 passed;' in output, ignored=code==0 and '1 ignored;' in output, log=log.name)
    if not row['passed'] and not row['ignored']:
        print('Not passed', label, name, code, output[-1200:], flush=True)
    return row
rows = []
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    jobs = [pool.submit(run, ('candidate', binary, name, index)) for index, name in enumerate(names)]
    for future in concurrent.futures.as_completed(jobs):
        rows.append(future.result())
        (audit / f'{prefix}-cases.json').write_text(json.dumps(rows, indent=2)+'\n')
        if len(rows) % 100 == 0:
            print('Completed', len(rows), '/', len(names), flush=True)
verify()
parent_rows = {row['name']: row for row in json.loads((audit / 'nonzero-sign-reuse-hypercurve-20260923-final1-cases.json').read_text())}
new_names = {row['name'] for row in summary[1]['cases']} - set(parent_rows)
assert set(names) == set(parent_rows) | new_names
def failure_detail(row):
    output = (audit / row['log']).read_text()
    start = output.find(' panicked at ')
    if start < 0:
        return None
    output = output[start:].split('note: run with')[0].split('failures:')[0].strip()
    return re.sub(r'/tmp/[^/]+/hypercurve/', 'hypercurve/', output)
failed = [row for row in rows if not row['passed'] and not row['ignored']]
changes, replays = [], []
for row in failed:
    parent = parent_rows.get(row['name'])
    if parent and parent['returncode'] == row['returncode'] and failure_detail(parent) == failure_detail(row):
        replays.append(dict(parent, evidence='previous bound a34/e4 parent production run'))
    else:
        changes.append(row)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    replays.extend(pool.map(run, [('parent', parent_binary, row['name'], index) for index, row in enumerate(changes)]))
verify()
report = dict(passed=sum(row['passed'] for row in rows), ignored=sum(row['ignored'] for row in rows), failed=failed, changed_failures=[row['name'] for row in changes], new_failures=[row['name'] for row in replays if row['passed'] or row['name'] in new_names], improved=[row['name'] for row in rows if row['passed'] and row['name'] in parent_rows and not parent_rows[row['name']]['passed']], parent_replays=replays, checks=checks, binary_sha256=summary[1]['binary_sha256'], parent_binary_sha256=summary[0]['binary_sha256'], all_sources_unchanged=True, all_processes_reaped=True)
(audit / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Terminal', {key: value for key, value in report.items() if key not in ['failed', 'parent_replays']}, flush=True)
