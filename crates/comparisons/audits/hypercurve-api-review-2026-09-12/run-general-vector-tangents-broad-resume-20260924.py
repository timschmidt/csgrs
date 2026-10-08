from pathlib import Path
import concurrent.futures, hashlib, json, os, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
root = Path('/tmp/hypercurve-general-vector-tangents-v1-20260924')
original = 'general-vector-tangents-20260924-v1-broad'
prefix = original + '-resumed'
bindings = json.loads((A / 'general-vector-tangents-20260924-v1-sources.json').read_text())
selection = json.loads((A / f'{original}-selection.json').read_text())
focused = json.loads((A / 'general-vector-tangents-20260924-v1-terminal.json').read_text())
artifacts = selection['binaries']
rows = json.loads((A / f'{original}-cases.json').read_text())
assert len(rows) == 11 and all(row['passed'] for row in rows)
assert len(selection['reused_focused']) == 8 and len(selection['isolated_jobs']) == 3
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
    for artifact in artifacts.values():
        assert hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest() == artifact['sha256']
verify()
assert artifacts['hypercurve']['sha256'] == focused['binary_sha256']
for row in rows:
    output = (A / row['log']).read_text()
    assert 'running 1 test' in output and '1 passed;' in output
completed = {(row['target'], row['name']) for row in rows}
assert len(completed) == len(rows)
jobs = selection['jobs']
remaining = [(index, job) for index, job in enumerate(jobs) if tuple(job) not in completed]
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
(A / f'{prefix}-selection.json').write_text(json.dumps(dict(selection, resumed_from=f'{original}-cases.json', reused_completed=rows), indent=2) + '\n')
print('Resume:', len(remaining), 'remaining cases from the same frozen binaries;', len(rows), 'completed receipts reused.', flush=True)
def run(job):
    index, (target, name) = job
    limit = 75 if target == 'hypercurve' else 180
    log = A / f'{prefix}-case-{index:04d}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run([artifacts[target]['path'], '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], cwd=root / 'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    output = log.read_text()
    assert 'running 1 test' in output
    return dict(target=target, name=name, returncode=code, passed=code == 0 and '1 passed;' in output, ignored=code == 0 and '1 ignored;' in output, elapsed_seconds=time.monotonic()-start, limit_seconds=limit, log=log.name)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    futures = [pool.submit(run, job) for job in remaining]
    for future in concurrent.futures.as_completed(futures):
        row = future.result()
        rows.append(row)
        (A / f'{prefix}-cases.json').write_text(json.dumps(rows, indent=2)+'\n')
        if not row['passed'] and not row['ignored']:
            print('Not passed:', row['target'], row['name'], row['returncode'], flush=True)
        if len(rows) % 100 == 0:
            print('Completed', len(rows), 'of', len(jobs), flush=True)
verify()
report = dict(reused_focused=selection['reused_focused'], isolated_jobs=selection['isolated_jobs'],
              resumed_from=f'{original}-cases.json', runner_error='list variable concurrent shadowed concurrent module before pool creation',
              original_driver_session=58957, original_driver_returncode=1, original_driver_reaped=True,
              remaining_workers=2, checks=focused['checks'], binaries=artifacts,
              attempted=len(rows), passed=sum(row['passed'] for row in rows), ignored=sum(row['ignored'] for row in rows),
              nonpasses=[row for row in rows if not row['passed'] and not row['ignored']],
              all_sources_unchanged=True, all_processes_reaped=True)
(A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Terminal:', {key: report[key] for key in ['attempted', 'passed', 'ignored', 'nonpasses']}, flush=True)
