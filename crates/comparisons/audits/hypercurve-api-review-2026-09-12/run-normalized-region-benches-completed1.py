from pathlib import Path
import concurrent.futures
import hashlib
import json
import os
import subprocess
import time

a = Path(__file__).resolve().parent
w = Path('/home/tim/Documents/GitHub/workspace')
r = Path('/tmp/hypercurve-region-admission-qualification')
prefix = 'normalized-region-benches-completed1'
previous = json.loads((a/'normalized-region-benches-candidate1-runs.json').read_text())
cases = []
for row in previous:
    if row['target'] == 'editing':
        row['environment']['HYPERCURVE_EDIT_BENCH'] = 'represented-bezier-corner'
        row['expected_output'] = row['name']+':'
        cases.append(row)
    elif row['target'] == 'comparative':
        for implementation in ['hypercurve_exact_round', 'hypercurve_exact_miter']:
            clone = json.loads(json.dumps(row))
            clone['name'] += '-'+implementation
            clone['environment']['HYPERCURVE_COMPARE_IMPL'] = implementation
            clone['expected_output'] = implementation+' '
            cases.append(clone)

def verify():
    for base, filename in [(w, 'normalized-region-benches-candidate1-working-sources.json'), (r, 'normalized-region-benches-candidate1-isolated-sources.json')]:
        for row in json.loads((a/filename).read_text()):
            assert hashlib.sha256((base/row['file']).read_bytes()).hexdigest() == row['sha256'], row['file']

def run_case(row):
    assert hashlib.sha256(Path(row['command'][0]).read_bytes()).hexdigest() == row['sha256']
    logname = prefix+'-'+row['name']+'.log'
    start = time.monotonic()
    with (a/logname).open('w') as log:
        try:
            result = subprocess.run(row['command'], cwd=r/'hypercurve', env=dict(os.environ, **row['environment']), stdout=log, stderr=subprocess.STDOUT, timeout=180)
            returncode = result.returncode
        except subprocess.TimeoutExpired:
            returncode = 'timeout'
    output = (a/logname).read_text()
    executed = row['expected_output'] in output
    row.update(returncode=returncode, elapsed_seconds=time.monotonic()-start, workload_executed=executed, log=logname)
    print(row['name'], returncode, 'executed='+str(executed), round(row['elapsed_seconds'], 2), flush=True)
    print(output[-3000:], flush=True)
    return row

verify()
runs = []
try:
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(run_case, cases):
            runs.append(row)
            (a/(prefix+'-runs.json')).write_text(json.dumps(runs, indent=2)+'\n')
finally:
    verify()
    print('Bound sources unchanged.', flush=True)
raise SystemExit(1 if any(row['returncode'] != 0 or not row['workload_executed'] for row in runs) else 0)
