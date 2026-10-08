from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib, json, re, shutil, subprocess, time

audit = Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
root = Path('/tmp/hypercurve-region-admission-qualification')
prefix = 'normalized-path-admission-hypercircuit-candidate3'
assert (audit / (prefix + '-test-build.exit')).read_text().strip() == '0'
archive = audit / (prefix + '-binaries')
archive.mkdir()
jobs = []
for line in (audit / (prefix + '-test-build.jsonl')).read_text().splitlines():
    item = json.loads(line)
    if item.get('reason') == 'compiler-artifact' and item.get('executable') and item['profile']['test']:
        target = 'lib' if item['target']['kind'] == ['lib'] else item['target']['name']
        source = Path(item['executable'])
        binary = archive / source.name
        shutil.copy2(source, binary)
        jobs.append((target, binary))
assert len(jobs) == 5, jobs

def run(job):
    target, binary = job
    command = [str(binary), '--test-threads=2', '--color', 'never']
    if target == 'lib':
        command.append('materialize::tests::')
    log = audit / (prefix + '-' + target + '.log')
    start = time.monotonic()
    with log.open('w') as output:
        try:
            code = subprocess.run(command, cwd=root / 'hypercircuit', stdout=output, stderr=subprocess.STDOUT, timeout=300).returncode
        except subprocess.TimeoutExpired:
            code = 124
    text = log.read_text()
    statuses = re.findall(r'^test ([^\n]+?) \.\.\. (ok|FAILED|ignored[^\n]*)$', text, re.M)
    row = dict(target=target, command=command, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name, passed=[name for name, status in statuses if status == 'ok'], failed=[name for name, status in statuses if status == 'FAILED'], ignored=[name for name, status in statuses if status.startswith('ignored')])
    (audit / (prefix + '-' + target + '.json')).write_text(json.dumps(row, indent=2) + '\n')
    print(target, code, len(row['passed']), row['failed'], round(row['elapsed_seconds'], 2), flush=True)
    return row

rows = []
with ThreadPoolExecutor(max_workers=2) as executor:
    for result in as_completed([executor.submit(run, job) for job in jobs]):
        rows.append(result.result())
(audit / (prefix + '-tests.json')).write_text(json.dumps(rows, indent=2) + '\n')
print('TOTAL', sum(len(row['passed']) for row in rows), 'passed', flush=True)
assert all(row['returncode'] == 0 and row['passed'] for row in rows)
