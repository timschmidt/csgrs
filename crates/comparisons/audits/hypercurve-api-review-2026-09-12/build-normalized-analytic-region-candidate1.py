from pathlib import Path
import concurrent.futures
import hashlib
import json
import os
import shutil
import subprocess
import time

w = Path('/home/tim/Documents/GitHub/workspace')
a = Path(__file__).resolve().parent
r = Path('/tmp/hypercurve-region-admission-qualification')
prefix = 'normalized-analytic-region-candidate1'
file = 'tests/hypercurve_analytic_parallel_region.rs'
changed = subprocess.check_output(['git', 'diff', '--name-only'], cwd=w/'hypercurve', text=True).splitlines()
assert sorted(x for x in changed if x != 'src/bezier_offset.rs') == [file], changed
working = [dict(file='hypercurve/'+name, sha256=hashlib.sha256((w/'hypercurve'/name).read_bytes()).hexdigest()) for name in [file, 'src/bezier_offset.rs']]
(a/(prefix+'-working-sources.json')).write_text(json.dumps(working, indent=2)+'\n')
(a/(prefix+'-source.patch')).write_bytes(subprocess.check_output(['git', 'diff', '--', file], cwd=w/'hypercurve'))
parent = subprocess.check_output(['git', 'show', 'HEAD:'+file], cwd=w/'hypercurve')
assert (r/'hypercurve'/file).read_bytes() == parent
(a/(prefix+'-parent-test.rs')).write_bytes(parent)
settings = json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text())
env = dict(os.environ, **settings)
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
archive = a/(prefix+'-libraries')
archive.mkdir()
builds = []
runs = []

def manifest():
    rows = []
    for repo in sorted(r.iterdir()):
        if repo.is_dir():
            for path in sorted(repo.rglob('*')):
                if path.is_file() and 'target' not in path.parts:
                    rows.append(dict(file=str(path.relative_to(r)), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    return rows

def verify(rows):
    for base, entries in [(r, rows), (w, working)]:
        for row in entries:
            assert hashlib.sha256((base/row['file']).read_bytes()).hexdigest() == row['sha256'], row['file']

def build(kind, args, rows):
    cmd = [cargo, *args, '--locked', '--offline']
    stem = prefix+'-'+kind
    start = time.monotonic()
    with (a/(stem+'.log')).open('w') as err, (a/(stem+'.jsonl')).open('w') as out:
        result = subprocess.run(cmd, cwd=r/'hypercurve', env=env, stdout=out if 'build' in kind else err, stderr=err, timeout=900)
    row = dict(kind=kind, command=cmd, returncode=result.returncode, elapsed_seconds=time.monotonic()-start)
    builds.append(row)
    (a/(prefix+'-builds.json')).write_text(json.dumps(builds, indent=2)+'\n')
    verify(rows)
    print(kind, result.returncode, round(row['elapsed_seconds'], 2), flush=True)
    if result.returncode:
        print((a/(stem+'.log')).read_text()[-8000:], flush=True)
        raise SystemExit(1)
    if 'build' not in kind:
        return None
    for line in (a/(stem+'.jsonl')).read_text().splitlines():
        item = json.loads(line)
        if item.get('reason') == 'compiler-artifact' and item.get('executable') and item['target']['name'] == 'hypercurve_analytic_parallel_region':
            binary = archive/kind
            shutil.copy2(item['executable'], binary)
            return binary
    raise AssertionError('missing analytic-region executable')

parent_manifest = manifest()
(a/(prefix+'-parent-isolated-sources.json')).write_text(json.dumps(parent_manifest, indent=2)+'\n')
args = ['test', '--release', '--all-features', '--test', 'hypercurve_analytic_parallel_region', '--no-run', '--message-format=json']
parent_binary = build('parent-build', args, parent_manifest)
shutil.copy2(w/'hypercurve'/file, r/'hypercurve'/file)
candidate_manifest = manifest()
(a/(prefix+'-isolated-sources.json')).write_text(json.dumps(candidate_manifest, indent=2)+'\n')
build('check', ['check', '--all-targets', '--no-default-features'], candidate_manifest)
candidate_binary = build('candidate-build', args, candidate_manifest)

def run_test(item):
    variant, binary, name = item
    cmd = [str(binary), '--exact', name, '--test-threads=1', '--nocapture', '--color', 'never']
    logname = prefix+'-'+variant+'-'+name+'.log'
    start = time.monotonic()
    with (a/logname).open('w') as log:
        try:
            result = subprocess.run(cmd, cwd=r/'hypercurve', stdout=log, stderr=subprocess.STDOUT, timeout=300)
            returncode = result.returncode
        except subprocess.TimeoutExpired:
            returncode = 'timeout'
    output = (a/logname).read_text()
    passed = returncode == 0 and '1 passed;' in output
    row = dict(variant=variant, name=name, command=cmd, returncode=returncode, passed=passed, elapsed_seconds=time.monotonic()-start, sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), log=logname)
    print(variant, name, returncode, round(row['elapsed_seconds'], 2), flush=True)
    if not passed:
        print(output[-4000:], flush=True)
    return row

listing = subprocess.check_output([str(candidate_binary), '--list'], cwd=r/'hypercurve', text=True)
names = [line.removesuffix(': test') for line in listing.splitlines() if line.endswith(': test')]
assert names
try:
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(run_test, [('candidate', candidate_binary, name) for name in names]):
            runs.append(row)
            (a/(prefix+'-runs.json')).write_text(json.dumps(runs, indent=2)+'\n')
    failed = [row['name'] for row in runs if not row['passed']]
    parent_listing = subprocess.check_output([str(parent_binary), '--list'], cwd=r/'hypercurve', text=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(run_test, [('parent', parent_binary, name) for name in failed if name+': test' in parent_listing]):
            runs.append(row)
            (a/(prefix+'-runs.json')).write_text(json.dumps(runs, indent=2)+'\n')
finally:
    verify(candidate_manifest)
    print('Bound isolated and working sources unchanged.', flush=True)
raise SystemExit(1 if failed else 0)
