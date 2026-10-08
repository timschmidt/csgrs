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
prefix = 'direct-generated-curves-candidate2'
files = ['src/curve.rs', 'tests/hypercurve_path_closure.rs', 'tests/hypercurve_curve_intersection.rs']
changed = subprocess.check_output(['git', 'diff', '--name-only'], cwd=w/'hypercurve', text=True).splitlines()
assert sorted(x for x in changed if x != 'src/bezier_offset.rs') == sorted(files), changed
working = []
for file in files:
    shutil.copy2(w/'hypercurve'/file, r/'hypercurve'/file)
for file in [*files, 'src/bezier_offset.rs']:
    working.append(dict(file='hypercurve/'+file, sha256=hashlib.sha256((w/'hypercurve'/file).read_bytes()).hexdigest()))
(a/(prefix+'-source.patch')).write_bytes(subprocess.check_output(['git', 'diff', '--', *files], cwd=w/'hypercurve'))
manifest = []
for repo in sorted(r.iterdir()):
    if repo.is_dir():
        for path in sorted(repo.rglob('*')):
            if path.is_file() and 'target' not in path.parts:
                manifest.append(dict(file=str(path.relative_to(r)), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(a/(prefix+'-isolated-sources.json')).write_text(json.dumps(manifest, indent=2)+'\n')
(a/(prefix+'-working-sources.json')).write_text(json.dumps(working, indent=2)+'\n')
settings = json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text())
env = dict(os.environ, **settings)
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
archive = a/(prefix+'-libraries')
archive.mkdir()
builds = []
tests = []

def verify():
    for base, rows in [(r, manifest), (w, working)]:
        for row in rows:
            assert hashlib.sha256((base/row['file']).read_bytes()).hexdigest() == row['sha256'], row['file']

try:
    for kind, args in [
        ('check', ['check', '--all-targets', '--no-default-features']),
        ('test-build', ['test', '--release', '--all-features', '--test', 'hypercurve_path_closure', '--test', 'hypercurve_curve_intersection', '--no-run', '--message-format=json']),
    ]:
        cmd = [cargo, *args, '--locked', '--offline']
        stem = prefix+'-'+kind
        start = time.monotonic()
        with (a/(stem+'.log')).open('w') as err, (a/(stem+'.jsonl')).open('w') as out:
            result = subprocess.run(cmd, cwd=r/'hypercurve', env=env, stdout=out if kind=='test-build' else err, stderr=err, timeout=900)
        builds.append(dict(kind=kind, command=cmd, returncode=result.returncode, elapsed_seconds=time.monotonic()-start))
        (a/(prefix+'-builds.json')).write_text(json.dumps(builds, indent=2)+'\n')
        print(kind, result.returncode, round(builds[-1]['elapsed_seconds'], 2), flush=True)
        if result.returncode:
            print((a/(stem+'.log')).read_text()[-8000:], flush=True)
            raise SystemExit(1)
    binaries = []
    for line in (a/(prefix+'-test-build.jsonl')).read_text().splitlines():
        item = json.loads(line)
        if item.get('reason') == 'compiler-artifact' and item.get('executable') and item['profile']['test']:
            path = Path(item['executable'])
            binary = archive/path.name
            shutil.copy2(path, binary)
            binaries.append((item['target']['name'], binary))
    def run_test(item):
        target, binary = item
        cmd = [str(binary), '--test-threads=2', '--color', 'never']
        logname = prefix+'-'+target+'-test.log'
        start = time.monotonic()
        with (a/logname).open('w') as log:
            try:
                result = subprocess.run(cmd, cwd=r/'hypercurve', stdout=log, stderr=subprocess.STDOUT, timeout=300)
                returncode = result.returncode
            except subprocess.TimeoutExpired:
                returncode = 'timeout'
        row = dict(target=target, command=cmd, returncode=returncode, elapsed_seconds=time.monotonic()-start, sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), log=logname)
        print(target, returncode, round(row['elapsed_seconds'], 2), flush=True)
        print((a/logname).read_text()[-3500:], flush=True)
        return row
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(run_test, binaries):
            tests.append(row)
            (a/(prefix+'-tests.json')).write_text(json.dumps(tests, indent=2)+'\n')
    if any(row['returncode'] != 0 for row in tests):
        raise SystemExit(1)
finally:
    verify()
    print('Bound isolated and working sources unchanged.', flush=True)
