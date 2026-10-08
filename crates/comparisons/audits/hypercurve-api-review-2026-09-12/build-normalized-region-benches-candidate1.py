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
prefix = 'normalized-region-benches-candidate1'
files = ['benches/editing.rs', 'benches/offset.rs', 'benches/comparative.rs', 'benches/curve_region_boolean_batch.rs']
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
runs = []

def verify():
    for base, rows in [(r, manifest), (w, working)]:
        for row in rows:
            assert hashlib.sha256((base/row['file']).read_bytes()).hexdigest() == row['sha256'], row['file']

try:
    for kind, args in [
        ('check', ['check', '--all-targets', '--no-default-features']),
        ('bench-build', ['bench', '--all-features', '--bench', 'editing', '--bench', 'offset', '--bench', 'comparative', '--bench', 'curve_region_boolean_batch', '--no-run', '--message-format=json']),
    ]:
        cmd = [cargo, *args, '--locked', '--offline']
        stem = prefix+'-'+kind
        start = time.monotonic()
        with (a/(stem+'.log')).open('w') as err, (a/(stem+'.jsonl')).open('w') as out:
            result = subprocess.run(cmd, cwd=r/'hypercurve', env=env, stdout=out if kind=='bench-build' else err, stderr=err, timeout=900)
        builds.append(dict(kind=kind, command=cmd, returncode=result.returncode, elapsed_seconds=time.monotonic()-start))
        (a/(prefix+'-builds.json')).write_text(json.dumps(builds, indent=2)+'\n')
        print(kind, result.returncode, round(builds[-1]['elapsed_seconds'], 2), flush=True)
        if result.returncode:
            print((a/(stem+'.log')).read_text()[-8000:], flush=True)
            raise SystemExit(1)
    binaries = {}
    for line in (a/(prefix+'-bench-build.jsonl')).read_text().splitlines():
        item = json.loads(line)
        if item.get('reason') == 'compiler-artifact' and item.get('executable'):
            path = Path(item['executable'])
            binary = archive/path.name
            shutil.copy2(path, binary)
            binaries[item['target']['name']] = binary
    cases = []
    for lane in [
        'curve_region_source_related_algebraic_chord_regularization_reuse',
        'curve_region_independent_field_algebraic_chord_boolean',
        'curve_path_noninjective_collinear_algebraic_chord_intersection',
        'curve_region_strict_interior_algebraic_chord_boolean',
        'curve_region_axis_algebraic_miter_offset',
        'curve_region_axis_algebraic_round_offset',
        'curve_region_axis_algebraic_repeated_miter_offset',
        'curve_region_axis_algebraic_neck_split',
    ]:
        cases.append(('editing', lane, dict(HYPERCURVE_EDIT_BENCH='corner-solver', HYPERCURVE_EDIT_ITERATIONS='1', HYPERCURVE_EDIT_CORNER_LANE=lane)))
    for fixture in ['analytic-squares', 'analytic-curved-cap']:
        for policy in ['strict', 'approximate-512']:
            cases.append(('curve_region_boolean_batch', fixture+'-'+policy, dict(HYPERCURVE_CURVE_REGION_BATCH_FIXTURE=fixture, HYPERCURVE_CURVE_REGION_BATCH_POLICY=policy, HYPERCURVE_CURVE_REGION_BATCH_ITERATIONS='1')))
    for group in ['curve-region-algebraic-partition', 'curve-region-cyclic-algebraic-partition']:
        cases.append(('offset', group, dict(HYPERCURVE_OFFSET_BENCH_GROUP=group)))
    cases.append(('comparative', 'algebraic-round-rectangle', dict(HYPERCURVE_COMPARE_SAMPLES='1', HYPERCURVE_COMPARE_ITERS='1', HYPERCURVE_COMPARE_GROUP='algebraic_round_offset/rectangle', HYPERCURVE_COMPARE_IMPL='hypercurve')))

    def run_case(item):
        target, name, variables = item
        binary = binaries[target]
        cmd = [str(binary)]
        logname = prefix+'-'+name+'.log'
        start = time.monotonic()
        with (a/logname).open('w') as log:
            try:
                result = subprocess.run(cmd, cwd=r/'hypercurve', env=dict(env, **variables), stdout=log, stderr=subprocess.STDOUT, timeout=180)
                returncode = result.returncode
            except subprocess.TimeoutExpired:
                returncode = 'timeout'
        row = dict(target=target, name=name, command=cmd, environment=variables, returncode=returncode, elapsed_seconds=time.monotonic()-start, sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), log=logname)
        print(name, returncode, round(row['elapsed_seconds'], 2), flush=True)
        print((a/logname).read_text()[-2500:], flush=True)
        return row
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(run_case, cases):
            runs.append(row)
            (a/(prefix+'-runs.json')).write_text(json.dumps(runs, indent=2)+'\n')
    if any(row['returncode'] != 0 for row in runs):
        raise SystemExit(1)
finally:
    verify()
    print('Bound isolated and working sources unchanged.', flush=True)
