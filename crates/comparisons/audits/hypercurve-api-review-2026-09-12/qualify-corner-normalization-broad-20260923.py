from pathlib import Path
import concurrent.futures
import hashlib
import json
import os
import re
import shutil
import subprocess
import time

a = Path(__file__).resolve().parent
r = Path('/tmp/hypercurve-closure-2026-09-23/hypercurve')
w = a.parent / 'hypercurve'
prefix = 'corner-normalization-20260923-fix6-broad'
binding = json.loads((a/'corner-normalization-20260923-fix6-sources.json').read_text())
focused = json.loads((a/'corner-normalization-20260923-fix6-runs.json').read_text())
assert len(focused) == 5 and all(row['returncode'] == 0 for row in focused)

def verify():
    for name, digest in binding['working'].items():
        assert hashlib.sha256((w/name).read_bytes()).hexdigest() == digest, name
    for row in binding['isolated']:
        assert hashlib.sha256((r.parent/row['file']).read_bytes()).hexdigest() == row['sha256'], row['file']

verify()
env = dict(os.environ, **json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
held_out = [
    'selected_parallel_normal_circle_intersects_genuinely_analytic_parallel_in_one_fiber',
    'independent_oblique_chord_pair_fillets_extend_on_infinite_supports',
    'selected_circle_and_analytic_parallel_extend_on_full_supports',
    'pair_native_boolean_algebraic_chord_corner_publishes_a_third_generation_fillet',
]

def library():
    binary = a/'corner-normalization-20260923-fix6-libtest'
    cmd = [str(binary), '--test-threads=4', '--color', 'never']
    for name in held_out:
        cmd += ['--skip', name]
    log = a/(prefix+'-library.log')
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(cmd, cwd=r, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=420).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    output = log.read_text()
    statuses = re.findall(r'^test (.+?) \.\.\. (ok|FAILED|ignored[^\n]*)$', output, re.M)
    row = dict(command=cmd, returncode=code, elapsed_seconds=time.monotonic()-start,
               binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), log=log.name,
               passed=[n for n,s in statuses if s == 'ok'], failed=[n for n,s in statuses if s == 'FAILED'],
               ignored=[n for n,s in statuses if s.startswith('ignored')], excluded=held_out,
               summary=re.findall(r'^test result:.*$', output, re.M))
    (a/(prefix+'-library.json')).write_text(json.dumps(row, indent=2)+'\n')
    verify()
    print('Library:', {k: len(row[k]) for k in ['passed','failed','ignored']}, 'exit', code,
          'seconds', round(row['elapsed_seconds'],2), 'failures', row['failed'], flush=True)
    return row

def integration():
    targets = ['hypercurve_curve_region_boolean', 'hypercurve_curve_region_promotion',
               'hypercurve_analytic_parallel_region', 'hypercurve_curve_region_stroke']
    cmd = [cargo,'test','--release','--all-features','--no-run','--message-format=json','--locked','--offline']
    for target in targets:
        cmd += ['--test',target]
    start = time.monotonic()
    with (a/(prefix+'-build.jsonl')).open('w') as out, (a/(prefix+'-build.log')).open('w') as err:
        code = subprocess.run(cmd, cwd=r, env=env, stdout=out, stderr=err, timeout=900).returncode
    print('Integration build:', code, round(time.monotonic()-start,2), flush=True)
    verify()
    assert code == 0, (a/(prefix+'-build.log')).read_text()[-6000:]
    jobs = []
    for line in (a/(prefix+'-build.jsonl')).read_text().splitlines():
        item = json.loads(line)
        if item.get('reason') != 'compiler-artifact' or not item.get('executable') or item['target']['name'] not in targets:
            continue
        assert not item['fresh'], item['target']['name']
        target = item['target']['name']
        binary = a/(prefix+'-'+target)
        shutil.copy2(item['executable'],binary)
        listing = subprocess.check_output([str(binary),'--list'], cwd=r, text=True)
        (a/(prefix+'-'+target+'-list.txt')).write_text(listing)
        for line in listing.splitlines():
            if line.endswith(': test'):
                jobs.append((target,binary,line.removesuffix(': test')))
    assert {j[0] for j in jobs} == set(targets)
    def run(job):
        target,binary,name = job
        cmd = [str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never']
        log = a/(prefix+'-'+target+'-'+name+'.log')
        start = time.monotonic()
        with log.open('w') as out:
            try:
                code = subprocess.run(cmd, cwd=r, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=75).returncode
            except subprocess.TimeoutExpired:
                code = 'timeout'
        output = log.read_text()
        row = dict(target=target,name=name,command=cmd,returncode=code,
                   elapsed_seconds=time.monotonic()-start,passed=code==0 and '1 passed;' in output,
                   ignored=code==0 and '1 ignored;' in output, log=log.name,
                   binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
        if not row['passed'] and not row['ignored']:
            print('Integration not passed:',target,name,code,round(row['elapsed_seconds'],2),flush=True)
            print(output[-1700:],flush=True)
        return row
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(run,jobs):
            results.append(row)
            (a/(prefix+'-integration.json')).write_text(json.dumps(results,indent=2)+'\n')
    verify()
    print('Integration:',len(results),'cases,',sum(row['passed'] for row in results),'passed,',sum(row['ignored'] for row in results),'ignored',flush=True)
    return results

with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    futures = [pool.submit(library), pool.submit(integration)]
    results = [f.result() for f in futures]
verify()
print('All bound working and isolated sources unchanged.',flush=True)
