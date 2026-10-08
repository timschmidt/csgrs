from pathlib import Path
import hashlib, json, subprocess, time, sys
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
attempt = sys.argv[1]
build = audit / ('finite-rational-pairs-attempt'+attempt+'-build')
assert build.with_suffix('.exit').read_text().strip() == '0'
items = []
for line in build.with_suffix('.log').read_text().splitlines():
    try: item = json.loads(line)
    except json.JSONDecodeError: continue
    if item.get('reason') == 'compiler-artifact' and item.get('executable') and item['target']['kind'] == ['lib']:
        items.append(item)
binary = Path(items[-1]['executable'])
results = []
for test in ['finite_rational_pairs_replay_all_roots_and_stationary_points', 'finite_rational_pairs_keep_nonlinear_correspondences_and_residual_contacts', 'finite_rational_pairs_keep_retracing_and_point_fibers', 'finite_rational_pairs_certify_poles_on_the_active_domain']:
    stem = 'finite-rational-pairs-attempt'+attempt+'-'+test
    command = [str(binary), test, '--nocapture', '--test-threads=1']
    start = time.monotonic()
    with (audit / (stem+'.log')).open('w') as out:
        try:
            result = subprocess.run(command, cwd=root/'hypercurve', stdout=out, stderr=subprocess.STDOUT, timeout=180)
            code = result.returncode
        except subprocess.TimeoutExpired: code = 124
    row = {'command':command,'returncode':code,'seconds':time.monotonic()-start,'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'log':stem+'.log'}
    results.append(row)
    print(test, code, round(row['seconds'],2), flush=True)
    if code: print((audit/(stem+'.log')).read_text()[-7000:],flush=True)
(audit / ('finite-rational-pairs-attempt'+attempt+'-focused.json')).write_text(json.dumps(results,indent=2)+'\n')
