from pathlib import Path
import hashlib, json, os, statistics, subprocess, time
root = Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
prefix = 'conic-exact-evaluation-final'
qualification = json.loads((root / (prefix + '-full-results.json')).read_text())
assert len(qualification) == 49
assert not any(row['new_failures'] or row['timed_out'] for row in qualification)
builds = json.loads((root / (prefix + '-bench-builds.json')).read_text())['builds']
records = []
for run in range(3):
    for build in (builds if run % 2 == 0 else list(reversed(builds))):
        binary = Path(build['binary'])
        assert hashlib.sha256(binary.read_bytes()).hexdigest() == build['binary_sha256']
        stem = root / f"{prefix}-bench-{build['label']}-{run}"
        with stem.with_suffix('.csv').open('w') as out, stem.with_suffix('.log').open('w') as err:
            try:
                code = subprocess.run([str(binary)], stdout=out, stderr=err, timeout=60).returncode
            except subprocess.TimeoutExpired:
                code = 124
        stem.with_suffix('.exit').write_text(str(code) + '\n')
        assert code == 0, stem
        for line in stem.with_suffix('.csv').read_text().splitlines():
            weights, parameter, refine, iterations, nanoseconds = line.split(',')
            records.append({'label': build['label'], 'run': run, 'weights': weights, 'parameter': parameter,
                            'refine': refine == 'true', 'iterations': int(iterations),
                            'nanoseconds_per_iteration': int(nanoseconds) / int(iterations)})
medians = []
for weights in ['unequal', 'unit_radical', 'unit_pi']:
    for parameter in ['rational', 'radical', 'pi_inverse']:
        for refine in [False, True]:
            values = {label: statistics.median(row['nanoseconds_per_iteration'] for row in records
                      if row['label'] == label and row['weights'] == weights and row['parameter'] == parameter
                      and row['refine'] == refine) for label in ['baseline', 'current']}
            medians.append({'weights': weights, 'parameter': parameter, 'refine': refine, **values,
                            'current_over_baseline': values['current'] / values['baseline']})
(root / (prefix + '-bench-results.json')).write_text(json.dumps({'method': 'Three unloaded alternating runs; 1000 constructions per case, separately with both coordinates certified at -128 bits; microbenchmark, not a whole-operation speedup claim.', 'runs': records, 'medians': medians}, indent=2) + '\n')
for row in medians:
    print(row['weights'], row['parameter'], 'refine' if row['refine'] else 'construct',
          'baseline/current us', round(row['baseline']/1000, 2), round(row['current']/1000, 2),
          'ratio', round(row['current_over_baseline'], 3), flush=True)
meta = json.loads((root / (prefix + '-probe-builds.json')).read_text())
env = os.environ.copy()
env.pop('PH_INSPECT_ONLY', None)
results = []
for build in meta['builds'][1:]:
    binary = root / build['binary']
    assert hashlib.sha256(binary.read_bytes()).hexdigest() == build['binary_sha256']
    start = time.monotonic()
    with binary.with_suffix('.log').open('w') as out:
        try:
            code = subprocess.run([str(binary)], stdout=out, stderr=subprocess.STDOUT, env=env, timeout=150).returncode
        except subprocess.TimeoutExpired:
            code = 124
    elapsed = time.monotonic() - start
    binary.with_suffix('.exit').write_text(str(code) + '\n')
    record = {'binary': build['binary'], 'binary_sha256': build['binary_sha256'], 'returncode': code,
              'elapsed_seconds': elapsed, 'log': binary.with_suffix('.log').name}
    results.append(record)
    print(json.dumps(record), flush=True)
    print(binary.with_suffix('.log').read_text()[-1600:], flush=True)
(root / (prefix + '-inward-probes.json')).write_text(json.dumps(results, indent=2) + '\n')
