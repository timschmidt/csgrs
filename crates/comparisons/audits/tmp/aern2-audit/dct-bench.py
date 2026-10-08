import json
import random
import statistics
import subprocess
import sys

rng = random.Random(20260906)
rounds = 21
rows = {}
def run(method, precision, degree, count):
    return float(subprocess.check_output(
        ['taskset', '-c', '6', '/tmp/aern2-audit/dct-performance',
         'time', method, str(precision), str(degree), str(count)],
        text=True, timeout=60))

for precision in [53, 100]:
    for degree in [8, 16, 32]:
        validation = subprocess.check_output(
            ['/tmp/aern2-audit/dct-performance', 'validate', 'ref',
             str(precision), str(degree), '1'], text=True, timeout=60).strip()
        print('validate', precision, degree, validation, file=sys.stderr, flush=True)
        estimates = [statistics.median(run(method, precision, degree, 13) for _ in range(5))
                     for method in ['ref', 'fast']]
        count = 13 * max(1, round(100_000_000 / (13 * max(estimates))))
        pairs = []
        for _ in range(rounds):
            a1 = run('ref', precision, degree, count)
            b1 = run('fast', precision, degree, count)
            b2 = run('fast', precision, degree, count)
            a2 = run('ref', precision, degree, count)
            pairs.append([(a1+a2)/2, (b1+b2)/2])
        ratios = [b/a for a,b in pairs]
        boots = sorted(statistics.median(rng.choices(ratios, k=rounds)) for _ in range(5000))
        row = {'ns': [statistics.median(p[i] for p in pairs) for i in [0,1]],
               'fast_over_reference': statistics.median(ratios),
               'ci': [boots[125], boots[4874]], 'count': count, 'pairs': pairs,
               'validation': validation}
        key = f'p{precision}_n{degree}'
        rows[key] = row
        print(key, {k:v for k,v in row.items() if k not in ['pairs', 'validation']}, file=sys.stderr, flush=True)
print(json.dumps(rows, indent=2))
