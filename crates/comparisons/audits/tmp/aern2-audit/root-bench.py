import json
import random
import statistics
import subprocess
import sys

rng = random.Random(20260905)
rounds = 21
rows = {}
def run(stage, method, degree, count):
    return float(subprocess.check_output(
        ['taskset', '-c', '6', '/tmp/aern2-audit/root-performance',
         stage, method, str(degree), str(count)], text=True, timeout=60))

for degree in [8, 16, 32, 64]:
    validation = subprocess.check_output(
        ['/tmp/aern2-audit/root-performance', 'validate', 'map', str(degree), '1'],
        text=True, timeout=60).strip()
    print('validate', degree, validation, file=sys.stderr, flush=True)
    for stage in ['initial', 'split']:
        estimates = [statistics.median(run(stage, method, degree, 13) for _ in range(5))
                     for method in ['map', 'vector']]
        count = 13 * max(1, round(200_000_000 / (13 * max(estimates))))
        pairs = []
        for _ in range(rounds):
            a1 = run(stage, 'map', degree, count)
            b1 = run(stage, 'vector', degree, count)
            b2 = run(stage, 'vector', degree, count)
            a2 = run(stage, 'map', degree, count)
            pairs.append([(a1+a2)/2, (b1+b2)/2])
        ratios = [b/a for a,b in pairs]
        boots = sorted(statistics.median(rng.choices(ratios, k=rounds)) for _ in range(5000))
        row = {'ns': [statistics.median(p[i] for p in pairs) for i in [0,1]],
               'vector_over_map': statistics.median(ratios),
               'ci': [boots[125], boots[4874]], 'count': count, 'pairs': pairs}
        key = f'{stage}_d{degree}'
        rows[key] = row
        print(key, {k:v for k,v in row.items() if k != 'pairs'}, file=sys.stderr, flush=True)
print(json.dumps(rows, indent=2))
