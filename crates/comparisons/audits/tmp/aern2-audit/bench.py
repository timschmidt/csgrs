import json
import random
import statistics
import subprocess
from pathlib import Path

NAMES = ['aern2', 'hyper']
BINS = ['/tmp/aern2-audit/performance',
        '/tmp/boehm-reals-audit/target/release/aern_performance']
ROUNDS = 15
RNG = random.Random(20260904)

def run(which, op, mode, bits, count):
    return float(subprocess.check_output(
        ['taskset', '-c', '6', BINS[which], op, mode, str(bits), str(count)],
        text=True, timeout=60))

rows = {}
for mode in ['cold', 'warm']:
    for bits in [53, 200]:
        for op in ['sqrt', 'exp', 'sin']:
            counts = [max(1000, min(2_000_000, round(50_000_000 / statistics.median(
                run(i, op, mode, bits, 2000) for _ in range(5))))) for i in range(2)]
            pairs = []
            for _ in range(ROUNDS):
                a1 = run(0, op, mode, bits, counts[0])
                b1 = run(1, op, mode, bits, counts[1])
                b2 = run(1, op, mode, bits, counts[1])
                a2 = run(0, op, mode, bits, counts[0])
                pairs.append([(a1+a2)/2, (b1+b2)/2])
            ratios = [b/a for a,b in pairs]
            boots = sorted(statistics.median(RNG.choices(ratios, k=ROUNDS)) for _ in range(5000))
            name = f'{op}_{mode}_{bits}'
            row = {'ns': {n:statistics.median(p[i] for p in pairs) for i,n in enumerate(NAMES)},
                   'hyper_over_aern2':statistics.median(ratios),
                   'ci':[boots[125],boots[4874]],
                   'counts':counts, 'pairs':pairs}
            rows[name] = row
            print(name, {k:v for k,v in row.items() if k != 'pairs'}, flush=True)
            Path('/tmp/aern2-hyper-scalar-bench.json').write_text(json.dumps(rows, indent=2))
