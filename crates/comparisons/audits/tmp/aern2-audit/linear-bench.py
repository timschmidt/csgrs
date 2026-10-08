import json
import random
import statistics
import subprocess
from pathlib import Path

ROUNDS = 21
RNG = random.Random(20260905)

def run(mode, bits, size, count):
    return float(subprocess.check_output(
        ['taskset', '-c', '6', '/tmp/aern2-audit/linear-performance',
         mode, str(bits), str(size), str(count)], text=True, timeout=60))

rows = {}
for bits in [53, 200]:
    for size in [2, 4, 8, 16]:
        print('validate', bits, size, subprocess.check_output(
            ['/tmp/aern2-audit/linear-performance','validate',str(bits),str(size),'1'],
            text=True,timeout=60).strip(),flush=True)
        # Every batch contains complete 13-input cycles, including tiny batches
        # at large dimensions, so both methods see exactly the same mixture.
        counts = [13 * max(1, min(1540, round(200_000_000 / (13 * statistics.median(
            run(mode, bits, size, 13) for _ in range(5)))))) for mode in ['plain','via']]
        counts = [max(counts), max(counts)]
        pairs = []
        for _ in range(ROUNDS):
            a1 = run('plain', bits, size, counts[0])
            b1 = run('via', bits, size, counts[1])
            b2 = run('via', bits, size, counts[1])
            a2 = run('plain', bits, size, counts[0])
            pairs.append([(a1+a2)/2, (b1+b2)/2])
        ratios = [b/a for a,b in pairs]
        boots = sorted(statistics.median(RNG.choices(ratios,k=ROUNDS)) for _ in range(5000))
        row = {'ns': [statistics.median(p[i] for p in pairs) for i in range(2)],
               'via_over_plain': statistics.median(ratios), 'ci': [boots[125],boots[4874]],
               'counts': counts, 'pairs':pairs}
        key = f'mul_p{bits}_n{size}'
        rows[key] = row
        print(key, {k:v for k,v in row.items() if k != 'pairs'},flush=True)
        Path('/tmp/aern2-linear-bench-final.json').write_text(json.dumps(rows,indent=2))
