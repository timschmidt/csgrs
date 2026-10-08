import json, random, statistics, subprocess, sys
from pathlib import Path

rng = random.Random(8604)
mode = sys.argv[1]
def run(which, case, count):
    if mode == 'guard':
        binary = '/tmp/hyper-float-denominator-baseline' if which == 'before' else '/tmp/boehm-reals-audit/target/release/float_performance'
        command = [binary, case, str(count)]
    else:
        command = ['/tmp/boehm-reals-audit/target/release/performance', which, case, str(count)]
    return float(subprocess.check_output(['taskset', '-c', '6'] + command, text=True))

names = ['before', 'after'] if mode == 'guard' else ['donor', 'hyper']
cases = ['third', 'word_dyadic', 'wide_normal', 'wide_fraction', 'subnormal', 'tiny_fraction', 'underflow'] if mode == 'guard' else ['clone', 'add_small', 'mul_small', 'cmp_small', 'same_denominator', 'cross_cancel', 'add_large', 'cmp_large']
rounds = 31 if mode == 'guard' else 15
if mode == 'cold':
    cases = [case + '_cold' for case in cases if case != 'clone']
rows = {}
for case in cases:
    counts = {name: max(20_000, min(2_000_000, round(50_000_000 / statistics.median(run(name, case, 5000) for _ in range(5))))) for name in names}
    pairs = []
    for _ in range(rounds):
        a, b = names
        first = run(a, case, counts[a]); second = run(b, case, counts[b])
        third = run(b, case, counts[b]); fourth = run(a, case, counts[a])
        pairs.append([(first + fourth) / 2, (second + third) / 2])
    ratios = [b/a for a,b in pairs]
    boot = sorted(statistics.median(rng.choices(ratios, k=rounds)) for _ in range(5000))
    row = {'ns': {name: statistics.median(p[i] for p in pairs) for i,name in enumerate(names)}, 'ratio': statistics.median(ratios), 'ci': [boot[125], boot[4874]], 'counts': counts, 'pairs': pairs}
    rows[case] = row
    print(case, {k:v for k,v in row.items() if k != 'pairs'}, flush=True)
    Path(f'/tmp/boehm-reals-{mode}-bench-final.json').write_text(json.dumps(rows, indent=2))
