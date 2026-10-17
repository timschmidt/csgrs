"""Matched complete-batch timing; each raw run and its exact rows are retained."""
import json
import random
import statistics
import sys
from fractions import Fraction
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import ROOT, RUNTIME, W, dump, read, run, sha

kind = sys.argv[1]
assert kind in ['preflight', 'matched']
build = RUNTIME / 'build-v1'
built, protocol = read(build / 'result.json'), read(build / 'protocol.json')
assert built['complete'] and built['output_closed'] and not built['command_failures']
if kind == 'matched':
    preflight = read(RUNTIME / 'cost-preflight-v1/result.json')
    assert preflight['complete'] and not preflight['command_failures']
    assert preflight['process_samples'] == 12
output = RUNTIME / ('cost-' + kind + '-v1')
output.mkdir(exist_ok=False)
repetitions = 1 if kind == 'preflight' else 7
pins = {str(path): sha(path) for path in [Path(__file__), W / 'runner_v1.py',
    W / 'contract-v1.md', build / 'result.json', build / 'protocol.json']}
pins |= {value['path']: value['sha256'] for value in built['binaries'].values()}
assert all(sha(path) == digest for path, digest in pins.items())
rng = random.Random(20261017)
schedule = []
for repetition in range(repetitions):
    cases = [1, 4, 8, 9, 16, 128]
    rng.shuffle(cases)
    for index, count in enumerate(cases):
        variants = ['baseline', 'candidate'] if (repetition + index) % 2 == 0 else ['candidate', 'baseline']
        schedule.extend((repetition, count, variant) for variant in variants)
dump(output / 'protocol.json', {'input_sha256': pins, 'schedule': schedule,
    'repetitions': repetitions, 'seed': 20261017,
    'production_bytes_unchanged_from_e42': True,
    'full_timing_contract': str(W / 'contract-v1.md'),
    'completion_required': 'Two independently checked strict secant witnesses per query; complete ordered rows every batch.',
    'measurements_sequential': True, 'no_instrumentation_in_latency': True})
commands, samples, witnesses = [], [], {}
for repetition, count, variant in schedule:
    name = f'n{count}-r{repetition}-{variant}'
    row = run(output, name, [built['binaries'][variant]['path'],
        'curve_intersection::circle_support_small_batch_probe::small_batch_complete_cost',
        '--exact', '--ignored', '--nocapture', '--test-threads=1'], output,
        {'CIRCLE_SMALL_BATCH_N': str(count)})
    commands.append(row)
    dump(output / 'commands.json', commands)
    if row['exit_code'] != 0:
        break
    text = (output / (name + '.stdout')).read_text()
    assert 'CIRCLE_SMALL_CERTAINTY Certified' in text
    assert 'test result: ok. 1 passed; 0 failed;' in text
    timing = [json.loads(line.split('CIRCLE_SMALL_SAMPLE ', 1)[1]) for line in text.splitlines()
              if 'CIRCLE_SMALL_SAMPLE ' in line]
    rows = [json.loads(line.removeprefix('CIRCLE_SMALL_ROWS ')) for line in text.splitlines()
            if line.startswith('CIRCLE_SMALL_ROWS ')]
    assert len(timing) == len(rows) == 2
    assert [value['phase'] for value in timing] == ['cold', 'warm']
    assert rows[0] == rows[1] and len(rows[0]) == count
    for n, coords in enumerate(rows[0]):
        x1, y1, x2, y2 = map(Fraction, coords)
        assert x1 == x2 == n * 32 + 3
        assert {y1, y2} == {Fraction(-4), Fraction(4)}
        for x, y in [(x1, y1), (x2, y2)]:
            assert all((x - center)**2 + y**2 == 25 for center in [n * 32, n * 32 + 6])
    if count in witnesses:
        assert witnesses[count] == rows[0]
    else:
        witnesses[count] = rows[0]
    for value in timing:
        assert value['count'] == count
        assert value['batches'] == (65536 + count - 1) // count
        assert value['queries'] == value['batches'] * count >= 65536
        samples.append(value | {'repetition': repetition, 'variant': variant, 'certainty': 'Certified',
                       'command_name': name, 'output_sha256': sha(output / (name + '.stdout'))})
    dump(output / 'samples.json', samples)
summary = []
for count in sorted(witnesses):
    for phase in ['cold', 'warm']:
        values = {variant: [v['seconds'] for v in samples
                   if v['count'] == count and v['phase'] == phase and v['variant'] == variant]
                  for variant in ['baseline', 'candidate']}
        if any(len(v) != repetitions for v in values.values()):
            continue
        baseline, candidate = values['baseline'], values['candidate']
        b, c = statistics.median(baseline), statistics.median(candidate)
        positive = sum(y > x for x, y in zip(baseline, candidate))
        nonoverlap = min(candidate) > max(baseline)
        summary.append({'count': count, 'phase': phase, 'baseline_seconds': baseline,
            'candidate_seconds': candidate, 'baseline_median_seconds': b,
            'candidate_median_seconds': c, 'candidate_over_baseline': c / b,
            'positive_paired_differences': positive,
            'candidate_min_exceeds_baseline_max': nonoverlap,
            'predeclared_rejection_criterion_met': kind == 'matched' and c / b > 1.10
                 and positive == 7 and nonoverlap})
assert all(sha(path) == digest for path, digest in pins.items())
failures = [row['name'] for row in commands if row['exit_code'] != 0]
dump(output / 'witnesses.json', witnesses)
dump(output / 'summary.json', summary)
dump(output / 'result.json', {'complete': True, 'output_closed': True,
    'process_samples': len(commands), 'expected_process_samples': len(schedule),
    'phase_observations': len(samples), 'command_failures': failures,
    'all_completed_observations_certified': True,
    'all_completed_exact_ordered_rows_identical': True,
    'independent_oracles_checked': True, 'read_only_inputs_unchanged': True,
    'decisive_rejection_cases': [row for row in summary if row['predeclared_rejection_criterion_met']],
    'summary_sha256': sha(output / 'summary.json'), 'cache_admitted': False,
    'full_plan_complete': False})
raise SystemExit(0 if len(commands) == len(schedule) and not failures else 101)
