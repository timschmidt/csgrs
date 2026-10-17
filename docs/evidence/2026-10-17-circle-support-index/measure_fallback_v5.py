"""Preserve independent exact witnesses and every paired cost observation."""
from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path
import random
import re
import resource
import statistics
import subprocess
import sys
import time

W = Path(__file__).resolve().parent
phase = sys.argv[1]
assert phase == 'fallback'
assert json.loads((W / 'cost-preflight-v4/result.json').read_text())['all_ordered_exact_rows_equal']
O = W / ('cost-' + phase + '-v5')

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

built = read(W / 'cost-build-v4/result.json')
assert built['complete'] and built['command_failures'] == []
prerequisites = [W / 'cost-build-v4/result.json', W / 'cost-build-v4/protocol.json']
if phase == 'matched':
    q = W / 'candidate-qualification-v2/result.json'
    assert read(q)['complete'] and read(q)['command_failures'] == [] and read(q)['cargo_target_closed']
    p = W / 'cost-preflight-v4/result.json'
    assert read(p)['complete'] and read(p)['command_failures'] == [] and read(p)['all_ordered_exact_rows_equal']
    prerequisites += [q, p]
binaries = {label: Path(b['path']) for label, b in built['binaries'].items()}
assert all(sha(binaries[label]) == b['sha256'] for label, b in built['binaries'].items())
O.mkdir(exist_ok=False)
cases = [(mode, n) for mode in ['symbolic', 'mixed'] for n in [128, 512]]
repetitions = 7
seed = 20261017
order = [(mode, count, rep) for rep in range(repetitions) for mode, count in cases]
random.Random(seed).shuffle(order)
pins = {str(p): sha(p) for p in [Path(__file__), *prerequisites, *binaries.values()]}
dump(O / 'protocol.json', {
    'input_sha256': pins, 'phase': phase, 'scope': 'Repeated fallback diagnostic while full qualification is pending; not retention evidence by itself.', 'cases': cases, 'repetitions': repetitions, 'seed': seed,
    'case_order': order, 'variant_order': 'baseline/candidate for even sequence index; reversed for odd',
    'hardware': subprocess.check_output(['lscpu'], text=True), 'initial_loadavg': list(os.getloadavg()),
    'external_timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'policy': 'STRICT with Certified receipt required after both phases',
    'cache': 'Fresh process and fresh owned batch per sample. Cold inserts all N ordered pairs, then warm repeats those pairs in order.',
    'timing': 'Preconstructed exact inputs and input coefficient-bit inventory are excluded. Each phase includes actual private kernel lookups, independent exact circle/3-4-5 assertions and all ordered exact witness-row materialization. Printing, teardown and process startup are outside phase time; process resource records include them.',
    'exact_encoding': 'Each row is [x base (0 or pi), rational residual x1, rational y1, rational residual x2, rational y2]; strict comparison establishes every encoded coordinate.',
    'warmup': 'The separately retained one-sample preflight for every case precedes matched measurements; no samples are discarded from the matched run.',
    'small_case_limitation': 'Single-support and other microsecond rows are timer-scale diagnostic controls, not evidence of a stable speedup or regression.',
    'source_build': 'Matched direct rustc --test builds with identical frozen dependency libraries, not distributed package binaries.',
    'performance_claimed': phase == 'matched', 'end_to_end_speedup_claimed': False,
})
commands = []
observations = []
reference = {}

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

def run(name, command, extra=None):
    start = time.monotonic()
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        r = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')), 'timeout', '--kill-after=20s', '1800s', *command],
                           cwd=O, env=os.environ | {'TMPDIR': str(O), 'RUST_BACKTRACE': '0'} | (extra or {}),
                           stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    row = {'name': name, 'command': command, 'environment_overrides': extra or {}, 'exit_code': r.returncode,
           'elapsed_seconds': time.monotonic() - start, 'loadavg_after': list(os.getloadavg()),
           **{s + '_sha256': sha(O / (name + '.' + s)) for s in ['stdout', 'stderr', 'resources']}}
    commands.append(row)
    dump(O / 'commands.json', commands)
    return r.returncode

if phase == 'preflight':
    for label, binary in binaries.items():
        assert run(label + '-contracts', [str(binary), 'curve_intersection::circle_support_cache_tests::', '--nocapture', '--test-threads=1']) == 0
        assert '7 passed; 0 failed' in (O / (label + '-contracts.stdout')).read_text()

for i, (mode, count, rep) in enumerate(order):
    labels = ['baseline', 'candidate'] if i % 2 == 0 else ['candidate', 'baseline']
    for label in labels:
        name = f'{mode}-{count}-{rep}-{label}'
        assert run(name, [str(binaries[label]), 'curve_intersection::circle_support_cost_probe::support_lookup_cost',
                          '--exact', '--ignored', '--nocapture', '--test-threads=1'],
                   {'CIRCLE_CACHE_COST_N': str(count), 'CIRCLE_CACHE_COST_MODE': mode}) == 0
        output = (O / (name + '.stdout')).read_text()
        assert 'CIRCLE_CACHE_CERTAINTY Certified' in output and '1 passed; 0 failed' in output
        samples = [json.loads(s) for s in re.findall(r'CIRCLE_CACHE_SAMPLE (\{[^\n]*\})', output)]
        rows = [json.loads(s) for s in re.findall(r'CIRCLE_CACHE_ROWS (\[[^\n]*\])', output)]
        assert len(samples) == len(rows) == 2 and [s['phase'] for s in samples] == ['cold', 'warm']
        assert rows[0] == rows[1] and len(rows[0]) == count
        assert all(s['count'] == s['entries'] == count and s['mode'] == mode for s in samples)
        scale = Fraction(1, 10**400) if mode == 'subrange' else Fraction(1)
        offset = Fraction(10**400) if mode == 'superrange' else Fraction(0)
        for n, row in enumerate(rows[0]):
            symbolic = mode == 'symbolic' or (mode == 'mixed' and n % 2 == 0)
            assert row[0] == ('pi' if symbolic else '0')
            assert Fraction(row[1]) == Fraction(row[3]) == offset + (n * 32 + 3) * scale
            assert {Fraction(row[2]), Fraction(row[4])} == {-4 * scale, 4 * scale}
        case = (mode, count)
        if case in reference:
            assert reference[case] == rows[0]
        else:
            reference[case] = rows[0]
        encoded = json.dumps(rows[0], separators=(',', ':')).encode()
        for sample in samples:
            observations.append(sample | {'variant': label, 'repetition': rep, 'command': name,
                'certainty': 'Certified', 'ordered_rows_sha256': hashlib.sha256(encoded).hexdigest(), 'ordered_rows_bytes': len(encoded)})
    dump(O / 'observations.json', observations)
    print(json.dumps({'mode': mode, 'count': count, 'repetition': rep, 'both_completed_certified': True, 'ordered_rows_identical': True}), flush=True)

summary = []
for mode, count in cases:
    for temperature in ['cold', 'warm']:
        values = {label: [r['seconds'] for r in observations if r['mode'] == mode and r['count'] == count and r['phase'] == temperature and r['variant'] == label] for label in binaries}
        assert all(len(v) == repetitions for v in values.values())
        summary.append({'mode': mode, 'count': count, 'phase': temperature,
            'seconds': {label: {'median': statistics.median(v), 'min': min(v), 'max': max(v), 'samples': v} for label, v in values.items()},
            'candidate_over_baseline_median': statistics.median(values['candidate']) / statistics.median(values['baseline'])})
assert all(sha(p) == h for p, h in pins.items())
dump(O / 'result.json', {'complete': True, 'command_failures': [r['name'] for r in commands if r['exit_code']],
    'all_ordered_exact_rows_equal': True, 'all_independent_oracles_passed': True, 'all_receipts_certified': True,
    'timed_observations': len(observations), 'process_samples': len(order) * 2, 'comparisons': summary,
    'read_only_inputs_unchanged': True, 'performance_claimed': phase == 'matched', 'end_to_end_speedup_claimed': False,
    'native_runtime_admitted': False})
