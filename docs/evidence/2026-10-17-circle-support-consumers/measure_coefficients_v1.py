"""Retain every matched exact-coefficient reuse observation."""
import ast
import os
import random
import statistics
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from coefficient_runner_v1 import RUNTIME, W, dump, read, resources, run, sha

build = RUNTIME / 'build-v1'
result = read(build / 'result.json')
assert result['complete'] and result['output_closed'] and not result['command_failures']
assert set(result['binaries']) == {'baseline', 'candidate'}
assert read('/dev/shm/csgrs-circle-support-consumers-v1/consumer-validation-v1/result.json')['cargo_target_closed']
protocol = read(build / 'protocol.json')
pins = protocol['input_sha256'] | protocol['read_only_dependencies_sha256']
pins |= {str(path): sha(path) for path in [Path(__file__), build / 'protocol.json', build / 'result.json']}
pins |= {binary['path']: binary['sha256'] for binary in result['binaries'].values()}
assert all(sha(path) == digest for path, digest in pins.items())
assert all(sha(Path(value['path']) / path) == digest
           for value in protocol['sources'].values() for path, digest in value['sha256'].items())
output = RUNTIME / 'matched-v1'
output.mkdir(exist_ok=False)
seed, repetitions = 20261017, 7
order = []
randomizer = random.Random(seed)
for repetition in range(repetitions):
    cases = [64, 65_536, 1_048_576]
    randomizer.shuffle(cases)
    for bits in cases:
        variants = ['baseline', 'candidate'] if len(order) % 2 == 0 else ['candidate', 'baseline']
        order.append({'repetition': repetition, 'coefficient_shift_bits': bits, 'variants': variants})
dump(output / 'protocol.json', {
    'input_sha256': pins, 'build_protocol': str(build / 'protocol.json'), 'seed': seed,
    'repetitions': repetitions, 'order': order, 'warm_queries_per_process': 20_000,
    'policy': 'STRICT', 'expected_certainty': 'Certified', 'expected_complete_relation': 'Disjoint',
    'timing_boundary': 'lookup, exact relation check, full ordered relation-row materialization',
    'timing_excludes': ['input construction', 'coefficient inventory', 'printing', 'teardown', 'process startup'],
    'process_resources_include_excluded_phase_work': True,
    'cold_single_query_timings_are_diagnostic_only': True,
    'fresh_process_and_fresh_batch_per_sample': True,
    'all_repository_validation_and_diagnostic_builds_terminal': True,
    'initial_resources': resources(), 'initial_host_load': os.getloadavg(),
    'process_observation': subprocess.check_output(['ps', '-eo', 'pid,comm,args'], text=True),
    'same_exact_inputs_policy_outputs_and_materialization_for_both_variants': True,
    'end_to_end_or_distributed_binary_speed_claimed': False,
})
commands, samples, failures = [], [], []
stop = False
for match in order:
    outputs = {}
    for variant in match['variants']:
        bits, repetition = match['coefficient_shift_bits'], match['repetition']
        name = f'r{repetition}-b{bits}-{variant}'
        binary = result['binaries'][variant]
        assert sha(binary['path']) == binary['sha256']
        row = run(output, name, [binary['path'],
                  'curve_intersection::circle_support_cost_probe::shared_coefficient_lookup_cost',
                  '--exact', '--ignored', '--nocapture', '--test-threads=1'],
                  Path(protocol['sources'][variant]['path']),
                  {'CIRCLE_SHARED_BITS': str(bits), 'CIRCLE_SHARED_REPEATS': '20000'})
        commands.append(row)
        dump(output / 'commands.json', commands)
        if row['exit_code'] != 0:
            failures.append({'name': name, 'reason': 'command-failed', 'exit_code': row['exit_code']})
            if row['external_stop_reason']:
                stop = True
                break
            continue
        text = (output / (name + '.stdout')).read_text()
        try:
            phase_samples = [__import__('json').loads(line.split('CIRCLE_SHARED_SAMPLE ', 1)[1])
                             for line in text.splitlines() if 'CIRCLE_SHARED_SAMPLE ' in line]
            phase_rows = [ast.literal_eval(line.split('CIRCLE_SHARED_ROWS ', 1)[1])
                          for line in text.splitlines() if 'CIRCLE_SHARED_ROWS ' in line]
            assert 'CIRCLE_SHARED_CERTAINTY Certified' in text
            assert 'test result: ok. 1 passed; 0 failed;' in text
            assert len(phase_samples) == len(phase_rows) == 2
            for sample, exact_rows, phase, queries in zip(phase_samples, phase_rows, ['cold', 'warm'], [1, 20_000]):
                assert sample['phase'] == phase and sample['queries'] == queries
                assert sample['coefficient_shift_bits'] == bits and sample['numerator_bits'] == bits + 1
                assert sample['entries'] == 1 and exact_rows == [0] * queries
                assert sample['seconds'] >= 0
                samples.append({'name': name, 'variant': variant, 'repetition': repetition,
                                'certainty': 'Certified', 'completion': 'certified completion',
                                'complete_relation': 'Disjoint', 'host_load_after': os.getloadavg(), **sample})
            outputs[variant] = phase_rows
        except Exception as error:
            failures.append({'name': name, 'reason': 'semantic-or-parser-check-failed', 'detail': repr(error)})
        dump(output / 'samples.json', samples)
    if len(outputs) == 2:
        assert outputs['baseline'] == outputs['candidate']
    if stop:
        break
summary = []
for bits in [64, 65_536, 1_048_576]:
    for phase in ['cold', 'warm']:
        times = {variant: [sample['seconds'] for sample in samples if sample['variant'] == variant
                           and sample['coefficient_shift_bits'] == bits and sample['phase'] == phase]
                 for variant in ['baseline', 'candidate']}
        if all(times.values()):
            medians = {variant: statistics.median(values) for variant, values in times.items()}
            summary.append({'coefficient_shift_bits': bits, 'phase': phase, 'seconds': times,
                            'median_seconds': medians,
                            'candidate_to_baseline_median_ratio': medians['candidate'] / medians['baseline']
                            if medians['baseline'] else None})
assert all(sha(path) == digest for path, digest in pins.items())
assert all(sha(Path(value['path']) / path) == digest
           for value in protocol['sources'].values() for path, digest in value['sha256'].items())
dump(output / 'result.json', {
    'complete': True, 'output_closed': True, 'command_count': len(commands), 'expected_command_count': 42,
    'timed_observation_count': len(samples), 'expected_timed_observation_count': 84,
    'all_exact_rows_equal_and_certified': not failures and len(samples) == 84,
    'failures': failures, 'external_resource_stop': stop, 'summary': summary,
    'all_inputs_unchanged': True, 'unfavorable_samples_retained': True,
    'cache_admitted': False, 'internal_resource_bounds_claimed': False,
})
print(__import__('json').dumps({'commands': len(commands), 'failures': failures, 'summary': summary}), flush=True)
raise SystemExit(0 if not failures and len(samples) == 84 else 101)
