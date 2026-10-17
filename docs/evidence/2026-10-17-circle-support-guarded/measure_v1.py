"""Preserve all original and large-coefficient exact cost diagnostics."""
import ast
from fractions import Fraction
import hashlib
import json
import os
import random
import re
import statistics
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import RUNTIME, W, dump, read, run, sha

phase = sys.argv[1]
assert phase in ['preflight', 'matched']
build = RUNTIME / 'build-v3'
compiled = read(build / 'result.json')
protocol = read(build / 'protocol.json')
corpus = RUNTIME / 'corpus-v2/result.json'
assert compiled['complete'] and compiled['output_closed'] and not compiled['command_failures']
assert read(corpus)['complete'] and read(corpus)['output_closed'] and not read(corpus)['command_failures']
assert read(corpus)['source_mirror_unchanged']
assert not read(corpus)['property_persistence_warnings']
all_features = RUNTIME / 'all-feature-library-v1/result.json'
assert read(all_features)['complete'] and read(all_features)['output_closed'] and not read(all_features)['command_failures']
assert read(all_features)['source_mirror_unchanged'] and not read(all_features)['property_persistence_warnings']
if phase == 'matched':
    preflight = read(RUNTIME / 'cost-preflight-v1/result.json')
    assert preflight['complete'] and preflight['output_closed'] and not preflight['failures']
    assert preflight['all_exact_rows_equal_and_certified']
output = RUNTIME / ('cost-' + phase + '-v1')
output.mkdir(exist_ok=False)
pins = {str(path): sha(path) for path in [Path(__file__), W / 'runner_v1.py', W / 'contract-v1.md',
                                       build / 'result.json', build / 'protocol.json', corpus, all_features]}
if phase == 'matched':
    path = RUNTIME / 'cost-preflight-v1/result.json'
    pins[str(path)] = sha(path)
pins |= {value['path']: value['sha256'] for value in compiled['binaries'].values()}
assert all(sha(path) == digest for path, digest in (pins | protocol['read_only_dependencies_sha256']).items())
cases = [('rational', count) for count in [1, 16, 128, 512, 2048, 8192]]
cases += [(mode, count) for mode in ['symbolic', 'mixed'] for count in [128, 512]]
cases += [(mode, 128) for mode in ['superrange', 'subrange']]
cases += [('shared-coefficient', bits) for bits in [64, 65_536, 1_048_576]]
seed, repetitions = 20261017, (1 if phase == 'preflight' else 7)
order = [(mode, count, repetition) for repetition in range(repetitions) for mode, count in cases]
random.Random(seed).shuffle(order)
dump(output / 'protocol.json', {
    'input_sha256': pins, 'build_protocol': str(build / 'protocol.json'), 'phase': phase,
    'cases': cases, 'repetitions': repetitions, 'seed': seed, 'case_order': order,
    'variant_order': 'baseline/candidate on even sequence index, reversed on odd',
    'initial_host_load': os.getloadavg(), 'policy': 'STRICT', 'required_certainty': 'Certified',
    'fresh_process_and_owned_batch_per_sample': True,
    'ordinary_case_timing': 'Preconstructed input; each phase includes private kernel queries, independent exact circle and 3-4-5 assertions, and complete ordered witness rows.',
    'coefficient_case_timing': 'Preconstructed concentric circles; one cold query then 20000 warm queries, complete Disjoint relations validated and materialized.',
    'timing_excludes': ['input construction', 'coefficient inventory', 'printing', 'teardown', 'process startup'],
    'process_resources_include_excluded_phase_work': True,
    'all_local_test_and_build_processes_terminal_before_measurement': True,
    'small_case_limitation': 'Single-query cold rows and other microsecond/sub-millisecond rows are diagnostic controls, not precise general speed factors.',
    'source_build': 'Matched direct rustc test harnesses with identical frozen dependencies, not distributed binaries.',
    'remaining_affected_feature_downstream_and_binding_qualification_open': True,
    'retention_or_end_to_end_performance_claimed': False,
})
commands, observations, failures, references = [], [], [], {}
stop = False
for sequence, (mode, count, repetition) in enumerate(order):
    for label in (['baseline', 'candidate'] if sequence % 2 == 0 else ['candidate', 'baseline']):
        name = f'{mode}-{count}-r{repetition}-{label}'
        binary = compiled['binaries'][label]
        assert sha(binary['path']) == binary['sha256']
        if mode == 'shared-coefficient':
            selection = 'curve_intersection::circle_support_coefficient_probe::shared_coefficient_lookup_cost'
            extra = {'CIRCLE_SHARED_BITS': str(count), 'CIRCLE_SHARED_REPEATS': '20000'}
        else:
            selection = 'curve_intersection::circle_support_cost_probe::support_lookup_cost'
            extra = {'CIRCLE_CACHE_COST_N': str(count), 'CIRCLE_CACHE_COST_MODE': mode}
        row = run(output, name, [binary['path'], selection, '--exact', '--ignored', '--nocapture', '--test-threads=1'],
                  output, extra | {'RUST_BACKTRACE': '0'})
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
            assert 'test result: ok. 1 passed; 0 failed;' in text
            if mode == 'shared-coefficient':
                assert 'CIRCLE_SHARED_CERTAINTY Certified' in text
                samples = [json.loads(value) for value in re.findall(r'CIRCLE_SHARED_SAMPLE (\{[^\n]*\})', text)]
                rows = [ast.literal_eval(value) for value in re.findall(r'CIRCLE_SHARED_ROWS (\[[^\n]*\])', text)]
                assert len(samples) == len(rows) == 2 and rows == [[0], [0] * 20000]
                assert [sample['phase'] for sample in samples] == ['cold', 'warm']
                assert [sample['queries'] for sample in samples] == [1, 20000]
                assert all(sample['coefficient_shift_bits'] == count and sample['numerator_bits'] == count + 1
                           and sample['entries'] == 1 for sample in samples)
            else:
                assert 'CIRCLE_CACHE_CERTAINTY Certified' in text
                samples = [json.loads(value) for value in re.findall(r'CIRCLE_CACHE_SAMPLE (\{[^\n]*\})', text)]
                rows = [json.loads(value) for value in re.findall(r'CIRCLE_CACHE_ROWS (\[[^\n]*\])', text)]
                assert len(samples) == len(rows) == 2 and [sample['phase'] for sample in samples] == ['cold', 'warm']
                assert rows[0] == rows[1] and len(rows[0]) == count
                assert all(sample['count'] == sample['entries'] == count and sample['mode'] == mode for sample in samples)
                scale = Fraction(1, 10**400) if mode == 'subrange' else Fraction(1)
                offset = Fraction(10**400) if mode == 'superrange' else Fraction(0)
                for n, exact in enumerate(rows[0]):
                    symbolic = mode == 'symbolic' or (mode == 'mixed' and n % 2 == 0)
                    assert exact[0] == ('pi' if symbolic else '0')
                    assert Fraction(exact[1]) == Fraction(exact[3]) == offset + (n * 32 + 3) * scale
                    assert {Fraction(exact[2]), Fraction(exact[4])} == {-4 * scale, 4 * scale}
            key = (mode, count)
            if key in references:
                assert references[key] == rows
            else:
                references[key] = rows
            for sample, exact in zip(samples, rows):
                encoded = json.dumps(exact, separators=(',', ':')).encode()
                observations.append(sample | {'case': mode, 'parameter': count, 'variant': label,
                    'repetition': repetition, 'command': name, 'certainty': 'Certified',
                    'completion': 'certified completion', 'ordered_rows_sha256': hashlib.sha256(encoded).hexdigest(),
                    'ordered_rows_bytes': len(encoded), 'host_load_after': os.getloadavg()})
        except Exception as error:
            failures.append({'name': name, 'reason': 'semantic-or-parser-check-failed', 'detail': repr(error)})
        dump(output / 'observations.json', observations)
    if stop:
        break
summary = []
for mode, count in cases:
    for temperature in ['cold', 'warm']:
        values = {label: [sample['seconds'] for sample in observations if sample['case'] == mode
                         and sample['parameter'] == count and sample['phase'] == temperature and sample['variant'] == label]
                  for label in ['baseline', 'candidate']}
        if all(values.values()):
            medians = {label: statistics.median(times) for label, times in values.items()}
            summary.append({'case': mode, 'parameter': count, 'phase': temperature, 'seconds': values,
                            'median_seconds': medians, 'candidate_over_baseline_median': medians['candidate'] / medians['baseline']})
assert all(sha(path) == digest for path, digest in (pins | protocol['read_only_dependencies_sha256']).items())
assert all(sha(Path(value['path']) / path) == digest
           for value in protocol['sources'].values() for path, digest in value['sha256'].items())
complete = not failures and len(observations) == len(order) * 4
dump(output / 'result.json', {'complete': True, 'output_closed': True, 'phase': phase,
     'process_samples': len(commands), 'expected_process_samples': len(order) * 2,
     'timed_observations': len(observations), 'expected_timed_observations': len(order) * 4,
     'all_exact_rows_equal_and_certified': complete, 'all_independent_oracles_passed': complete,
     'failures': failures, 'external_resource_stop': stop, 'comparisons': summary,
     'read_only_inputs_unchanged': True, 'unfavorable_samples_retained': True,
     'cache_admitted': False, 'end_to_end_performance_claimed': False})
print(json.dumps({'phase': phase, 'process_samples': len(commands), 'failures': failures,
                  'all_exact_rows_equal_and_certified': complete}), flush=True)
raise SystemExit(0 if complete else 101)
