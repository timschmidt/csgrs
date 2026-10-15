"""Close the native enclosure comparison from every raw exact endpoint and sample."""
from collections import Counter
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import re
import statistics

from close_v1 import E, ROOT, mapping, read, sha

OUT = E / 'competitive-qualification-v2'


def main():
    assert not (E / 'competitive-closure-v2.json').exists() and not (OUT / 'failure.json').exists()
    result, protocol = read(OUT / 'result.json'), read(OUT / 'protocol.json')
    commands = read(OUT / 'commands.json')
    assert result['complete'] and len(commands) == 13 and all(row['exit_code'] == 0 for row in commands)
    expected = ['format', 'fuzz-format', 'python-tests', 'serde-clippy', 'trace-clippy', 'all-features-clippy',
                'serde-tests', 'all-features-tests', 'native-build', 'native-symbols', 'comparison', 'comparison-replay', 'native-size']
    assert [row['name'] for row in commands] == expected
    for name, digest in result['artifacts_sha256'].items():
        assert sha(OUT / name) == digest
    for name, digest in protocol['inputs_sha256'].items():
        assert sha(E / name) == digest
    sources = mapping(E / 'competitive-source-v2.json.gz')
    source = Path(protocol['source'])
    assert {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()} == sources
    for version in range(1, 5):
        base = ROOT / f'target/directed-scalar-search-v{version}/source'
        assert all(sha(base / name) == digest for name, digest in mapping(E / f'source-hashes-v{version}.json.gz').items())
        for name, digest in read(E / f'closure-v{version}.json')['trial_artifacts_sha256'].items():
            assert sha(E / f'qualification-v{version}' / name) == digest
    assert all(sha(ROOT / name) == digest for name, digest in mapping(E / 'main-source-before.json.gz').items())
    binaries = read(OUT / 'binaries.json')
    assert all(sha(Path(row['path'])) == row['sha256'] for row in [*binaries.values(), *protocol['preserved_binaries']])
    assert 'Ran 33 tests' in (OUT / 'python-tests.stderr').read_text()
    for name in ['serde-tests', 'all-features-tests']:
        assert '3 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out' in (OUT / (name + '.stdout')).read_text()
    native_compile = [line for line in (OUT / 'native-build.stderr').read_text().splitlines()
                      if '--crate-name directed_scalar_bench ' in line]
    assert len(native_compile) == 1
    assert 'feature="serde"' in native_compile[0]
    assert 'feature="dispatch-trace"' not in native_compile[0] and 'sanitizer=address' not in native_compile[0]
    assert not re.search(r'\b__(?:a|l)san_', (OUT / 'native-symbols.stdout').read_text())
    comparison = OUT / 'comparison'
    completed, schedule = read(comparison / 'result.json'), read(comparison / 'schedule.json')
    assert completed['complete'] and completed['completed_samples'] == 1298
    assert completed['input_count'] == 59 and completed['samples_per_backend_per_input'] == 11
    for name, digest in completed['artifacts_sha256'].items():
        assert sha(comparison / name) == digest
    selected = read(E / 'competitive-inputs-v2.json')
    inputs = read(comparison / 'inputs.json')
    identity = lambda payload: hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    assert inputs == {identity(payload): payload for rows in selected.values() for payload in rows}
    assert len(inputs) == 59 and len(selected) == 22
    journal = [json.loads(line) for line in (comparison / 'samples.jsonl').read_text().splitlines()]
    assert len(journal) == len(schedule) == 1298
    samples, final_precision = {}, {'hyperreal': Counter(), 'mpfr': Counter()}
    target = Fraction(1, 2 ** 256)
    for plan, row in zip(schedule, journal, strict=True):
        assert all(row[key] == value for key, value in plan.items())
        process = read(comparison / row['process_file'])
        assert process['status'] == 'exited' and process['exit_code'] == 0
        assert (comparison / process['stderr']).read_bytes() == b''
        assert sha(comparison / process['stdout']) == row['response_sha256'] == process['stdout_sha256']
        response = read(comparison / row['response_file'])
        assert response['recipe']['input'] == inputs[row['id']]
        assert response['backend'] == row['backend'] and response['status'] == 'ok'
        assert response['elapsed_ns'] == row['elapsed_ns'] > 0
        assert not response['dispatch_trace_enabled'] and not response['debug_assertions']
        assert Fraction(response['maximum_absolute_width']) == target
        lo, hi = map(Fraction, response['oracle']['interval'])
        assert lo <= hi and (lo > 0 or hi < 0)
        steps = response['steps']
        assert 1 <= len(steps) <= 15
        assert [step['precision_bits'] for step in steps] == list(range(256, 2049, 128))[:len(steps)]
        for index, step in enumerate(steps):
            a, b = map(Fraction, step['interval'])
            assert a <= b and b >= lo and a <= hi
            assert Fraction(step['width']) == b - a
            assert (b - a <= target) == (index == len(steps) - 1)
        final_precision[row['backend']][steps[-1]['precision_bits']] += 1
        key = (row['id'], row['backend'])
        samples.setdefault(key, {})
        assert row['sample'] not in samples[key]
        samples[key][row['sample']] = row['elapsed_ns']
    assert len(samples) == 118 and all(set(values) == set(range(11)) for values in samples.values())
    summary = read(comparison / 'summary.json')
    assert len(summary) == 59 and {row['id'] for row in summary} == set(inputs)
    ratios, time_ranges = [], {'hyperreal': [], 'mpfr': []}
    for row in summary:
        assert row['input'] == inputs[row['id']]
        for backend in time_ranges:
            values = [samples[(row['id'], backend)][i] for i in range(11)]
            stats = row['backends'][backend]
            assert stats == {'raw_elapsed_ns': values, 'median_ns': statistics.median(values),
                             'minimum_ns': min(values), 'maximum_ns': max(values)}
            time_ranges[backend].append(stats['median_ns'])
        ratio = row['backends']['hyperreal']['median_ns'] / row['backends']['mpfr']['median_ns']
        assert row['hyperreal_over_mpfr_median_ratio'] == ratio
        ratios.append(ratio)
    logs = ''.join(p.read_text() for p in OUT.rglob('*') if p.suffix in ['.stdout', '.stderr'])
    assert all(marker not in logs for marker in [
        'ERROR: AddressSanitizer', 'ERROR: LeakSanitizer', 'SUMMARY: AddressSanitizer',
        'LeakSanitizer has encountered a fatal error', 'DEADLYSIGNAL'])
    driver = [json.loads(line) for line in (E / 'competitive-driver-v2.stdout').read_text().splitlines()]
    assert [row for row in driver if 'exit_code' in row] == commands
    assert driver[-1] == {'complete': True, 'result_sha256': sha(OUT / 'result.json')}
    assert (E / 'competitive-driver-v2.stderr').read_bytes() == b''
    report = {
        'status': 'closed-completed-native-enclosure-comparison', 'driver_terminal_exit_code_observed': 0,
        'qualification_result_sha256': sha(OUT / 'result.json'), 'commands_executed': 13,
        'python_tests_passed': 33, 'rust_adapter_tests_passed_each_configuration': 3,
        'rust_adapter_test_configurations': ['serde', 'all-features'],
        'representations': 22, 'inputs': 59, 'samples_per_backend_per_input': 11, 'native_samples': 1298,
        'all_final_selected_inputs_preserved': True, 'all_exact_endpoints_widths_and_samples_checked': True,
        'maximum_absolute_width': str(target), 'per_backend_final_precision_counts': final_precision,
        'hyperreal_lower_median_cases': sum(ratio < 1 for ratio in ratios),
        'hyperreal_higher_median_cases': sum(ratio > 1 for ratio in ratios),
        'hyperreal_over_mpfr_median_ratio_range': [min(ratios), max(ratios)],
        'per_backend_median_time_range_ns': {backend: [min(values), max(values)] for backend, values in time_ranges.items()},
        'results_interpretation': 'Native adapter enclosure costs on these exact selected inputs at the same absolute accuracy contract. Every slower result is retained; no whole-library ranking, symbolic-certificate comparison or warm-result claim.',
        'native_binary': binaries['native-build'], 'native_sanitizer_symbols_absent': True,
        'native_sections': (OUT / 'native-size.stdout').read_text(),
        'cpu': read(comparison / 'protocol.json')['cpu'], 'sanitizer_error_markers_observed': False,
        'raw_cpuinfo_trailing_whitespace_lines': sum(line.endswith((b' ', b'\t')) for line in (comparison / 'cpuinfo.txt').read_bytes().splitlines()),
        'trial_artifacts_sha256': {str(p.relative_to(OUT)): sha(p) for p in OUT.rglob('*') if p.is_file()},
        'driver_logs_sha256': {name: sha(E / name) for name in ['competitive-driver-v2.stdout', 'competitive-driver-v2.stderr']},
        'closure_sources_sha256': {name: sha(E / name) for name in ['close_v1.py', 'close_competitive_v2.py']},
        'limits': read(E / 'competitive-contract-v2.json')['limits'],
        'admitted': False, 'finite_scalar_interval_comparison_complete': True,
        'P38_complete': False, 'plan_complete': False,
        'closed_trial_retried_resumed_filtered_extended_or_pooled': False,
    }
    (E / 'competitive-closure-v2.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in ['status', 'native_samples', 'inputs', 'hyperreal_lower_median_cases',
                     'hyperreal_higher_median_cases', 'raw_cpuinfo_trailing_whitespace_lines']}))
    print(json.dumps({'closure_sha256': sha(E / 'competitive-closure-v2.json')}))


if __name__ == '__main__':
    main()
