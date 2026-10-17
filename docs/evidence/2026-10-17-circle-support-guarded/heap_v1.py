"""Measure complete-process allocations without treating instrumented time as latency."""
import ast
import json
import re
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import RUNTIME, W, dump, read, run, sha

built = read(RUNTIME / 'build-v3/result.json')
assert built['complete'] and built['output_closed'] and not built['command_failures']
matched = RUNTIME / 'cost-matched-v1'
assert read(matched / 'result.json')['all_exact_rows_equal_and_certified']
preflight = RUNTIME / 'cost-preflight-v1'
assert read(preflight / 'result.json')['all_exact_rows_equal_and_certified']
output = RUNTIME / 'heap-v1'
output.mkdir(exist_ok=False)
cases = [('rational', 1), ('rational', 512), ('rational', 2048), ('symbolic', 128),
         ('mixed', 128), ('superrange', 128), ('subrange', 128)]
cases += [('shared-coefficient', bits) for bits in [64, 65_536, 1_048_576]]
tools = [Path('/usr/bin/heaptrack'), Path('/usr/bin/heaptrack_print'), Path('/usr/bin/size')]
tools += [path for path in Path('/usr/lib64/heaptrack').rglob('*') if path.is_file()]
pins = {str(path): sha(path) for path in [Path(__file__), W / 'runner_v1.py', W / 'contract-v1.md',
                                       RUNTIME / 'build-v3/result.json', matched / 'result.json', preflight / 'result.json', *tools]}
pins |= {value['path']: value['sha256'] for value in built['binaries'].values()}
dump(output / 'protocol.json', {
    'input_sha256': pins, 'cases': cases,
    'heaptrack_version': subprocess.check_output(['heaptrack', '--version'], text=True),
    'print_version': subprocess.check_output(['heaptrack_print', '--version'], text=True),
    'scope': 'One complete-process observation per variant/case: input construction, cold/warm queries, independent exact oracles, complete result rows, printing and cleanup.',
    'instrumented_time_is_not_latency_evidence': True,
    'cache_owned_heap_not_isolated': True, 'internal_heap_bound_claimed': False,
    'process_end_retention_is_not_automatically_a_cache_leak': True,
    'size_scope': 'Matched no-feature diagnostic executables with identical relative source paths, flags and dependencies, not distributed package size.',
})


def witnesses(path, mode):
    text = path.read_text()
    prefix = 'CIRCLE_SHARED' if mode == 'shared-coefficient' else 'CIRCLE_CACHE'
    assert prefix + '_CERTAINTY Certified' in text and 'test result: ok. 1 passed; 0 failed;' in text
    rows = [ast.literal_eval(value) for value in re.findall(prefix + r'_ROWS (\[[^\n]*\])', text)]
    assert len(rows) == 2
    return rows


commands, measurements, failures, sizes = [], [], [], {}
stop = False
for mode, count in cases:
    expected = witnesses(preflight / f'{mode}-{count}-r0-baseline.stdout', mode)
    for label, binary in built['binaries'].items():
        assert sha(binary['path']) == binary['sha256']
        name = f'{mode}-{count}-{label}'
        if mode == 'shared-coefficient':
            selection = 'curve_intersection::circle_support_coefficient_probe::shared_coefficient_lookup_cost'
            extra = {'CIRCLE_SHARED_BITS': str(count), 'CIRCLE_SHARED_REPEATS': '20000'}
        else:
            selection = 'curve_intersection::circle_support_cost_probe::support_lookup_cost'
            extra = {'CIRCLE_CACHE_COST_N': str(count), 'CIRCLE_CACHE_COST_MODE': mode}
        row = run(output, name + '-record', ['heaptrack', '--record-only', '-o', str(output / (name + '-allocations')),
                  binary['path'], selection, '--exact', '--ignored', '--nocapture', '--test-threads=1'],
                  output, extra | {'DEBUGINFOD_URLS': '', 'RUST_BACKTRACE': '0'})
        commands.append(row)
        dump(output / 'commands.json', commands)
        if row['exit_code'] != 0:
            failures.append({'name': name, 'stage': 'record', 'exit_code': row['exit_code']})
            stop = bool(row['external_stop_reason'])
            if stop:
                break
            continue
        assert witnesses(output / (name + '-record.stdout'), mode) == expected
        traces = [path for path in output.glob(name + '-allocations*') if path.suffix in ['.gz', '.zst']]
        assert len(traces) == 1
        trace = traces[0]
        massif = output / (name + '.massif')
        row = run(output, name + '-print', ['heaptrack_print', '-f', str(trace), '-m', '0', '-n', '5', '-s', '2',
                  '-M', str(massif), '--disable-builtin-suppressions', '--disable-embedded-suppressions'],
                  output, {'DEBUGINFOD_URLS': ''})
        commands.append(row)
        dump(output / 'commands.json', commands)
        if row['exit_code'] != 0:
            failures.append({'name': name, 'stage': 'print', 'exit_code': row['exit_code']})
            stop = bool(row['external_stop_reason'])
            if stop:
                break
            continue
        printed = (output / (name + '-print.stdout')).read_text()
        allocations = int(re.search(r'calls to allocation functions: (\d+)', printed).group(1))
        peak = re.search(r'peak heap memory consumption: ([^\n]+)', printed).group(1)
        retained = re.search(r'total memory leaked: ([^\n]+)', printed).group(1)
        snapshots = [int(value) for value in re.findall(r'^mem_heap_B=(\d+)$', massif.read_text(), re.M)]
        measurements.append({'case': mode, 'parameter': count, 'variant': label, 'allocation_calls': allocations,
            'peak_heap_human_readable': peak, 'process_end_retention_human_readable': retained,
            'largest_massif_snapshot_heap_bytes': max(snapshots) if snapshots else None,
            'exact_instrumented_witnesses_equal_to_uninstrumented': True,
            'trace': {'path': str(trace), 'sha256': sha(trace), 'bytes': trace.stat().st_size},
            'massif_sha256': sha(massif)})
        dump(output / 'measurements.json', measurements)
        print(json.dumps({key: measurements[-1][key] for key in ['case', 'parameter', 'variant', 'allocation_calls', 'peak_heap_human_readable']}), flush=True)
    if stop:
        break
if not stop:
    for label, binary in built['binaries'].items():
        row = run(output, label + '-size', ['size', binary['path']], output)
        commands.append(row)
        dump(output / 'commands.json', commands)
        if row['exit_code'] != 0:
            failures.append({'name': label + '-size', 'stage': 'size', 'exit_code': row['exit_code']})
            continue
        values = (output / (label + '-size.stdout')).read_text().splitlines()[1].split()
        sizes[label] = {'file_bytes': Path(binary['path']).stat().st_size,
                       'text_bytes': int(values[0]), 'data_bytes': int(values[1]), 'bss_bytes': int(values[2])}
assert all(sha(path) == digest for path, digest in pins.items())
dump(output / 'result.json', {'complete': True, 'output_closed': True, 'failures': failures,
     'measurements': measurements, 'expected_measurements': len(cases) * 2, 'binary_sizes': sizes,
     'all_instrumented_witnesses_identical': not failures and len(measurements) == len(cases) * 2,
     'read_only_inputs_unchanged': True, 'instrumented_latencies_used': False,
     'isolated_cache_heap_bytes_measured': False, 'external_resource_stop': stop,
     'cache_admitted': False, 'full_plan_complete': False})
raise SystemExit(0 if not failures and len(measurements) == len(cases) * 2 and len(sizes) == 2 else 101)
