"""Measure whole-process allocation/peak costs and verify every instrumented witness."""
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import subprocess
import time

W = Path(__file__).resolve().parent
O = W / 'heap-cost-v3'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

matched = W / 'cost-matched-v3'
assert read(matched / 'result.json')['all_ordered_exact_rows_equal']
built = read(W / 'cost-build-v3/result.json')
assert built['command_failures'] == []
binaries = {label: Path(b['path']) for label, b in built['binaries'].items()}
assert all(sha(binaries[label]) == b['sha256'] for label, b in built['binaries'].items())
O.mkdir(exist_ok=False)
cases = [('rational', 1), ('rational', 512), ('rational', 2048), ('symbolic', 128), ('mixed', 128), ('superrange', 128), ('subrange', 128)]
tools = [Path('/usr/bin/heaptrack'), Path('/usr/bin/heaptrack_print'), Path('/usr/bin/size')]
tools += [p for p in Path('/usr/lib64/heaptrack').rglob('*') if p.is_file()]
pins = {str(p): sha(p) for p in [Path(__file__), matched / 'result.json', *binaries.values(), *tools]}
dump(O / 'protocol.json', {'input_sha256': pins, 'cases': cases,
    'heaptrack_version': subprocess.check_output(['heaptrack', '--version'], text=True),
    'print_version': subprocess.check_output(['heaptrack_print', '--version'], text=True),
    'external_timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'scope': 'One whole-process allocation observation for each variant/case, including input construction, both phases, exact oracles, complete witness encoding, printing and cleanup. Instrumented time is excluded from latency evidence. Cache-owned heap bytes are not isolated and no internal memory bound is established. Process-end retention includes runtime and scalar state; it is not automatically a cache leak.',
    'size_scope': 'Matched diagnostic test executables with identical source-path remapping and dependencies; not a package size claim.'})
commands = []
measurements = []

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

def run(name, command, extra=None):
    start = time.monotonic()
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        r = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')), 'timeout', '--kill-after=20s', '1800s', *command],
                           cwd=O, env=os.environ | {'TMPDIR': str(O), 'DEBUGINFOD_URLS': ''} | (extra or {}),
                           stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    row = {'name': name, 'command': command, 'environment_overrides': extra or {}, 'exit_code': r.returncode,
           'elapsed_seconds': time.monotonic() - start,
           **{s + '_sha256': sha(O / (name + '.' + s)) for s in ['stdout', 'stderr', 'resources']}}
    commands.append(row)
    dump(O / 'commands.json', commands)
    return r.returncode

def witnesses(path):
    text = Path(path).read_text()
    assert 'CIRCLE_CACHE_CERTAINTY Certified' in text and '1 passed; 0 failed' in text
    return [json.loads(s) for s in re.findall(r'CIRCLE_CACHE_ROWS (\[[^\n]*\])', text)]

for mode, count in cases:
    expected = witnesses(matched / f'{mode}-{count}-0-baseline.stdout')
    assert expected[0] == expected[1]
    for label, binary in binaries.items():
        name = f'{mode}-{count}-{label}'
        assert run(name + '-record', ['heaptrack', '--record-only', '-o', str(O / (name + '-allocations')), str(binary),
                   'curve_intersection::circle_support_cost_probe::support_lookup_cost', '--exact', '--ignored', '--nocapture', '--test-threads=1'],
                   {'CIRCLE_CACHE_COST_N': str(count), 'CIRCLE_CACHE_COST_MODE': mode}) == 0
        assert witnesses(O / (name + '-record.stdout')) == expected
        traces = [p for p in O.glob(name + '-allocations*') if p.suffix in ['.gz', '.zst']]
        assert len(traces) == 1
        trace = traces[0]
        massif = O / (name + '.massif')
        assert run(name + '-print', ['heaptrack_print', '-f', str(trace), '-m', '0', '-n', '5', '-s', '2',
                   '-M', str(massif), '--disable-builtin-suppressions', '--disable-embedded-suppressions']) == 0
        printed = (O / (name + '-print.stdout')).read_text()
        allocations = int(re.search(r'calls to allocation functions: (\d+)', printed).group(1))
        peak = re.search(r'peak heap memory consumption: ([^\n]+)', printed).group(1)
        retained = re.search(r'total memory leaked: ([^\n]+)', printed).group(1)
        snapshots = [int(s) for s in re.findall(r'^mem_heap_B=(\d+)$', massif.read_text(), re.M)]
        row = {'mode': mode, 'count': count, 'variant': label, 'allocation_calls': allocations,
               'peak_heap_human_readable': peak, 'process_end_retention_human_readable': retained,
               'largest_massif_snapshot_heap_bytes': max(snapshots) if snapshots else None,
               'exact_instrumented_witnesses_equal_to_uninstrumented': True,
               'trace': {'path': str(trace), 'sha256': sha(trace), 'bytes': trace.stat().st_size},
               'massif_sha256': sha(massif)}
        measurements.append(row)
        dump(O / 'measurements.json', measurements)
        print(json.dumps({k: row[k] for k in ['mode', 'count', 'variant', 'allocation_calls', 'peak_heap_human_readable']}), flush=True)
sizes = {}
for label, binary in binaries.items():
    assert run(label + '-size', ['size', str(binary)]) == 0
    values = (O / (label + '-size.stdout')).read_text().splitlines()[1].split()
    sizes[label] = {'file_bytes': binary.stat().st_size, 'text_bytes': int(values[0]), 'data_bytes': int(values[1]), 'bss_bytes': int(values[2])}
assert all(sha(p) == h for p, h in pins.items())
dump(O / 'result.json', {'complete': True, 'command_failures': [], 'all_instrumented_witnesses_identical': True,
                       'read_only_inputs_unchanged': True, 'measurements': measurements, 'binary_sizes': sizes,
                       'instrumented_latencies_used': False, 'isolated_cache_heap_bytes_measured': False,
                       'native_runtime_admitted': False})
