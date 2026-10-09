"""Verify the scalar-only early rejection, including restored runtime sources."""
import hashlib
import json
from pathlib import Path
import re
import statistics
import tarfile

cost = Path(__file__).resolve().parent
root = cost.parent
candidate = root / 'normal-form-inline-union-candidate'
read = lambda p: json.loads(p.read_text())
info = lambda p: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
source = read(candidate / 'sources.json')
assert source['unchanged']
with tarfile.open(candidate / 'sources.tar.gz') as archive:
    assert set(archive.getnames()) == set(source['source_hashes'])
    assert all(hashlib.sha256(archive.extractfile(p).read()).hexdigest() == h
               for p, h in source['source_hashes'].items())
status = read(candidate / 'status.json')
assert len(status) == 5 and all(row['exit_code'] == 0 for row in status)
assert status[-1]['name'] == 'build-scalar-library'
tests = {}
for name in ['real-default-tests', 'real-all-features-tests']:
    rows = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored',
                      (candidate / (name + '.log')).read_text())
    assert rows and all(failed == ignored == '0' for _, failed, ignored in rows)
    tests[name] = sum(int(passed) for passed, _, _ in rows)
measurements = read(cost / 'manifest.json')
assert measurements['source_unchanged'] and not measurements['validation']['full_board_run']
assert len(measurements['commands']) == 28 and all(row['exit_code'] == 0 for row in measurements['commands'])
assert measurements['probe_sha256'] == read(root / 'normal-form-ownership-cost-corrected/manifest.json')['probe_sha256']
for mode in ['time', 'memory']:
    trials = [t for t in measurements['trials'] if t['mode'] == mode]
    witness = lambda t: [r for r in t['rows'] if 'witness' in r or 'fixture' in r]
    assert len(trials) == 10 and len(witness(trials[0])) == 24
    assert all(witness(t) == witness(trials[0]) for t in trials)
totals = {}
for variant in ['baseline', 'candidate']:
    values = [sum(r['nanos'] for r in t['rows'] if 'nanos' in r)
              for t in measurements['trials'] if t['mode'] == 'time' and t['variant'] == variant]
    totals[variant] = dict(samples_ns=values, median_ns=statistics.median(values))
memory = [r for r in measurements['summaries'] if r['mode'] == 'memory']
assert len(memory) == 24
assert all(len(set(field['samples'])) == 1
           for row in memory for variant in row['variants'].values() for field in variant.values())
record = dict(
    decision='rejected', previous_diagnostic='diagnostics-18.json',
    scope='Scalar-only early evaluation of one-entry SmallVec storage with union state encoding. No curve/DRC/full-board test or production admission was run for this variant.',
    contract='normal-form-inline-union-contract.json', production_baseline='73d162ed0',
    validation=dict(commands=5, tests=tests, source_archive_verified=True),
    exact_probe_witnesses_equal=True, allocation_samples_repeat_identically=True,
    summed_timing_samples=totals,
    first_query_allocations=[r for r in memory if r['name'].endswith('_first')],
    binary_bytes={k: v['bytes'] for k, v in measurements['binaries'].items()},
    decision_reason='Reduced metadata lowers requested allocation bytes, but summed median scalar time is 173.034 ms versus 155.473 ms for ordinary vectors. The five candidate totals exceed all five baseline totals. This measured performance cost rejects the memory tradeoff before downstream runs; no statistical significance or broad speedup is claimed.',
    failed_probe_build='The initial direct scalar-library probe omitted release/deps from its dependency lookup. The build failed before paired measurements. Both directories and the corrected shared driver are preserved.',
    restoration=read(candidate / 'restoration.json'),
    next_step='Ordinary vectors are restored. Inspect the recovered geometry consumers for repeated equivalent comparisons; keep exact equality, uncertainty, cache and policy behavior explicit.',
    remaining='The native-source candidate remains provisional and its original 120 worker CPU second gate remains failing. All 39 properties, 14 targets and crate future work remain in scope.',
)
record['artifacts'] = {str(p.relative_to(root)): info(p)
                       for directory in [candidate, root / 'normal-form-inline-union-cost', cost]
                       for p in sorted(directory.rglob('*')) if p.is_file()}
record['artifacts']['normal-form-inline-union-contract.json'] = info(root / 'normal-form-inline-union-contract.json')
(root / 'diagnostics-19.json').write_text(json.dumps(record, indent=2) + '\n')
print('compact inline layout rejected; restored source receipt and exact witnesses verified')
