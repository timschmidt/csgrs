"""Verify and record the rejected default-layout inline storage experiment."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import statistics
import tarfile

root = Path(__file__).resolve().parent
candidate = root / 'normal-form-inline-candidate'
cost = root / 'normal-form-inline-cost'
read = lambda p: json.loads(p.read_text())
info = lambda p: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
source = read(candidate / 'sources.json')
baseline = read(root / 'normal-form-ownership-candidate/sources.json')
changes = [p for p, h in source['source_hashes'].items() if baseline['source_hashes'].get(p) != h]
assert changes == ['Cargo.lock', 'crates/hyperreal/Cargo.toml', 'crates/hyperreal/src/computable/node/algebra.rs']
assert source['unchanged']
with tarfile.open(candidate / 'sources.tar.gz') as archive:
    assert set(archive.getnames()) == set(source['source_hashes'])
    for name, expected in source['source_hashes'].items():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == expected
status = read(candidate / 'status.json')
assert len(status) == 33 and all(row['exit_code'] == 0 for row in status[:-1])
assert status[-1]['name'] == 'full-board-smoke' and status[-1]['exit_code'] == 101
board = (candidate / 'full-board-smoke.log').read_text()
worker_seconds = float(re.search(r'took ([\d.]+)s of worker time', board)[1])
assert worker_seconds > 120 and board.count('hyperdrc: finished emit report') == 2
assert 'skipping complex project fixture test' not in board
tests = {}
for path in candidate.glob('*.log'):
    matches = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', path.read_text())
    if matches:
        tests[path.name] = {key: sum(int(row[i]) for row in matches)
                            for i, key in enumerate(['passed', 'failed', 'ignored'])}
measurements = read(cost / 'manifest.json')
assert measurements['source_unchanged'] and measurements['source_change'] == changes
assert len(measurements['commands']) == 28 and all(r['exit_code'] == 0 for r in measurements['commands'])
assert measurements['probe_sha256'] == read(root / 'normal-form-ownership-cost-corrected/manifest.json')['probe_sha256']
totals = {}
for variant in ['baseline', 'candidate']:
    values = [sum(r['nanos'] for r in trial['rows'] if 'nanos' in r)
              for trial in measurements['trials'] if trial['mode'] == 'time' and trial['variant'] == variant]
    assert len(values) == 5
    totals[variant] = dict(samples_ns=values, median_ns=statistics.median(values))
for mode in ['time', 'memory']:
    trials = [trial for trial in measurements['trials'] if trial['mode'] == mode]
    witness = lambda t: [r for r in t['rows'] if 'witness' in r or 'fixture' in r]
    assert len(trials) == 10 and len(witness(trials[0])) == 24
    assert all(witness(t) == witness(trials[0]) for t in trials)
memory = [r for r in measurements['summaries'] if r['mode'] == 'memory']
assert len(memory) == 24
assert all(len(set(field['samples'])) == 1
           for row in memory for variant in row['variants'].values() for field in variant.values())
record = dict(
    decision='rejected',
    scope='Default SmallVec layout with one inline entry for each temporary Laurent-term, radical, and opaque-atom vector, layered over the retained ownership change and the provisional native-source candidate.',
    production_baseline_commit='73d162ed0',
    previous_diagnostic='diagnostics-17.json',
    contract='normal-form-inline-contract.json',
    source_change=changes,
    validation=dict(commands_passed_before_board=32, tests=tests,
                    passed_test_invocations=sum(r['passed'] for r in tests.values()),
                    ignored=sum(r['ignored'] for r in tests.values()),
                    source_archive_verified=True, worker_cpu_seconds=worker_seconds,
                    unchanged_limit_seconds=120,
                    original_board_report_finding_counts=[12, 197]),
    independent_oracle=dict(path='normal-form-ownership-oracle.json', identities=126,
                            scope='Reused unchanged mathematical identities; the public probe source hash is unchanged and verifies each expected rational answer outside the measured body.'),
    exact_probe_witnesses_equal=True,
    allocation_samples_repeat_identically=True,
    summed_timing_samples=totals,
    first_query_allocations=[r for r in memory if r['name'].endswith('_first')],
    binary_bytes={k: v['bytes'] for k, v in measurements['binaries'].items()},
    decision_reason='Allocation calls and bytes decrease for five nontrivial first-query families, but all corresponding first/warm/cloned timing medians regress. Summed median time rises from 153.272 to 165.239 ms. The full-board worker time remains essentially unchanged at 208.690337472 seconds versus the preceding 209.577392864 seconds. This memory improvement does not justify the observed higher-rung performance cost.',
    limitations='Five short paired fresh processes support rejection of this implementation; they do not establish statistical significance or a cross-library result. Full-board runs are single diagnostics, not a matched speedup. No complete JSON replay or isolated production admission was run for the rejected layout.',
    next_step='Try the smaller SmallVec union layout as a separate experiment. Gate it on scalar correctness and the same matched costs before repeating expensive downstream validation. If it does not improve the tradeoff, restore ordinary vectors and pursue the recovered geometry callers.',
)
record['artifacts'] = {str(p.relative_to(root)): info(p)
                       for directory in [root / 'normal-form-inline-preparation', candidate, cost]
                       for p in sorted(directory.rglob('*')) if p.is_file()}
for name in ['analyze_normal_form_inline.py', 'normal-form-inline-contract.json',
             'analyze_native_normal_form_callers.py', 'normal-form-caller-pairs.json']:
    record['artifacts'][name] = info(root / name)
(root / 'diagnostics-18.json').write_text(json.dumps(record, indent=2) + '\n')
print('default inline layout rejected; scalar correctness and matched evidence verified')
