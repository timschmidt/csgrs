"""Verify a frozen point equality trial and record its scoped decision."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import statistics
import sys
import tarfile

root = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--validation', required=True)
parser.add_argument('--cost', required=True)
parser.add_argument('--diagnostic', required=True)
parser.add_argument('--previous', required=True)
parser.add_argument('--decision', required=True)
parser.add_argument('--reason', required=True)
parser.add_argument('--next-step', required=True)
args = parser.parse_args()
read = lambda p: json.loads(p.read_text())
info = lambda p: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
candidate, cost = root / args.validation, root / args.cost
source = read(candidate / 'sources.json')
baseline = read(root / 'normal-form-ownership-candidate/sources.json')
changes = [p for p, h in source['source_hashes'].items() if baseline['source_hashes'].get(p) != h]
assert changes == ['crates/hypercurve/src/point/equality_tests.rs', 'crates/hypercurve/src/point.rs']
assert source['unchanged']
with tarfile.open(candidate / 'sources.tar.gz') as archive:
    assert set(archive.getnames()) == set(source['source_hashes'])
    for name, expected in source['source_hashes'].items():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == expected
status = read(candidate / 'status.json')
assert len(status) == 5 and status[-1]['name'] == 'build-drc-library'
assert all(row['exit_code'] == 0 for row in status)
tests = {}
for phase in ['point-tests', 'public-curve-point']:
    result = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', (candidate / (phase + '.log')).read_text())
    assert len(result) == 1 and result[0][1:] == ('0', '0')
    tests[phase] = int(result[0][0])
measurements = read(cost / 'manifest.json')
assert measurements['source_unchanged'] and measurements['source_change'] == changes
assert len(measurements['commands']) == 28 and all(r['exit_code'] == 0 for r in measurements['commands'])
assert measurements['features']['baseline'] == measurements['features']['candidate']
assert measurements['probe_sha256'] == info(root / 'point_equality_probe.rs')['sha256']
for mode in ['time', 'memory']:
    trials = [t for t in measurements['trials'] if t['mode'] == mode]
    witness = lambda t: [r for r in t['rows'] if 'witness' in r or 'fixture' in r]
    assert len(trials) == 10 and len(witness(trials[0])) == 56
    assert all(witness(t) == witness(trials[0]) for t in trials)
    for t in trials:
        raw = cost / f"{mode}-{t['index']}-{t['variant']}.jsonl"
        assert [json.loads(line) for line in raw.read_text().splitlines()] == t['rows']
memory = [r for r in measurements['summaries'] if r['mode'] == 'memory']
assert len(memory) == 56
repeatable = all(len(set(field['samples'])) == 1
                 for row in memory for variant in row['variants'].values() for field in variant.values())
totals = {}
for category in ['query', 'construction', 'prepare']:
    totals[category] = {}
    for variant in ['baseline', 'candidate']:
        def selected(row):
            if category == 'query':
                return not row['name'].endswith(('_construction', '_prepare'))
            return row['name'].endswith('_' + category)
        values = [sum(r['nanos'] for r in t['rows'] if 'nanos' in r and selected(r))
                  for t in measurements['trials'] if t['mode'] == 'time' and t['variant'] == variant]
        assert len(values) == 5
        totals[category][variant] = dict(samples_ns=values, median_ns=statistics.median(values))
record = dict(
    decision=args.decision,
    scope='Early public Point2 equality gate on the provisional native-source candidate. No full downstream matrix, original full-board gate, complete report replay or isolated production admission is implied.',
    previous_diagnostic=args.previous,
    contract='point-equality-contract.json',
    source_record=args.validation + '/sources.json',
    source_change=changes,
    validation=dict(commands=5, tests=tests, independent_integer_comparisons=45000, source_archive_verified=True),
    oracle='point-equality-oracle.json',
    exact_probe_witnesses_equal=True,
    allocation_samples_repeat_identically=repeatable,
    timing_totals=totals,
    binary_bytes={k:v['bytes'] for k,v in measurements['binaries'].items()},
    decision_reason=args.reason,
    next_step=args.next_step,
    limitations=measurements['limitations'],
    analysis_command=['python3', *sys.argv],
)
record['artifacts'] = {str(p.relative_to(root)):info(p)
                       for directory in [candidate, cost]
                       for p in sorted(directory.rglob('*')) if p.is_file()}
for name in ['analyze_point_equality.py', 'point-equality-contract.json', 'point-equality-oracle.json',
             'point_equality_oracle.py', 'point_equality_probe.rs', 'reproduce_point_equality_cost.py']:
    record['artifacts'][name] = info(root / name)
(root / args.diagnostic).write_text(json.dumps(record, indent=2) + '\n')
print(args.decision, tests, 'query medians', {k:v['median_ns'] for k,v in totals['query'].items()})
