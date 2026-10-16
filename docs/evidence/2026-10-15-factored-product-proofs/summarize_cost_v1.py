"""Retain every row; compare latency only where both revisions certify it."""
import hashlib
import json
from pathlib import Path
from statistics import median

E = Path(__file__).resolve().parent
PHASE = E / 'cost-measurement-v1'
BUILD = Path('/tmp/csgrs-factored-product-proofs/cost-build-v1')
result = json.loads((PHASE / 'result.json').read_text())
assert result['passed'] and len(result['commands']) == 22
all_rows = []
groups = {}
for command in result['commands']:
    if command['name'] in {'prepare-target', 'artifact-sections'}:
        continue
    sample, label, kind = command['name'].split('-')
    rows = [json.loads(line) for line in (PHASE / (command['name'] + '.stdout')).read_text().splitlines()]
    assert len(rows) == (24 if kind == 'controls' else 14)
    for row in rows:
        assert row['certified'] + row['unknown'] == row['iterations']
        if kind == 'controls':
            assert row['certified'] == row['iterations']
        if kind == 'forms' and label == 'candidate':
            assert row['certified'] == (0 if row['case'] == 'unresolved_trig' else row['iterations'])
        key = (kind, row['case'], row.get('method'), row['prepared'])
        groups.setdefault(key, {}).setdefault(label, []).append(row | {'sample': int(sample)})
        all_rows.append(row | {'sample': int(sample), 'revision': label, 'probe': kind})

comparisons = []
for (kind, case, method, prepared), revisions in groups.items():
    assert set(revisions) == {'baseline', 'candidate'}
    assert all(len(rows) == 5 for rows in revisions.values())
    completed = all(row['certified'] == row['iterations'] for rows in revisions.values() for row in rows)
    summary = {
        'probe': kind, 'case': case, 'method': method, 'prepared': prepared,
        'both_completed': completed,
        'revisions': {},
    }
    for label, rows in revisions.items():
        summary['revisions'][label] = {
            'certified_counts': [row['certified'] for row in rows],
            'unknown_counts': [row['unknown'] for row in rows],
            'iterations_per_sample': rows[0]['iterations'],
            'nanoseconds_per_query': [row['nanoseconds'] / row['iterations'] for row in rows],
            'median_nanoseconds_per_query': median(row['nanoseconds'] / row['iterations'] for row in rows),
            'allocations_per_query': [row['allocations'] / row['iterations'] for row in rows],
            'requested_bytes_per_query': [row['requested_bytes'] / row['iterations'] for row in rows],
            'peak_live_bytes': [row['peak_live_bytes'] for row in rows],
            'retained_bytes': [row['retained_bytes'] for row in rows],
        }
    if completed:
        baseline = summary['revisions']['baseline']['median_nanoseconds_per_query']
        candidate = summary['revisions']['candidate']['median_nanoseconds_per_query']
        summary['median_latency_change_percent'] = 100 * (candidate / baseline - 1)
    comparisons.append(summary)

artifacts = {}
for p in BUILD.iterdir():
    if p.is_file() and (p.suffix == '.rlib' or p.name.endswith(('-controls', '-forms'))):
        artifacts[str(p)] = {'bytes': p.stat().st_size, 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
summary = {
    'contract': 'cost-contract-v1.json',
    'contract_sha256': hashlib.sha256((E / 'cost-contract-v1.json').read_bytes()).hexdigest(),
    'phase_result_sha256': hashlib.sha256((PHASE / 'result.json').read_bytes()).hexdigest(),
    'sample_rows': all_rows,
    'comparisons': comparisons,
    'artifacts': artifacts,
    'elf_sections': (PHASE / 'artifact-sections.stdout').read_text(),
    'claim': 'Focused instrumented scalar comparison, not a cross-library or downstream performance result. All five fixed alternating pairs are retained. Newly certified rows are capability expansion, not speedups over Unknown.',
}
p = E / 'cost-summary-v1.json'
assert not p.exists()
p.write_text(json.dumps(summary, indent=2) + '\n')
print('Retained', len(all_rows), 'rows across', len(comparisons), 'workload/mode combinations')
for row in comparisons:
    print(row['probe'], row['case'], row['method'], row['prepared'],
          'matched change %=' + str(row.get('median_latency_change_percent')),
          {label: data['certified_counts'] for label, data in row['revisions'].items()})
