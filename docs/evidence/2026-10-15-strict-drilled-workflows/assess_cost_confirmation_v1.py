"""Retain every long-batch result alongside the earlier adverse short batches."""
from pathlib import Path
import hashlib, json, re, statistics

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
phase = 'cost-confirmation-v1'
out = E/'cost-confirmation-assessment-v1.json'
assert not out.exists()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
earlier = json.loads((E/'scalar-assessment-v1.json').read_text())
assert all(sha(ROOT/p) == digest for p, digest in earlier['implementation_sha256'].items())
assert all(sha(ROOT/p) == digest for p, digest in earlier['pending_native_sha256'].items())
result = json.loads((E/phase/'result.json').read_text())
assert result['passed'] and len(result['commands']) == 10
samples = {}
for command in result['commands']:
    assert command['exit_code'] == 0 and command['sources_unchanged']
    match = re.fullmatch(r'(\d)-(baseline|candidate)', command['name'])
    assert match
    pair, revision = int(match[1]), match[2]
    rows = [json.loads(line) for line in (E/phase/(command['name']+'.stdout')).read_text().splitlines()]
    assert len(rows) == 24
    for row in rows:
        assert row['iterations'] == 4096 and row['certified'] + row['unknown'] == 4096
        expected = 0 if revision == 'baseline' and row['case'] == 'cancel' else 4096
        assert row['certified'] == expected
        key = (row['case'], row['method'], row['prepared'])
        samples.setdefault(key, {}).setdefault(revision, {})[pair] = row
comparisons = []
for (case, method, prepared), revisions in sorted(samples.items()):
    measured = {}
    for revision, rows in revisions.items():
        assert set(rows) == set(range(5))
        measured[revision] = {'raw_samples': [rows[pair] for pair in range(5)]}
        for metric in ['nanoseconds', 'allocations', 'requested_bytes']:
            measured[revision][metric+'_per_operation_median'] = statistics.median(row[metric]/4096 for row in rows.values())
        for metric in ['peak_live_bytes', 'retained_bytes']:
            measured[revision][metric+'_median'] = statistics.median(row[metric] for row in rows.values())
        for metric in ['certified', 'unknown']:
            measured[revision][metric+'_total'] = sum(row[metric] for row in rows.values())
    complete = all(row['certified'] == 4096 for rows in revisions.values() for row in rows.values())
    before, after = [measured[revision]['nanoseconds_per_operation_median'] for revision in ['baseline', 'candidate']]
    comparisons.append({
        'case': case, 'method': ['diff_of_products', 'signed_product_sum', 'active_signed_product_sum'][method],
        'prepared': prepared, 'both_contracts_completed': complete,
        'latency_change_percent_for_completed_contract': 100*(after/before-1) if complete else None,
        'latency_ratios_by_process_pair': [revisions['candidate'][pair]['nanoseconds']/revisions['baseline'][pair]['nanoseconds'] for pair in range(5)] if complete else None,
        'measurements': measured,
    })
assert len(comparisons) == 24
record = {
    'status': 'all fixed long-batch samples retained; retention decision remains separate',
    'earlier_assessment': 'scalar-assessment-v1.json', 'earlier_assessment_sha256': sha(E/'scalar-assessment-v1.json'),
    'earlier_results_remain_valid_observations': True,
    'scope': json.loads((E/'cost-confirmation-contract-v1.json').read_text()),
    'comparisons': comparisons,
    'totals': {revision: {metric: sum(row['measurements'][revision][metric+'_total'] for row in comparisons) for metric in ['certified', 'unknown']} for revision in ['baseline', 'candidate']},
    'limitations': 'Five fixed process pairs on a shared unpinned host, System allocator instrumented. These are focused scalar probes, not downstream/application speed claims or statistical significance. Cancellation baseline did not complete the required proof; no latency ratio is assigned to it.',
}
out.write_text(json.dumps(record, indent=2)+'\n')
print(json.dumps({'comparisons': len(comparisons), 'totals': record['totals']}))
