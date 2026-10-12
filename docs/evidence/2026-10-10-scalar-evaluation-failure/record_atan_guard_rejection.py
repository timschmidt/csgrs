"""Reject the borrowed-magnitude shortcut while retaining every measured trial."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    implementation = read(E/'atan-zero-term-implementation-v1540.json')
    paired = read(E/'atan-zero-term-paired-cost-v1535/summary.json')
    controlled = read(E/'atan-zero-term-preconditioned-cost-v1536/summary.json')
    assert paired['source_record_sha256'] == controlled['source_record_sha256'] == implementation['source_record_sha256']
    assert paired['processes'] == controlled['processes'] == 936
    assert paired['guards_unchanged'] and controlled['guards_unchanged']
    assert controlled['cpu_preparation']['prelude_processes'] == 936
    assert paired['provenance'] == controlled['provenance']
    regressions = []
    for family, rows in paired['cases'].items():
        for old, new in zip(rows, controlled['cases'][family]):
            assert old['arguments'] == new['arguments']
            if old['median_percent_change'] > 5 and new['median_percent_change'] > 5:
                regressions.append({'family': family, 'arguments': old['arguments'],
                    'paired_percent_change': old['median_percent_change'],
                    'fixed_cpu_preparation_percent_change': new['median_percent_change']})
    cache = [r for r in regressions if r['family'] == 'atan' and r['arguments'][0] == 'cached_reciprocal_one']
    assert len(cache) == 1
    for directory, summary in [('atan-zero-term-paired-cost-v1535', paired),
                               ('atan-zero-term-preconditioned-cost-v1536', controlled)]:
        assert all(sha(E/directory/name) == digest for name, digest in summary['artifacts_sha256'].items())
    out = E/'atan-magnitude-guard-rejection-v1541.json'
    assert not out.exists()
    out.write_text(json.dumps({
        'status': 'Rejected isolated optimization; no runtime admission',
        'candidate_commit': 'b57a0546d', 'baseline_commit': '71c245225',
        'source_record': implementation['source_record'],
        'source_record_sha256': implementation['source_record_sha256'],
        'correctness_and_allocation_record': 'atan-zero-term-implementation-v1540.json',
        'paired_record': 'atan-zero-term-paired-cost-v1535/summary.json',
        'fixed_cpu_record': 'atan-zero-term-preconditioned-cost-v1536/summary.json',
        'persistent_measured_increases_above_five_percent': regressions,
        'controlled_cases': controlled['cases'], 'cpu_preparation': controlled['cpu_preparation'],
        'decision': 'The borrowed-magnitude early return preserves the frozen answers, allowances, cache facts and independent enclosures, and removes the unused wide square. The single-root zero-valued cache control nevertheless remains slower in both alternating-order measurements, including fixed-CPU preparation. Reject this variant under the performance-before-memory ladder; no causal explanation is established.',
        'next_experiment': 'Compute the existing first quotient before squaring, return when that quotient is zero, then square for nonzero terms. This tests a cheaper guard while keeping all frozen tests, inputs and cost controls.',
        'scope': 'All 468 original cost processes, 936 alternating-order processes and 936 fixed-CPU processes plus 936 preparation processes remain retained. No broad-stack, formal-proof or full-board admission is claimed for this rejected source.'
    }, indent=2)+'\n')
    print(json.dumps({'record': out.name, 'regressions': regressions}))


if __name__ == '__main__':
    main()
