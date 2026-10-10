"""Keep every matched timing sample and allocation result, including regressions."""
import hashlib
import json
from pathlib import Path
from statistics import median

EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'class-precheck-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
trials = read(OUT / 'trials.json')
assert len(trials) == 20
verification = read(OUT / 'source-verification.json')
assert verification['sampling_complete'] and verification['all_source_library_probe_and_binary_hashes_unchanged']
groups = {}
for trial in trials:
    for row in trial['rows']:
        key = (row['family'], row['stage'], trial['variant'], trial['mode'])
        groups.setdefault(key, []).append(row)
assert len(groups) == 112 and all(len(rows) == 5 for rows in groups.values())
memory_fields = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
comparisons = []
for family, stage in sorted({key[:2] for key in groups}):
    output = {'family': family, 'stage': stage}
    checksums = set()
    for variant in ['first', 'class_precheck']:
        timing = groups[(family, stage, variant, 'time')]
        memory = groups[(family, stage, variant, 'memory')]
        checksums.update(row['checksum'] for row in timing)
        assert len({row['checksum'] for row in memory}) == 1
        assert all(row['iterations'] == 8192 for row in timing)
        assert all(row['iterations'] == 1000 for row in memory)
        assert all({name: row[name] for name in memory_fields} == {name: memory[0][name] for name in memory_fields} for row in memory)
        samples = [row['nanos'] / row['iterations'] for row in timing]
        output[variant] = {
            'nanoseconds_per_query_samples': samples,
            'median_nanoseconds_per_query': median(samples),
            'sample_range': [min(samples), max(samples)],
            'allocations_per_1000_queries': {name: memory[0][name] for name in memory_fields},
            'allocation_checksum': memory[0]['checksum'],
        }
    assert len(checksums) == 1
    assert output['first']['allocation_checksum'] == output['class_precheck']['allocation_checksum']
    first, candidate = output['first'], output['class_precheck']
    output['median_ratio'] = candidate['median_nanoseconds_per_query'] / first['median_nanoseconds_per_query']
    output['sample_ranges_overlap'] = max(first['sample_range'][0], candidate['sample_range'][0]) <= min(first['sample_range'][1], candidate['sample_range'][1])
    output['allocation_deltas_per_1000_queries'] = {name: candidate['allocations_per_1000_queries'][name] - first['allocations_per_1000_queries'][name] for name in memory_fields}
    comparisons.append(output)
(OUT / 'comparisons.json').write_text(json.dumps(comparisons, indent=2) + '\n')
binaries = read(OUT / 'binaries.json')
summary = {
    'decision': 'exact-class precheck not retained; continue downstream validation with the first checked formatter',
    'assessment': 'The hypothesis did not produce the proposed allocation or retained-memory saving: every measured memory field is identical for all 28 rows. Twenty-one timing medians are slower, seven faster, with all ranges overlapping; this does not prove a timing regression or equivalence. A 16/32-byte executable reduction is insufficient reason to retain an extra runtime branch without a demonstrated higher-priority benefit. Sampling is closed and the complete candidate, passing validation and adverse samples remain retained.',
    'scope': read(OUT / 'plan.json')['scope'],
    'samples_each_variant_and_mode': 5, 'query_rows': 28,
    'all_outputs_checked_each_iteration': True, 'all_checksums_match': True,
    'all_allocation_samples_repeat_exactly': True,
    'medians_faster': sum(row['median_ratio'] < 1 for row in comparisons),
    'medians_slower': sum(row['median_ratio'] > 1 for row in comparisons),
    'median_ratio_range': [min(row['median_ratio'] for row in comparisons), max(row['median_ratio'] for row in comparisons)],
    'slower_nonoverlapping_rows': [{'family': row['family'], 'stage': row['stage'], 'median_ratio': row['median_ratio']} for row in comparisons if row['median_ratio'] > 1 and not row['sample_ranges_overlap']],
    'rows_with_increased_memory': [{'family': row['family'], 'stage': row['stage'], 'deltas': row['allocation_deltas_per_1000_queries']} for row in comparisons if any(value > 0 for value in row['allocation_deltas_per_1000_queries'].values())],
    'executable_bytes': {name: record['bytes'] for name, record in binaries.items()},
    'executable_byte_deltas': {mode: binaries['class_precheck-' + mode]['bytes'] - binaries['first-' + mode]['bytes'] for mode in ['time', 'memory']},
    'sampling_complete': True, 'main_runtime_unchanged': True,
    'limitations': 'Five-sample ranges are descriptive, not confidence intervals; overlap does not prove absence of regression. Construction is outside query rows, global constants may be warm, equality/checksum overhead is included, and allocator accounting excludes native heaps and overhead. No downstream performance or universal equivalence is inferred.',
}
summary['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted(OUT.iterdir()) if path.is_file()}
summary['scripts'] = {name: {'bytes': (EVIDENCE / name).stat().st_size, 'sha256': digest(EVIDENCE / name)} for name in ['checked_cost_probe.rs', 'reproduce_class_precheck_cost.py', 'analyze_class_precheck_cost.py']}
(EVIDENCE / 'class-precheck-cost.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({name: value for name, value in summary.items() if name not in ['artifacts', 'scripts']}, indent=2))
