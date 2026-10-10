"""Describe all checked-format resource costs without equating proof contracts."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import statistics

EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'checked-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
trials = read(OUT / 'trials.json')
assert len(trials) == 20
assert read(OUT / 'source-verification.json')['all_source_library_probe_and_binary_hashes_unchanged']
groups = defaultdict(list)
for trial in trials:
    for row in trial['rows']:
        groups[trial['variant'], trial['mode'], row['family'], row['stage']].append(row)
assert all(len(rows) == 5 for rows in groups.values())
common = [(row['family'], row['stage']) for row in trials[0]['rows']]
comparisons = []
memory_fields = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
for family, stage in common:
    timing, memory = {}, {}
    for variant in ['legacy', 'checked']:
        rows = groups[variant, 'time', family, stage]
        assert len({(row['iterations'], row['checksum'], row['places']) for row in rows}) == 1
        samples = [row['nanos'] / row['iterations'] for row in rows]
        timing[variant] = {'nanoseconds_per_query_samples': samples, 'median': statistics.median(samples), 'min': min(samples), 'max': max(samples)}
        rows = groups[variant, 'memory', family, stage]
        assert len({tuple(row[field] for field in memory_fields) for row in rows}) == 1
        memory[variant] = {field: rows[0][field] for field in memory_fields} | {'iterations': rows[0]['iterations'], 'identical_in_all_five_samples': True}
    for mode in ['time', 'memory']:
        assert groups['legacy', mode, family, stage][0]['checksum'] == groups['checked', mode, family, stage][0]['checksum']
    comparisons.append({'family': family, 'stage': stage, 'timing': timing, 'checked_over_legacy_median': timing['checked']['median'] / timing['legacy']['median'], 'nonoverlapping_slower_range': timing['checked']['min'] > timing['legacy']['max'], 'nonoverlapping_faster_range': timing['checked']['max'] < timing['legacy']['min'], 'allocations': memory})
exclusive = []
for family in ['unresolved_zero', 'original_hypot', 'rounding_tie']:
    for stage in ['first', 'warm']:
        rows = groups['checked', 'time', family, stage]
        samples = [row['nanos'] / row['iterations'] for row in rows]
        memory = groups['checked', 'memory', family, stage]
        assert len({tuple(row[field] for field in memory_fields) for row in memory}) == 1
        exclusive.append({'family': family, 'stage': stage, 'nanoseconds_per_query_samples': samples, 'median_nanoseconds_per_query': statistics.median(samples), 'allocations_per_1000_queries': {field: memory[0][field] for field in memory_fields}, 'typed_outcome': 'Exhausted' if family == 'rounding_tie' else 'UnknownZero'})
(OUT / 'comparisons.json').write_text(json.dumps(comparisons, indent=2) + '\n')
(OUT / 'checked-only.json').write_text(json.dumps(exclusive, indent=2) + '\n')
binaries = read(OUT / 'binaries.json')
assert all(digest(Path(item['path'])) == item['sha256'] for item in binaries.values())
record = {
    'decision': 'checked API costs characterized; test a cheaper exact-class zero precheck before choosing the final candidate',
    'scope': read(OUT / 'plan.json')['scope'],
    'samples_each_variant_and_mode': 5, 'common_timing_rows': 22, 'checked_only_rows': 6,
    'all_outputs_checked_each_iteration': True, 'all_common_checksums_match': True,
    'all_allocation_samples_repeat_exactly': True,
    'common_medians_faster': sum(row['checked_over_legacy_median'] < 1 for row in comparisons),
    'common_medians_slower': sum(row['checked_over_legacy_median'] > 1 for row in comparisons),
    'median_ratio_range': [min(row['checked_over_legacy_median'] for row in comparisons), max(row['checked_over_legacy_median'] for row in comparisons)],
    'assessment': 'Exact zero, rational and extreme rational scales are faster in this probe. Pi, sqrt(2), huge-scaled pi and unresolved nonzero offsets cost more; pi rows are about 3.7 times the legacy approximate formatter cost. First calls on 1000 huge/tiny pi-scaled values retain 384000 requested Rust bytes, versus 168/32 in the legacy path. These are costs of a stronger explicit rounding/completion contract, not regressions introduced into existing callers. The exact-class precheck hypothesis must preserve every receipt and measure its own tradeoff; these timings do not establish that full structural facts cause all observed costs.',
    'checked_only_outcomes': exclusive,
    'executable_bytes': {name: item['bytes'] for name, item in binaries.items()},
    'executable_scope': 'These executables select different formatting implementations, and the checked binary also contains three additional typed-failure fixtures. Their byte difference is not an isolated library-function size measurement or a claimed application-size saving.',
    'pending': ['evaluate the exact-class zero precheck as a separate candidate', 'downstream witness/report replay and retained-candidate review'],
    'main_runtime_unchanged': True,
    'limitations': 'Construction is outside timed rows; constants can already be warm. Equality/checksum overhead is included in both paths. Timing binaries contain no allocation counter. Allocation accounting excludes native heaps and allocator overhead. Five-sample ranges are descriptive, not confidence intervals. Different proof contracts prevent interpreting these API cost ratios as a drop-in optimization comparison; no unchanged downstream performance or hard work bound is established.',
}
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted(OUT.iterdir()) if path.is_file()}
record['scripts'] = {name: {'bytes': (EVIDENCE / name).stat().st_size, 'sha256': digest(EVIDENCE / name)} for name in ['checked_cost_probe.rs', 'reproduce_checked_cost.py', 'analyze_checked_cost.py']}
(EVIDENCE / 'checked-cost.json').write_text(json.dumps(record, indent=2) + '\n')
print('Common medians faster/slower:', record['common_medians_faster'], record['common_medians_slower'])
print('Ratio range:', record['median_ratio_range'], '; exact repeatable allocation samples; all witnesses preserved.')
