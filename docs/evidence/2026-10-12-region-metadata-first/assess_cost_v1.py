"""Summarize every fixed paired operation observation, including controls."""
import gzip
import hashlib
import json
from pathlib import Path
from statistics import median
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'cost-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    result = json.loads((OUT / 'result.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    assert result['complete'] and result['all_expected_outcomes_passed']
    assert result['fresh_processes'] == 20 and result['observations'] == 360
    assert sha(E / 'measure_v1.py') == protocol['runner_sha256']
    assert sha(E / 'cost_probe.rs') == protocol['probe_sha256']
    for row in json.loads((OUT / 'commands.json').read_text()):
        assert row['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    maps = json.loads(gzip.decompress((OUT / 'source-hashes.json.gz').read_bytes()))
    for variant, entries in maps.items():
        location = Path(protocol['sources'][variant])
        assert all(sha(location / name) == digest for name, digest in entries.items())
    preparation = json.loads((E / 'preparation.json').read_text())
    assert sources() == preparation['main_source_sha256']
    binaries = json.loads((OUT / 'binaries.json').read_text())
    assert all(sha(Path(value['path'])) == value['sha256'] for value in binaries.values())
    observations = json.loads((OUT / 'observations.json').read_text())
    index = {}
    for sample in observations:
        for row in sample['rows']:
            key = (sample['mode'], sample['pair'], sample['variant'], row['case'], row['phase'])
            assert key not in index
            index[key] = row
    cases = ['unequal_sources', 'absent_sources', 'mixed_sources', 'equal_sources', 'duplicate', 'singleton']
    phases = ['first_validation', 'warm_validation', 'cloned_validation']
    timing, memory = [], []
    for case in cases:
        for phase in phases:
            pairs = [{variant: index[('time', pair, variant, case, phase)]['nanos']
                      for variant in ['baseline', 'candidate']} for pair in range(5)]
            ratios = [pair['candidate'] / pair['baseline'] for pair in pairs]
            timing.append({'case': case, 'phase': phase, 'pairs_nanos': pairs, 'paired_ratios': ratios,
                           'median_paired_ratio': median(ratios),
                           'candidate_faster_pairs': sum(ratio < 1 for ratio in ratios),
                           'baseline_range_nanos': [min(pair['baseline'] for pair in pairs), max(pair['baseline'] for pair in pairs)],
                           'candidate_range_nanos': [min(pair['candidate'] for pair in pairs), max(pair['candidate'] for pair in pairs)]})
            metrics = {}
            for metric in ['allocation_calls', 'allocated_bytes', 'retained_bytes', 'peak_extra_bytes']:
                values = {variant: [index[('memory', pair, variant, case, phase)][metric] for pair in range(5)]
                          for variant in ['baseline', 'candidate']}
                metrics[metric] = values | {'median_delta': median(values['candidate']) - median(values['baseline']),
                                            'paired_deltas': [c - b for c, b in zip(values['candidate'], values['baseline'])]}
            memory.append({'case': case, 'phase': phase, 'metrics': metrics})
    assessment = {
        'decision': 'fixed-operation-observations-complete',
        'timing': timing, 'memory': memory,
        'all_five_time_pairs_slower': [f"{row['case']}/{row['phase']}" for row in timing if row['candidate_faster_pairs'] == 0],
        'all_five_time_pairs_faster': [f"{row['case']}/{row['phase']}" for row in timing if row['candidate_faster_pairs'] == 5],
        'memory_median_increases': [f"{row['case']}/{row['phase']}/{metric}" for row in memory
                                    for metric, values in row['metrics'].items() if values['median_delta'] > 0],
        'probe_binaries': binaries,
        'limits': [
            'Six authored metadata cases are local constructor controls, not a representative or exhaustive geometry corpus.',
            'Both probes include the same TLS allocator wrapper; counting is enabled only in memory mode. Timing ratios are local instrumented operation observations.',
            'Fixture construction, input cloning between phases, and output-vector reservation are excluded. Each measured request clones a boundary collection and retains its returned region until the phase ends.',
            'Retained and peak-extra bytes are thread-local Rust allocator deltas, not process RSS, native allocation, or global peak-live memory.',
            'All expected outcomes pass in every probe. Three focused regression functions also pass separately in both measurement variants; they overlap candidate validation evidence.',
            'The test executable size is affected by probe/test code and is not the production CLI size.',
        ],
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'cost-assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'timing': [{key: row[key] for key in ['case', 'phase', 'median_paired_ratio', 'candidate_faster_pairs']} for row in timing],
                      'memory_median_increases': assessment['memory_median_increases']}))


if __name__ == '__main__':
    main()
