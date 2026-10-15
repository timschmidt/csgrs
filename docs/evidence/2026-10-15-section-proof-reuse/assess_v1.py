"""Summarize every sample without discarding unfavorable controls."""
import hashlib
import json
from pathlib import Path
import re
import statistics

E = Path(__file__).resolve().parent
ROOT = E.parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / 'assessment-v1.json'
    assert not out.exists()
    result = json.loads((E / 'cost-v1/result.json').read_text())
    assert result['passed'] and len(result['commands']) == 88
    rows = result['commands']
    samples = [row for row in rows if 'sample' in row]
    assert len(samples) == 72
    summaries = {}
    for shape in ['slice_middle', 'slice_bottom', 'slice_top', 'flatten']:
        summaries[shape] = {}
        for cache in ['warm', 'varied', 'disabled']:
            pair = {}
            for label in ['baseline', 'candidate']:
                group = [row['sample'] for row in samples if row['name'].endswith('-' + shape + '-' + cache + '-' + label)]
                assert len(group) == 3
                metrics = ['nanos', 'allocations', 'requested_bytes', 'retained_byte_delta', 'peak_live_bytes_above_start', 'output_bytes']
                pair[label] = {}
                for metric in metrics:
                    per_operation = metric not in {'retained_byte_delta', 'peak_live_bytes_above_start'}
                    values = [s[metric] / s['iterations'] if per_operation else s[metric] for s in group]
                    pair[label][metric] = {'unit': 'per operation' if per_operation else 'bytes per sample',
                                           'median': statistics.median(values), 'samples': values}
            pair['median_latency_change_percent'] = 100 * (pair['candidate']['nanos']['median'] / pair['baseline']['nanos']['median'] - 1)
            summaries[shape][cache] = pair
    corpus = {}
    for label in ['baseline', 'candidate']:
        group = [row for row in rows if row['name'].endswith('-' + label + '-corpus')]
        assert len(group) == 3
        corpus[label] = {'process_wall_seconds': [row['seconds'] for row in group],
                         'maximum_rss_kib': [int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', (E / 'cost-v1' / (row['name'] + '.resources')).read_text()).group(1)) for row in group]}
    phase_results = {}
    tests = {}
    for phase in ['baseline-v1', 'baseline-v2', 'candidate-v1', 'cargo-v1']:
        folder = E / phase
        result = json.loads((folder / 'result.json').read_text())
        phase_results[phase] = {'passed': result['passed'], 'commands': len(result['commands']), 'sha256': sha(folder / 'result.json')}
        for row in result['commands']:
            matches = re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored', (folder / (row['name'] + '.stdout')).read_text())
            if matches:
                tests[phase + '/' + row['name']] = [sum(int(match[i]) for match in matches) for i in range(3)]
    assert not phase_results['baseline-v1']['passed']
    assert all(phase_results[p]['passed'] for p in ['baseline-v2', 'candidate-v1', 'cargo-v1'])
    outputs = json.loads((E / 'cost-v1/exact-rows.json').read_text())
    assert all(row['identical'] for row in outputs.values())
    comparison = json.loads((E / 'candidate-v1/comparison.json').read_text())
    assert comparison['passed'] and len(comparison['cases']) == 43
    identity = json.loads((E / 'build-identity-v1/result.json').read_text())
    assert identity['passed']
    prior = json.loads((ROOT / 'docs/evidence/2026-10-15-slice-projection-contract/closure-v1.json').read_text())
    assert all(sha(ROOT / path) == digest for path, digest in prior['pending_native_sha256'].items())
    report = {
        'baseline_commit': '3a2d3fd1b',
        'idea': 'Reuse one signed-winding fill when both native one-sided section lists have identical directed segment indices and closure flags.',
        'owning_layer': 'CSGRS modeling composition; Hypermesh owns section records, Hypercurve owns winding proof and already supports union idempotence. No native algorithm, tolerance, policy, test or fixture changed.',
        'properties': ['P4', 'P17', 'P18', 'P19', 'P20', 'P25', 'P27', 'P35', 'P36', 'P37', 'P39'],
        'semantic_oracles': 'Unchanged 25 regressions use independently derived exact area, membership and determinant volume, separate topology checks, invalid-input diagnostics, cancellation/multiplicity and cache/policy sequences. Cost probes independently prove square area, containment and every boundary edge. Snapshot agreement alone is not a semantic oracle.',
        'phase_results': phase_results,
        'compiled_dependency_identity': identity,
        'tests_passed_failed_ignored': tests,
        'baseline_harness_failure': 'baseline-v1 passes all 25 regressions but cannot compile the JSON snapshot against serialization-disabled minimal native libraries. No runtime finding; trial retained. baseline-v2 and candidate-v1 directly compile the same CSGRS curve feature against identical serialization-enabled native paths/bytes. Cargo curve-only and minimal boundaries are independently validated in cargo-v1.',
        'cost': {'commands': 88, 'fresh_process_timing_samples': 72, 'completed_timed_operations': 9216,
                 'dump_variants_per_revision': 512, 'statistics': summaries,
                 'corpus_process_samples': corpus, 'exact_rows': outputs,
                 'artifacts': json.loads((E / 'cost-v1/artifacts.json').read_text())},
        'snapshot': comparison,
        'implementation_sha256': {'src/solid.rs': sha(ROOT / 'src/solid.rs')},
        'pending_native_sha256': prior['pending_native_sha256'],
        'limitations': [
            'Shared unpinned host, three samples per case and allocation instrumentation. Short controls can vary. No universal or cross-library speedup claim; all unfavorable results retained.',
            'Exact native library bytes are shared by both direct-rustc probe builds; separate Cargo tests cover feature isolation. Prior E4 timing used a different configuration and is not a matched cost baseline for this probe.',
            'Operation-local certainty and exposed boundary evidence are preserved; this does not establish inherited mesh-construction provenance or analytic-surface fidelity.',
            'No internal total-work, retained-byte or cancellation bounds. Trials use external 1800-second wall and 8 GiB per-process address-space containment.',
            'Full workflow matrix, clean external release, broad strict completion and the 35 pending native files remain open. No push. Previously reproduced Hypercircuit fixture failures are not re-run here.'
        ],
        'full_plan_complete': False,
    }
    out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'tests': tests, 'latency_changes_percent': {shape: {cache: item['median_latency_change_percent'] for cache, item in states.items()} for shape, states in summaries.items()}, 'corpus': corpus}, indent=2))


if __name__ == '__main__':
    main()
