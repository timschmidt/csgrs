"""Describe every fixed combined-library row without filtering adverse samples."""
import gzip
import json
from pathlib import Path
import statistics

from common_v1 import E, ROOT, PREP, sha, dump, verify


def main():
    preparation = json.loads((PREP / 'protocol.json').read_text())
    verify(preparation, full=True)
    out = E / 'library-costs-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['fixed_cost_processes'] == 110
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    protocol = json.loads((out / 'protocol.json').read_text())
    assert sha(__file__) == protocol['assessor_sha256']
    trials = json.loads(gzip.decompress((out / 'trials.json.gz').read_bytes()))
    assert len(trials) == 136 and all(r['identity_matches_reference'] for r in trials)
    selected = [r for r in trials if not r['preflight'] and r['suite'] != 'abort']
    assert len(selected) == 110
    timing, memory = [], []
    for suite in ['rational', 'quadratic', 'integer', 'arc', 'metadata', 'broad']:
        for mode in (['time'] if suite == 'broad' else ['time', 'memory']):
            subset = [r for r in selected if r['suite'] == suite and r['mode'] == mode]
            assert len(subset) == 10
            rows = {}
            for variant in ['baseline', 'candidate']:
                samples = sorted((r for r in subset if r['variant'] == variant), key=lambda r: r['pair'])
                assert [r['pair'] for r in samples] == list(range(1, 6))
                rows[variant] = [{r['name']: r for r in s['rows'] if 'name' in r} for s in samples]
            names = list(rows['baseline'][0])
            assert all(list(r) == names for group in rows.values() for r in group)
            for name in names:
                value = {'suite': suite, 'name': name, 'iterations': rows['baseline'][0][name]['iterations']}
                assert all(r[name]['iterations'] == value['iterations'] for group in rows.values() for r in group)
                if mode == 'time':
                    a, b = ([r[name]['nanos'] for r in rows[v]] for v in ['baseline', 'candidate'])
                    assert all(x > 0 for x in a + b)
                    ratios = [y / x for x, y in zip(a, b)]
                    value |= {'baseline_ns': a, 'candidate_ns': b,
                              'baseline_median_ns': statistics.median(a), 'candidate_median_ns': statistics.median(b),
                              'ratio_of_medians': statistics.median(b) / statistics.median(a),
                              'paired_ratios': ratios, 'median_paired_ratio': statistics.median(ratios),
                              'candidate_higher_pairs': sum(y > x for x, y in zip(a, b)),
                              'candidate_lower_pairs': sum(y < x for x, y in zip(a, b))}
                    timing.append(value)
                else:
                    fields = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
                    value['fields'] = {}
                    for field in fields:
                        a, b = ([r[name][field] for r in rows[v]] for v in ['baseline', 'candidate'])
                        value['fields'][field] = {'baseline': a, 'candidate': b,
                                                  'baseline_median': statistics.median(a), 'candidate_median': statistics.median(b),
                                                  'paired_deltas': [y - x for x, y in zip(a, b)],
                                                  'candidate_higher_pairs': sum(y > x for x, y in zip(a, b)),
                                                  'candidate_lower_pairs': sum(y < x for x, y in zip(a, b))}
                    memory.append(value)
    assert len(timing) == 365 and len(memory) == 134
    broad = [r for r in selected if r['suite'] == 'broad']
    identities = [[{k: v for k, v in r.items() if k != 'nanos'} for r in t['rows']] for t in broad]
    history_path = ROOT / 'docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/cost/rows.json'
    historical = json.loads(history_path.read_text())
    assert all(rows == identities[0] for rows in identities)
    binary_data = json.loads((out / 'binaries.json').read_text())
    sizes = []
    for name in sorted(k.removeprefix('baseline-') for k in binary_data if k.startswith('baseline-')):
        a, b = (binary_data[v + '-' + name] for v in ['baseline', 'candidate'])
        assert all(sha(r['path']) == r['sha256'] for r in [a, b])
        sizes.append({'probe': name, 'baseline_bytes': a['bytes'], 'candidate_bytes': b['bytes'],
                      'candidate_minus_baseline_bytes': b['bytes'] - a['bytes']})
    summary = {suite: {'timing_rows': sum(r['suite'] == suite for r in timing),
                       'candidate_higher_all_five': sum(r['suite'] == suite and r['candidate_higher_pairs'] == 5 for r in timing),
                       'candidate_lower_all_five': sum(r['suite'] == suite and r['candidate_lower_pairs'] == 5 for r in timing)}
               for suite in ['rational', 'quadratic', 'integer', 'arc', 'metadata', 'broad']}
    assessment = {
        'status': 'combined-library-fixed-comparison-complete-pending-admission-decision',
        'assessor_sha256': sha(__file__), 'preparation_sha256': sha(PREP / 'protocol.json'),
        'scope': 'Current main including authorized native changes versus the complete reviewed candidate, at independent library boundaries using System. This is distinct from all closed per-component trials.',
        'fixed_cost_processes': 110, 'abort_control_processes': 6, 'preflight_processes': 20,
        'all_measured_witnesses_representation_inventories_and_abort_receipts_match_between_variants': True,
        'historical_broad_witnesses': {'path': str(history_path.relative_to(ROOT)), 'sha256': sha(history_path),
                                      'all_231_identical': identities[0] == historical},
        'timing_summary': summary, 'timing_rows': timing, 'memory_rows': memory, 'binary_sizes': sizes,
        'limits': ['Ratios and all-five directions are descriptive observations, not significance estimates.',
                   'Memory is requested Rust allocation traffic and phase-relative live/peak deltas, not allocator RSS or whole-life reclamation.',
                   'Private metadata probes use an identical TLS allocator wrapper even in time mode. They are not production executable timings.',
                   'The broad probe retains its existing setup and timing windows; its timings do not cover the whole application.',
                   'Finite witnesses cannot establish all P1-P39 properties. Existing adverse experiments and failures remain valid.',
                   'This assessment does not run or replace the original native gates or whole-report/CLI costs.'],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'plan_complete': False,
    }
    destination = E / 'library-assessment-v1.json'
    assert not destination.exists()
    dump(destination, assessment)
    print(json.dumps({'timing_rows': len(timing), 'memory_rows': len(memory), 'summary': summary,
                      'historical_broad_witnesses_identical': identities[0] == historical, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
