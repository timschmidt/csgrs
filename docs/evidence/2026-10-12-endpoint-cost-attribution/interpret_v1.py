"""Summarize observed work and phase retention, preserving every adverse row."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not (E / 'interpretation-v1.json').exists()
    assessment = json.loads((E / 'assessment-v1.json').read_text())
    windows = json.loads(gzip.decompress((E / 'all-window-comparisons-v1.json.gz').read_bytes()))
    assert all(not row['differences'] for row in assessment['arc_allocation_replay_against_all_five_original_pairs'])
    broad = [row for row in assessment['selected_windows']['broad'] if row['stage'] == 'measured']
    assert len(broad) == 9
    assert all(row['all_counters_identical'] for row in broad)
    assert all(row[variant]['spans']['endpoint']['entries'] == 0
               for row in broad for variant in ['baseline', 'candidate'])
    family_retention = []
    for family in ['opaque_translated_non_endpoint', 'off_circle', 'aborted_unknown',
                   'approximate_non_endpoint', 'near_endpoint_off_circle']:
        # Start immediately before construction and stop immediately after the
        # cloned query, while cases and their clone are both still retained.
        # Exclude the first "between" window, which can drop the prior family.
        rows = [row for row in windows['arc'] if row['label'].startswith(family + '_')
                and not (row['label'] == family + '_construction' and row['stage'] == 'between')]
        assert len(rows) == 7
        totals = {}
        for variant in ['baseline', 'candidate']:
            totals[variant] = {
                field: sum(row[variant]['spans']['probe'][field] for row in rows)
                for field in ['allocations', 'allocated_bytes', 'deallocations', 'deallocated_bytes']}
            totals[variant]['live_requested_bytes_increment'] = (
                totals[variant]['allocated_bytes'] - totals[variant]['deallocated_bytes'])
        delta = {field: totals['candidate'][field] - totals['baseline'][field] for field in totals['baseline']}
        family_retention.append({'family': family, 'boundary': 'immediately before construction through cloned measured query',
                                 'includes_three_intervening_windows': True, 'totals': totals,
                                 'candidate_minus_baseline': delta})
    phase_rows = []
    for row in assessment['selected_windows']['arc']:
        if row['stage'] != 'measured':
            continue
        delta = row['candidate_minus_baseline']
        phase_rows.append({'label': row['label'], 'candidate_minus_baseline': delta,
                           'net_live_requested_bytes_delta': delta['spans']['probe']['allocated_bytes']
                           - delta['spans']['probe']['deallocated_bytes']})
    record = {
        'decision': 'continue-isolated-endpoint-parent-diagnosis-no-production-retention',
        'runner_sha256': sha(Path(__file__)),
        'evidence_sha256': {str(path.relative_to(ROOT)): sha(path) for path in
                           [E / 'assessment-v1.json', E / 'all-window-comparisons-v1.json.gz']},
        'broad_slower_in_all_five_prior_pairs': [
            {'label': row['label'], 'endpoint_calls_in_measured_window': 0,
             'all_recorded_counters_identical': True, 'measured_probe_counters': row['baseline']['spans']['probe']}
            for row in broad],
        'broad_interpretation': 'The nine observed measured loops do not execute the changed endpoint path and do not perform more recorded work. Setup and checked windows also agree. The preceding warm-area preparation window does execute endpoint classification and saves 96 allocation calls / 3840 requested bytes. Earlier state, code generation/layout and timing variability remain possible explanations for the original slower samples; none is established as their cause.',
        'arc_phase_increases': phase_rows,
        'arc_family_accounting': family_retention,
        'arc_interpretation': [
            'The off-circle and near-endpoint warm phases each add 143 allocations / 25168 requested bytes inside rational linear-retention scopes. Their phase net increases remain 25168 and 46904 bytes respectively.',
            'Including construction, first/warm/cloned queries and all three intervening windows, each of those families has a 125664-byte smaller live requested-byte increment in the candidate. This is derived from probe-root allocation/deallocation totals, not a sum of overlapping scopes. It does not remove either adverse phase row or prove reclamation after all owners are dropped.',
            'Opaque construction adds 11 allocations / 256 bytes, including one additional 80-byte evaluation-cache allocation; it retains 88 additional requested bytes in the phase. No endpoint call occurs in this construction window.',
            'Aborted-unknown first queries add 128 allocations / 3080 bytes, wholly reflected in the existing policy equality and is_zero scopes, with one additional 80-byte evaluation-cache allocation. Their net phase increase is 120 bytes. The probe attaches abort to an intermediate, as previously documented; final-coordinate abort qualification remains the separate prior probe.',
            'Approximate non-endpoint first queries add three allocations / 168 bytes inside squared distance, with unchanged call counts and no additional Laurent-cache or rational-retention counter activity. The 168-byte retention increase persists through the cloned measured query. These counters do not identify the specific retained scalar payload.',
        ],
        'retention_basis': 'Keep this source available only for the next isolated full-board profile. Correctness and report evidence remain intact; the diagnostic explains observed call and phase-memory changes but does not erase timing tradeoffs or qualify production retention. In particular, equal counters do not establish equal runtime or a universal cache-completeness theorem.',
        'next_step': 'Profile the frozen uninstrumented endpoint candidate on the original complete boards, using a separate frame-pointer target and the original 120-second worker assertion. Select the next substantial caller cost from that profile. Preserve all five original timing pairs, approximate controls, retained-memory increases, complete reports, original native gates and broader qualification before any production admission.',
        'runtime_admitted': False, 'plan_complete': False,
    }
    for family in ['off_circle', 'near_endpoint_off_circle']:
        row = next(row for row in family_retention if row['family'] == family)
        assert row['candidate_minus_baseline']['live_requested_bytes_increment'] == -125664
    for row in assessment['selected_windows']['broad']:
        if row['stage'] in ['setup', 'checked']:
            assert row['all_counters_identical']
    warm_before = next(row for row in windows['broad'] if row['label'] == 'warm_polygon_area_17' and row['stage'] == 'between')
    assert warm_before['candidate_minus_baseline']['spans']['probe']['allocations'] == -96
    assert warm_before['candidate_minus_baseline']['spans']['probe']['allocated_bytes'] == -3840
    (E / 'interpretation-v1.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({'decision': record['decision'], 'broad_loops_with_equal_counters': len(broad),
                      'family_live_increment_deltas': {row['family']: row['candidate_minus_baseline']['live_requested_bytes_increment'] for row in family_retention}}))


if __name__ == '__main__':
    main()
