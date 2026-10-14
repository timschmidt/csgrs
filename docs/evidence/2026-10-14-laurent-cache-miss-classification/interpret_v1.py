"""Post-observation interpretation of the closed V2 counts; no new samples."""
import json
from miss_classification_common_v2 import E, PARENT, PREP, SUITES, sha, dump, verify


def main():
    destination = E / 'interpretation-v1.json'
    assert not destination.exists()
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    assessment = json.loads((E / 'assessment-v2.json').read_text())
    assert assessment['assessor_sha256'] == sha(E / 'assess_v2.py')
    assert assessment['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert all(sha(E / path) == value for path, value in assessment['artifacts_sha256'].items())
    assert assessment['all_counts_repeat_in_reversed_order']
    assert assessment['instrumented_parent_public_counts_equal_in_all_68_phases']
    assert assessment['all_56_historical_counter_phases_match']
    phases = assessment['classified_phases']
    assert len(phases) == 68
    summaries = []
    for row in phases:
        groups = row['eligibility_groups']
        empty = sum(g['empty_cell'] for g in groups.values())
        initialized = sum(g['initialized_miss'] for g in groups.values())
        hits = sum(g['hit'] for g in groups.values())
        lookups = row['events']['retained_lookup']
        assert empty + initialized + hits == lookups
        # These are checked observations of the already closed samples, not
        # prospective acceptance rules or a requirement to make a count positive.
        assert initialized == 0
        summaries.append({'suite': row['suite'], 'phase': row['phase'], 'iterations': row['iterations'],
            'lookups': lookups, 'empty_cell': empty, 'initialized_miss': initialized, 'hits': hits,
            'eligible_empty_cell': groups['eligible']['empty_cell'],
            'ineligible_empty_cell': groups['ineligible']['empty_cell'],
            'ineligible_lookup_count_fraction': groups['ineligible']['empty_cell'] / lookups if lookups else None,
            'local_memo_hits': row['events']['local_memo_hit'],
            'normalizer_work_visits': row['events']['normalizer_work_visit'],
            'publications': row['events']['store_published'],
            'closed_copy_rejections': row['events']['closed_copy_declined_opaque'],
            'nonzero_kinds': row['nonzero_kinds']})
    suite_rows = []
    for suite in SUITES:
        own = [r for r in summaries if r['suite'] == suite]
        suite_rows.append({'suite': suite, 'phases': len(own),
            'lookup_phases': sum(r['lookups'] > 0 for r in own),
            'hit_phases': sum(r['hits'] > 0 for r in own),
            'zero_lookup_phases': sum(r['lookups'] == 0 for r in own),
            'per_classified_process': assessment['per_suite_totals_per_classified_process'][suite]})
    contrasts = []
    for left, right in [('main', 'borrowed'), ('main', 'eligibility'), ('borrowed', 'eligibility')]:
        own = [r for r in assessment['public_comparisons']
               if (r['baseline_group'], r['candidate_group']) == (left, right)]
        assert len(own) == 68
        contrasts.append({'baseline_group': left, 'candidate_group': right, 'phases': len(own),
            'all_public_counts_equal_phases': sum(r['all_public_counts_equal'] for r in own),
            'by_suite': {suite: {'phases': sum(r['suite'] == suite for r in own),
                'equal': sum(r['suite'] == suite and r['all_public_counts_equal'] for r in own)} for suite in SUITES}})
    assert [r['all_public_counts_equal_phases'] for r in contrasts] == [27, 27, 68]
    observed = sorted({kind for r in phases for kind in r['nonzero_kinds']})
    hit_kinds = sorted({kind for r in phases for kind, values in r['classes'].items() if values['hit']})
    assert len(observed) == 11 and hit_kinds == ['Add', 'Sqrt']
    zero_events = [{'suite': r['suite'], 'phase': r['phase']} for r in phases if not any(r['events'].values())]
    assert len(zero_events) == 17
    cost = json.loads((PARENT / 'scalar-cost-assessment-v1.json').read_text())
    cost_interpretation = json.loads((PARENT / 'scalar-cost-interpretation-v1.json').read_text())
    assert cost_interpretation['assessment_sha256'] == sha(PARENT / 'scalar-cost-assessment-v1.json')
    focus_keys = [('quadratic', 'mixed_field_decline_first'), ('quadratic', 'pi_transport_warm'),
                  ('rational', 'opaque_linear_atom_first'), ('rational', 'opaque_linear_atom_warm'),
                  ('rational', 'opaque_linear_atom_cloned')]
    focus = []
    for suite, phase in focus_keys:
        count = next(r for r in summaries if (r['suite'], r['phase']) == (suite, phase))
        costs = [r for r in cost['timing_rows'] if (r['suite'], r['name']) == (suite, phase)]
        assert len(costs) == 3
        focus.append({'counts': count, 'previous_closed_timing_rows': costs})
    mixed = focus[0]
    assert mixed['counts']['lookups'] == mixed['counts']['empty_cell'] == 20480
    assert mixed['counts']['ineligible_empty_cell'] == 8192
    mixed_cost = next(r for r in mixed['previous_closed_timing_rows'] if r['baseline_group'] == 'borrowed')
    assert mixed_cost['candidate_higher_pairs'] == 6 and mixed_cost['median_paired_ratio'] > 1.08
    for row in focus[2:4]:
        assert row['counts']['lookups'] == row['counts']['empty_cell'] == 87186
        assert row['counts']['eligible_empty_cell'] == 50127
        assert row['counts']['ineligible_empty_cell'] == 37059
        assert row['counts']['hits'] == row['counts']['publications'] == row['counts']['closed_copy_rejections'] == 0
    result = {'status': 'all-observed-misses-are-empty-cell-returns-read-guard-lane-closed',
        'interpretation_is_post_observation': True, 'interpreter_sha256': sha(__file__),
        'assessment_sha256': sha(E / 'assessment-v2.json'),
        'prior_cost_assessment_sha256': sha(PARENT / 'scalar-cost-assessment-v1.json'),
        'prior_cost_interpretation_sha256': sha(PARENT / 'scalar-cost-interpretation-v1.json'),
        'failed_v1_sha256': sha(E / 'failure-v1.json'), 'commands_passed': 53, 'scalar_tests_passed': 881,
        'fixed_count_processes': 24, 'trace_windows': 544, 'classified_windows': 136,
        'distinct_classified_phases': 68, 'public_comparison_rows': 204,
        'all_witnesses_and_instrumented_parent_counts_preserved': True, 'historical_counter_phases_matched': 56,
        'all_phase_summaries': summaries, 'per_suite_summary': suite_rows, 'public_count_contrasts': contrasts,
        'node_kind_coverage': {'declared_kinds': 60, 'workload_kinds_observed': observed,
            'kinds_with_hits': hit_kinds, 'unclassified_public_lookups': 0,
            'initialized_miss_workload_phases': 0,
            'initialized_miss_positive_controls': 'The passing diagnostic control separately seeds valid approximation and tower payloads and observes initialized misses. Those controls are not workload samples.'},
        'all_zero_event_phases': zero_events, 'focus_rows': focus,
        'findings': [
            'Every miss in the fixed rational, quadratic and integer workloads returned at the original null evaluation-pointer branch. No miss acquired a cell read lock; hits still read the locked certificate.',
            'All ineligible lookups that the earlier guard aims to remove are empty-cell returns in these parent workloads. The count evidence therefore supplies no observed lock-avoidance benefit for that guard.',
            'Mixed-field first query has 20480 empty-cell returns, including 8192 ineligible Int reads. The old guard nevertheless has a 1.0800725321966862 paired median ratio versus its parent and is higher in all six closed cost pairs.',
            'Opaque rational first and warm queries each have 87186 empty-cell returns, split 50127 eligible and 37059 ineligible, with no hits, publications or closed-copy rejections.',
            'The earlier six zero-event quadratic construction phases remain zero; all four rational radical-Laurent transport phases also remain zero. Their old timing differences are not attributed to directly executed cache work.',
            'All public arithmetic counts are equal between borrowed and eligibility in all 68 phases; main differs from both in 41 phases. Instrumented-parent counts exactly match borrowed in every phase.'],
        'decision': [
            'Keep the entry eligibility guard rejected. Do not advance another read predicate, lock flag or branch-placement runtime variant on this evidence.',
            'Close this diagnostic as a count attribution result. Preserve all fixed samples and the V1 compilation failure; no samples are extended, replaced, filtered or pooled.',
            'Before another runtime prototype, audit a distinct source of executed work in the retained normalizer and its callers, including local memo ownership and result projection, against existing profiles and rejected experiments. Cache reads may not be moved behind memo or budget checks without a proof-preserving design.'],
        'limits': [
            'These are event counts, not cost shares, timing measurements or a significance test. The old timing samples retain their descriptive status.',
            'The workloads observe 11 of 60 node kinds. They do not establish that initialized misses never occur in other APIs, approximate queries, native boards or concurrent executions.',
            'Diagnostic correctness controls exercise initialized misses but do not estimate their frequency in real workloads.',
            'Instrumented counts do not establish a processor-frequency, instruction-cache, branch-prediction or code-layout explanation for prior timing differences.',
            'No new performance candidate, allocation measurement, native gate or runtime admission is performed.',
            'Main runtime and all 35 original pending native paths remain unchanged. Both original native limits and the full 39-property, 14-target and crate-future-work scope remain open.'],
        'new_samples': 0, 'new_native_gates_run': False,
        'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    dump(destination, result)
    print(json.dumps({'status': result['status'], 'phases': 68, 'initialized_miss_phases': 0,
        'workload_node_kinds_observed': len(observed), 'historical_phases_match': 56,
        'public_equal_phase_counts': [r['all_public_counts_equal_phases'] for r in contrasts],
        'new_runtime_candidate': False, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
