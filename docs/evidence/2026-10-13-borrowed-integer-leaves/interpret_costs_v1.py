"""Describe the closed cost trial without adding or selecting any samples."""
import json

from cost_common_v1 import E, ROOT, PARENT, sha, dump


def main():
    destination = E / 'library-interpretation-v1.json'
    assert not destination.exists()
    assessment_path = E / 'library-assessment-v1.json'
    assessment = json.loads(assessment_path.read_text())
    assert assessment['commands_passed'] == 254
    assert assessment['all_allocation_fields_repeat_within_each_group']
    assert len(assessment['timing_rows']) == 1095
    assert len(assessment['memory_rows']) == 402
    assert len(assessment['binary_sizes']) == 33
    assert all(sha(E / p) == h for p, h in assessment['artifacts_sha256'].items())

    historical_path = PARENT / 'library-assessment-v1.json'
    historical = json.loads(historical_path.read_text())
    old_memory = {(r['suite'], r['name']): r for r in historical['memory_rows']}
    current_memory = [r for r in assessment['memory_rows']
                      if (r['baseline_group'], r['candidate_group']) == ('main', 'full-cache')]
    assert len(old_memory) == len(current_memory) == 134
    for row in current_memory:
        previous = old_memory[row['suite'], row['name']]
        assert row['iterations'] == previous['iterations']
        for field, value in row['fields'].items():
            for group in ['baseline', 'candidate']:
                assert set(value[group]) == set(previous['fields'][field][group])

    qualification_path = E / 'integer-allocation-comparisons-v1.json'
    qualification = json.loads(qualification_path.read_text())
    integer_rows = {(r['baseline_group'], r['candidate_group'], r['name']): r
                    for r in assessment['memory_rows'] if r['suite'] == 'integer'}
    assert len(integer_rows) == len(qualification) == 36
    for row in qualification:
        current = integer_rows[row['baseline'], row['candidate'], row['phase']]
        for field, value in row['fields'].items():
            for group in ['baseline', 'candidate']:
                assert set(value[group]) == set(current['fields'][field][group])

    summaries = []
    for left, right in [('main', 'candidate'), ('full-cache', 'candidate'), ('main', 'full-cache')]:
        def own(rows):
            return [r for r in rows if (r['baseline_group'], r['candidate_group']) == (left, right)]

        timing = own(assessment['timing_rows'])
        memory = own(assessment['memory_rows'])
        sizes = own(assessment['binary_sizes'])
        assert (len(timing), len(memory), len(sizes)) == (365, 134, 11)
        memory_deltas = [{
            'suite': r['suite'], 'name': r['name'], 'iterations': r['iterations'],
            'candidate_minus_baseline': {
                field: int(v['candidate_median'] - v['baseline_median'])
                for field, v in r['fields'].items()},
        } for r in memory]
        fields = list(memory[0]['fields'])
        memory_counts = {field: {
            'higher': sum(r['candidate_minus_baseline'][field] > 0 for r in memory_deltas),
            'equal': sum(r['candidate_minus_baseline'][field] == 0 for r in memory_deltas),
            'lower': sum(r['candidate_minus_baseline'][field] < 0 for r in memory_deltas),
        } for field in fields}
        summaries.append({
            'baseline_group': left, 'candidate_group': right,
            'timing_rows': len(timing),
            'higher_all_six': sum(r['candidate_higher_pairs'] == 6 for r in timing),
            'lower_all_six': sum(r['candidate_lower_pairs'] == 6 for r in timing),
            'higher_at_least_five': sum(r['candidate_higher_pairs'] >= 5 for r in timing),
            'higher_median': sum(r['ratio_of_medians'] > 1 for r in timing),
            'display_rule': 'Include every row higher in all six pairs, with all raw values. This descriptive subset is not a significance test or an admission threshold; all 365 rows remain in the assessment.',
            'all_six_higher_rows': [r for r in timing if r['candidate_higher_pairs'] == 6],
            'all_six_lower_rows': [r for r in timing if r['candidate_lower_pairs'] == 6],
            'memory_field_counts': memory_counts,
            'all_memory_increases': [r for r in memory_deltas
                                     if any(v > 0 for v in r['candidate_minus_baseline'].values())],
            'all_memory_changes': [r for r in memory_deltas
                                  if any(v != 0 for v in r['candidate_minus_baseline'].values())],
            'binary_sizes': sizes,
        })
    assert [(s['higher_all_six'], s['lower_all_six'], s['higher_at_least_five'])
            for s in summaries] == [(15, 51, 58), (3, 11, 49), (24, 68, 70)]
    assert [len(s['all_memory_increases']) for s in summaries] == [29, 0, 30]
    helper = summaries[1]
    assert len(helper['all_memory_changes']) == 12
    assert helper['memory_field_counts']['allocation_calls']['equal'] == 134
    assert helper['memory_field_counts']['allocated_bytes']['lower'] == 12
    assert helper['memory_field_counts']['retained_bytes_delta']['lower'] == 6
    assert helper['memory_field_counts']['peak_bytes_delta']['lower'] == 8
    assert all(r['candidate_minus_baseline_bytes'] > 0 for r in helper['binary_sizes'])

    focus_names = {
        ('quadratic', 'negated_scaled_surd_construction'),
        ('quadratic', 'multiradical_square_construction'),
        ('rational', 'opaque_linear_atom_warm'),
    }
    history_context = []
    for suite, name in sorted(focus_names):
        old = next(r for r in historical['timing_rows'] if (r['suite'], r['name']) == (suite, name))
        fresh = [r for r in assessment['timing_rows'] if (r['suite'], r['name']) == (suite, name)]
        history_context.append({'suite': suite, 'name': name,
                                'closed_previous_five_pairs': old, 'fresh_six_pairs': fresh})
    result = {
        'status': 'allocation-repair-confirmed-combined-library-costs-block-retention',
        'interpreter_sha256': sha(__file__),
        'assessment': str(assessment_path.relative_to(ROOT)), 'assessment_sha256': sha(assessment_path),
        'historical_assessment_sha256': sha(historical_path),
        'qualification_comparisons_sha256': sha(qualification_path),
        'complete_comparisons': {'timing': 1095, 'memory': 402, 'binary_sizes': 33},
        'all_original_main_and_full_cache_allocation_fields_match_closed_history': True,
        'all_integer_allocation_fields_match_qualification': True,
        'contrasts': summaries,
        'historical_timing_context_no_pooling': history_context,
        'findings': [
            'All 254 commands, 30 preflights, nine abort controls and 198 cost processes passed. Every original witness, representation inventory and abort receipt matched.',
            'The borrowed-integer repair restores all four signed-word construction allocation fields to main. Against the previous full-cache build it changes no allocation-call counts, lowers requested bytes in 12 phases, lowers retained bytes in six phases and lowers peak bytes in eight; none increase.',
            'Against main, 51 timing rows are lower in every pair and 15 are higher in every pair. Quadratic negated-scaled and multiradical construction remain higher in all six pairs, with ratios of medians about 1.181 and 1.146. Opaque linear-atom warm queries remain higher in all six pairs at about 1.098.',
            'The helper comparison also contains adverse timing: rational non-endpoint arc warm queries, off-circle arc construction and IPC drill-diameter operations are higher in all six pairs. Allocation improvement does not establish a performance improvement.',
            'Twenty-nine phases still increase retained and peak bytes against main. Three also increase allocation calls and requested bytes: opaque linear-atom cloned, opaque cancellation cloned, and approximate non-endpoint first.',
            'All 11 measured binaries are larger than the previous full-cache build by 3176 to 4592 bytes. Against main, seven are smaller and four are larger; both integer probes and both quadratic probes are larger.',
            'Fresh main-versus-full-cache quadratic construction ratios are about 1.080 and 1.048, while the closed earlier trial observed about 1.656 and 1.646. Both sets remain evidence; no historical sample is replaced and no cause of the timing variation is established.',
        ],
        'decision': 'Do not admit the combined candidate. Preserve the qualified allocation repair as an isolated experiment, with unresolved library timing and inherited retention costs. Do not run a new native gate for this candidate while these library costs remain unresolved.',
        'next_action': 'Review the persistent-cache work reached by the unchanged opaque linear-atom probe, whose warm-query slowdown repeats in both fresh full-cache comparisons. Reuse existing diagnostic artifacts where their coverage suffices. Establish a concrete mechanism before proposing another runtime change; keep the construction timing cause open.',
        'native_gates_run': False,
        'native_limits_unchanged': {'full_board_worker_cpu_seconds': 120, 'copper_neck_check_body_wall_seconds': 60},
        'limits': assessment['limits'] + [
            'Timing direction counts and ratios are descriptive. They are not significance tests, universal speed claims or measured causal attribution.',
            'Matching source and features allowed reuse of the 876-test scalar qualification. The fresh cost rlib has different bytes from that qualification rlib, as recorded in scalar-build-identity.json.',
            'The complete 39-property, 14-target and crate-future-work goal remains active. Earlier native failures and application costs are unchanged.',
        ],
        'runtime_admitted': False, 'plan_complete': False,
    }
    dump(destination, result)
    print(json.dumps({'status': result['status'], 'contrasts': [
        {k: r[k] for k in ['baseline_group', 'candidate_group', 'higher_all_six', 'lower_all_six', 'higher_at_least_five']}
        for r in summaries], 'native_gates_run': False, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
