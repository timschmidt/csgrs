"""Post-observation admission review of every scalar contrast and adverse row."""
import json

from eligibility_cost_common_v1 import E, COST_PREP, sha, dump, verify_costs

CONTRASTS = [('main', 'candidate'), ('borrowed', 'candidate'), ('main', 'borrowed')]


def main():
    destination = E / 'scalar-cost-interpretation-v1.json'
    assert not destination.exists()
    prep = json.loads((COST_PREP / 'protocol.json').read_text())
    verify_costs(prep, full=True)
    path = E / 'scalar-cost-assessment-v1.json'
    assessment = json.loads(path.read_text())
    assert assessment['status'] == 'closed-three-group-scalar-cost-comparison-complete'
    assert assessment['assessor_sha256'] == sha(E / 'assess_scalar_costs_v1.py')
    assert all(sha(E / p) == h for p, h in assessment['artifacts_sha256'].items())
    assert assessment['all_allocation_fields_repeat']
    assert assessment['all_original_group_allocation_fields_match_closed_history']
    summaries = []
    for left, right in CONTRASTS:
        timings = [r for r in assessment['timing_rows'] if (r['baseline_group'], r['candidate_group']) == (left, right)]
        memory = [r for r in assessment['memory_rows'] if (r['baseline_group'], r['candidate_group']) == (left, right)]
        sizes = [r for r in assessment['binary_sizes'] if (r['baseline_group'], r['candidate_group']) == (left, right)]
        assert len(timings) == len(memory) == 68 and len(sizes) == 6
        compact = lambda row: {k: row[k] for k in ['suite', 'name', 'median_paired_ratio', 'paired_ratios',
            'baseline_median_ns', 'candidate_median_ns', 'candidate_higher_pairs', 'candidate_lower_pairs']}
        deltas = [{'suite': r['suite'], 'name': r['name'], 'fields': {key: field['paired_deltas'][0]
                   for key, field in r['fields'].items()}} for r in memory]
        assert all(len(set(field['paired_deltas'])) == 1 for row in memory for field in row['fields'].values())
        fields = list(deltas[0]['fields'])
        summaries.append({'baseline_group': left, 'candidate_group': right,
            'timing_rows': 68, 'higher_all_six': sum(r['candidate_higher_pairs'] == 6 for r in timings),
            'lower_all_six': sum(r['candidate_lower_pairs'] == 6 for r in timings),
            'higher_at_least_five': sum(r['candidate_higher_pairs'] >= 5 for r in timings),
            'higher_median': sum(r['median_paired_ratio'] > 1 for r in timings),
            'all_higher_median_rows': [compact(r) for r in sorted(timings, key=lambda r: -r['median_paired_ratio']) if r['median_paired_ratio'] > 1],
            'all_higher_at_least_five_rows': [compact(r) for r in timings if r['candidate_higher_pairs'] >= 5],
            'all_lower_all_six_rows': [compact(r) for r in timings if r['candidate_lower_pairs'] == 6],
            'memory_rows': 68, 'all_memory_deltas': deltas,
            'all_memory_fields_equal_rows': sum(all(d == 0 for d in r['fields'].values()) for r in deltas),
            'memory_increase_rows': sum(any(d > 0 for d in r['fields'].values()) for r in deltas),
            'memory_field_counts': {f: {'higher': sum(r['fields'][f] > 0 for r in deltas),
                                      'lower': sum(r['fields'][f] < 0 for r in deltas),
                                      'equal': sum(r['fields'][f] == 0 for r in deltas)} for f in fields},
            'binary_sizes': sizes, 'larger_binaries': sum(r['candidate_minus_baseline_bytes'] > 0 for r in sizes)})
    lookup = {(r['baseline_group'], r['candidate_group'], r['suite'], r['name']): r for r in assessment['timing_rows']}
    focus = [lookup['borrowed', 'candidate', 'quadratic', name] for name in [
        'mixed_field_decline_first', 'pi_transport_warm', 'affine_surd_first', 'affine_surd_warm']]
    focus += [lookup[group, 'candidate', 'rational', 'opaque_linear_atom_' + phase]
              for group in ['main', 'borrowed'] for phase in ['construction', 'first', 'warm', 'cloned']]
    memory_lookup = {(r['baseline_group'], r['candidate_group'], r['suite'], r['name']): r for r in assessment['memory_rows']}
    signed_word = memory_lookup['main', 'candidate', 'integer', 'signed_word_boundary_construction']
    assert all(all(d == 0 for d in field['paired_deltas']) for field in signed_word['fields'].values())
    assert summaries[1]['all_memory_fields_equal_rows'] == 68
    decision = {
        'status': 'lookup-eligibility-preserves-proofs-but-scalar-costs-block-retention',
        'interpreter_sha256': sha(__file__), 'assessment_sha256': sha(path),
        'qualification_assessment_sha256': sha(E / 'qualification-assessment-v1.json'),
        'synchronization_clarification_sha256': sha(E / 'source-audit-synchronization-clarification-v1.json'),
        'commands_passed': 133, 'fixed_cost_processes': 108, 'preflight_processes': 18,
        'comparison_rows': {'timing': 204, 'memory': 204, 'binary_size': 18},
        'all_witnesses_and_representations_match': True, 'all_allocation_fields_repeat': True,
        'all_original_group_allocation_fields_match_closed_history': True,
        'complete_contrasts': summaries, 'focus_rows': focus,
        'signed_word_construction_matches_main_all_four_allocation_fields': signed_word,
        'decision': [
            'Do not retain this unconditional entry predicate as an admitted runtime optimization. All 879 scalar tests pass, but the first mixed-field-decline query is higher in all six pairs versus the otherwise identical borrowed-integer runtime, with median paired ratio 1.0800725321966862.',
            'The pi-transport warm query is also higher in all six pairs versus borrowed, median paired ratio 1.007918755858952. Eight rows are higher in at least five pairs and fifteen have higher paired medians; every one is retained above.',
            'The candidate also has eleven quadratic rows lower in all six pairs versus borrowed, including affine-surd first/warm median paired ratios 0.8443670003018569 and 0.8840451552563088. These gains do not remove the adverse mixed-field result.',
            'Opaque rational first/warm/cloned queries have lower medians versus borrowed and are lower in five of six pairs, but remain higher than main in all six pairs: median paired ratios 1.086402100248491, 1.0755643400542647 and 1.068538520434346.',
            'All four allocation fields are identical to borrowed in every one of the 68 scalar phases. The signed-word construction repair remains exactly equal to main. Twenty inherited scalar rows still have increased fields versus main, including 40000 more retained/peak bytes in each signed-word and wide-integer cloned phase.',
            'All six probe binaries grow versus borrowed (240 or 248 bytes) and versus main (136 through 816 bytes). There is no allocation or size improvement to offset the measured timing tradeoff.',
            'No broader arc/metadata/operation samples or native gates are opened for this rejected variant. Their obligations remain for any later admission candidate, with unchanged original native deadlines and CLI evidence requirements.'
        ],
        'limits': [
            'This is an admission judgment from the full fixed observations, not a statistical significance test or universal speed claim.',
            'No sample is filtered, replaced, extended or pooled. Both substantial and small adverse rows and every beneficial row remain in the assessment.',
            'A lower construction time where prior instrumentation recorded zero cache events is not attributed to removed cache reads; timing cause and code-layout/frequency effects remain unresolved.',
            'The existing cache diagnostic does not split misses by immutable kind or empty versus initialized evaluation cell. It does not quantify how much lookup/lock work this branch removes.',
            'The audit clarification corrects overbroad wording about synchronization traffic; all original frozen protocols and sources remain unchanged.',
            'The full 39-property, 14-target and crate-future-work scope remains open.'
        ],
        'new_runtime_admission': False, 'new_native_gates_run': False,
        'runtime_candidate': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    dump(destination, decision)
    print(json.dumps({'status': decision['status'], 'contrasts': [
        {k: r[k] for k in ['baseline_group', 'candidate_group', 'higher_all_six', 'lower_all_six',
                           'higher_at_least_five', 'higher_median', 'memory_increase_rows', 'larger_binaries']}
        for r in summaries], 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
