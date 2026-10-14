"""Summarize all closed hardware observations without new executions or exclusions."""
import json
import statistics

from common_v2 import E, ROOT, sha, dump


def main():
    destination = E / 'decision-v1.json'
    assert not destination.exists()
    assessment = json.loads((E / 'assessment-v2.json').read_text())
    assert assessment['status'] == 'hardware-diagnostic-complete'
    assert assessment['comparisons_sha256'] == sha(E / 'comparisons-v2.json')
    assert assessment['empty_window_accounting_sha256'] == sha(E / 'empty-window-accounting-v2.json')
    assert assessment['result_sha256'] == sha(E / 'run-v2/result.json')
    result = json.loads((E / 'run-v2/result.json').read_text())
    assert result['artifact_sha256']['samples.json'] == sha(E / 'run-v2/samples.json')
    comparisons = json.loads((E / 'comparisons-v2.json').read_text())
    overhead = json.loads((E / 'empty-window-accounting-v2.json').read_text())
    samples = json.loads((E / 'run-v2/samples.json').read_text())
    assert len(comparisons) == len(overhead) == 96
    metrics = ['user_instructions', 'user_cycles', 'thread_cpu_ns', 'wall_ns']
    matrix, variation, directions = [], [], []
    for row in comparisons:
        aggregate = row['aggregate_only_if_all_six_usable']
        assert row['all_six_pairs_usable'] and aggregate is not None
        matrix.append({key: row[key] for key in ['baseline', 'candidate', 'phase']} | {
            'metrics': {key: {field: aggregate[key][field] for field in
                ['ratio_of_medians', 'paired_ratios', 'candidate_higher_pairs', 'candidate_lower_pairs']}
                for key in metrics}})
    for row in overhead:
        variation.append({'variant': row['variant'], 'phase': row['phase'],
            'metrics': {key: {'six_work_observations': [w[key] for w in row['work']],
                'minimum': min(w[key] for w in row['work']),
                'maximum': max(w[key] for w in row['work']),
                'max_over_min': max(w[key] for w in row['work']) / min(w[key] for w in row['work'])}
                for key in metrics}})
    for left, right in [('main', 'full-cache'), ('main', 'no-persistent'), ('no-persistent', 'full-cache')]:
        own = [r for r in matrix if (r['baseline'], r['candidate']) == (left, right)]
        assert len(own) == 32
        directions.append({'baseline': left, 'candidate': right,
            'metrics': {key: {
                'higher_all_six': [r['phase'] for r in own if r['metrics'][key]['candidate_higher_pairs'] == 6],
                'lower_all_six': [r['phase'] for r in own if r['metrics'][key]['candidate_lower_pairs'] == 6],
                'mixed_or_equal': [r['phase'] for r in own if max(r['metrics'][key]['candidate_higher_pairs'],
                                                             r['metrics'][key]['candidate_lower_pairs']) != 6]}
                for key in metrics}})
    dump(E / 'complete-phase-matrix-v1.json', matrix)
    dump(E / 'within-binary-variation-v1.json', variation)
    dump(E / 'all-phase-directions-v1.json', directions)
    focus = ['negated_scaled_surd_construction', 'multiradical_square_construction']
    old_path = ROOT / 'docs/evidence/2026-10-13-combined-native-admission/library-assessment-v1.json'
    old = json.loads(old_path.read_text())
    historical = [r for r in old['timing_rows'] if r['suite'] == 'quadratic' and r['name'] in focus]
    assert len(historical) == 2 and all(r['candidate_higher_pairs'] == 5 for r in historical)
    maximum_overhead = {key: max(
        ({'variant': r['variant'], 'phase': r['phase'],
          'empty_over_work_ratio_of_medians': r['empty_to_work_ratio_of_medians'][key]} for r in overhead),
        key=lambda r: r['empty_over_work_ratio_of_medians']) for key in metrics}
    selected = [r for r in matrix if r['baseline'] == 'main' and r['candidate'] == 'full-cache' and r['phase'] in focus]
    widest_instructions = max(variation, key=lambda r: r['metrics']['user_instructions']['max_over_min'])
    decision = {
        'status': 'small-instruction-differences-and-large-within-binary-cycle-variation-cause-unresolved',
        'interpreter_sha256': sha(__file__),
        'input_sha256': {name: sha(E / name) for name in ['assessment-v2.json', 'comparisons-v2.json',
            'empty-window-accounting-v2.json', 'run-v2/samples.json', 'run-v2/result.json']},
        'derived_sha256': {name: sha(E / name) for name in ['complete-phase-matrix-v1.json',
            'within-binary-variation-v1.json', 'all-phase-directions-v1.json']},
        'fixed_processes': 18, 'work_windows': 576, 'empty_windows': 576, 'unusable_windows': 0,
        'complete_contrast_phase_rows': 96, 'all_witnesses_and_representation_inventories_match': True,
        'focus_current_observations': selected,
        'historical_timing_assessment': str(old_path.relative_to(ROOT)),
        'historical_timing_assessment_sha256': sha(old_path),
        'historical_timing_rows_preserved_without_pooling': historical,
        'maximum_empty_over_work_ratios': maximum_overhead,
        'largest_within_variant_phase_instruction_range': widest_instructions,
        'interpretation': [
            'In this instrumented driver, the two historically expensive full-cache construction phases have only small changes in instruction totals: negated-scaled is lower in all six pairs; multiradical-square is higher in all six. Neither has approximately 65 percent more retired instructions.',
            'Negated-scaled construction still has higher cycles, CPU and wall in every pair. Multiradical-square cycles/CPU/wall split three higher and three lower. All adverse rows across all phases and contrasts are retained.',
            'Each variant/phase repeats its instruction total closely, while many show substantially larger cycle and elapsed variation with the same binary. This demonstrates observed variability, not its cause or statistical significance.',
            'The earlier timing-only driver and this hardware driver have different wrappers and machine code. These data cannot invalidate, replace, average away or explain the two closed approximately 1.65x median timing regressions.',
            'Current snapshots and user-only cycles do not establish prior frequency, contention, code layout, cache state or host conditions. The causal attribution of historical costs remains open.',
            'The original cache-free 136.717191044-second worker-CPU board failure against 120 seconds remains a failure. No new native gate was run and no runtime candidate is admitted.'
        ],
        'closed_measurements': True, 'no_further_samples_in_this_trial': True,
        'cause_of_prior_elapsed_costs_established': False,
        'next_step': 'Review the independently repeatable signed-word allocation increase at its integer conversion source; confirm the proposed mechanism before any new isolated runtime prototype. This does not resolve the construction or native admission blockers.',
        'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False,
    }
    dump(destination, decision)
    print(json.dumps({key: decision[key] for key in ['status', 'fixed_processes', 'unusable_windows',
                                                   'complete_contrast_phase_rows', 'runtime_admitted']}))


if __name__ == '__main__':
    main()
