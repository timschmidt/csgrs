"""Record the rejection without extending the completed fixed sample set."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'decision-v1.json'
    assert not destination.exists()
    assessment = json.loads((E / 'cost-assessment-v1.json').read_text())
    qualification = json.loads((E / 'correctness-v1.json').read_text())
    assert assessment['all_mathematical_witnesses_and_representation_inventories_match']
    assert qualification['all_30_commands_passed']
    assert qualification['same_coefficient_size_as_baseline']
    summary = {}
    for comparison in assessment['comparisons']:
        key = comparison['variant'] + '_vs_' + comparison['reference']
        timing = [r for r in assessment['timing_rows'] if r['comparison'] == key]
        memory = [r for r in assessment['memory_rows'] if r['comparison'] == key]
        sizes = [r['delta_bytes'] for r in assessment['binary_sizes'] if r['comparison'] == key]
        assert len(timing) == len(memory) == 68 and len(sizes) == 6
        fields = {}
        for field in memory[0]['fields']:
            rows = [r['fields'][field] for r in memory]
            assert all(len(set(r['block_deltas'])) == 1 for r in rows)
            fields[field] = {
                'reduced_rows': sum(r['median_block_delta'] < 0 for r in rows),
                'increased_rows': sum(r['median_block_delta'] > 0 for r in rows),
                'min_delta': min(r['median_block_delta'] for r in rows),
                'max_delta': max(r['median_block_delta'] for r in rows),
                'all_five_deltas_equal': True,
            }
        summary[key] = {
            'timing_rows': len(timing), 'memory_rows': len(memory),
            'all_five_slower_rows': sum(r['slower_blocks'] == 5 for r in timing),
            'at_least_four_slower_rows': sum(r['slower_blocks'] >= 4 for r in timing),
            'all_five_faster_rows': sum(r['faster_blocks'] == 5 for r in timing),
            'at_least_four_faster_rows': sum(r['faster_blocks'] >= 4 for r in timing),
            'ratio_of_medians_above_one_rows': sum(r['ratio_of_medians'] > 1 for r in timing),
            'memory_fields': fields,
            'probe_binary_delta_bytes_range': [min(sizes), max(sizes)],
        }
    assert [summary[k]['all_five_slower_rows'] for k in summary] == [2, 13, 9]
    assert all(f['increased_rows'] == 0 for s in summary.values() for f in s['memory_fields'].values())
    selected = [('rational', 'conjugate_product_warm'),
                ('rational', 'opaque_linear_atom_cloned'),
                ('rational', 'quadratic_reciprocal_construction'),
                ('rational', 'quadratic_reciprocal_first')]
    decision = {
        'status': 'reject-radical-union-storage-feature-only-change-not-admitted',
        'qualification_sha256': sha(E / 'correctness-v1.json'),
        'cost_assessment_sha256': sha(E / 'cost-assessment-v1.json'),
        'cost_result_sha256': sha(E / 'cost-v1/result.json'),
        'decision_runner_sha256': sha(Path(__file__)),
        'decision': 'Reject the radical-only union-storage candidate. It preserves the seven measured layouts and reduces requested allocations, but repeated scalar slowdowns remain against both controls. The performance-before-memory ladder does not support admission on these data. The feature-only control also has mixed costs, unchanged requested memory and larger probes; it is not admitted as a separate optimization.',
        'closed_sample_set': {
            'measurement_processes': 90, 'preflight_processes': 18, 'blocks': 5,
            'timing_inputs_per_phase': 4096, 'allocation_inputs_per_phase': 1000,
            'extended': False, 'all_witnesses_and_inventories_match': True,
            'comparisons_reuse_same_samples': True,
        },
        'summary': summary,
        'selected_adverse_and_variable_rows': [r for r in assessment['timing_rows']
                                              if (r['suite'], r['name']) in selected],
        'layouts': qualification['layouts'],
        'actual_smallvec_scalar_features': qualification['smallvec_features'],
        'limits': assessment['limits'] + [
            'Feature-only reciprocal timings have substantial within-source variation. For reciprocal construction, the ratio of medians favors the radical candidate by 0.6058 versus the feature control, while the median paired ratio is 0.9312. These are different statistics, not independent evidence of a large speedup. All samples and adverse comparisons are retained.',
            'The same-size layout and requested-heap savings establish that union storage removes the earlier nonunion memory penalties in these scalar probes. They do not identify the cause of the remaining timing differences.',
            'This is a retention decision for the measured workloads, not a universal claim about inline storage. No native gate, native cost control, full-report replay or production CLI build was executed for these rejected variants.',
            'The uninstrumented parent still fails the original board assertion at 133.31248412 worker-CPU seconds against 120. Its separate 60-second copper-neck check-body assertion passes. These are parent results.',
        ],
        'next_step': {
            'status': 'source-review-pending',
            'task': 'Keep the parent and all completed storage trials frozen. Review ownership at the recursive closed-certificate copy boundary for a way to avoid copying unchanged radical arrays without changing polynomial arithmetic, node ownership, recognition budgets or the original retry. Establish mutation and release requirements before preparing any candidate.',
            'motivation': 'The earlier diagnostic observed 35,239,829 recursive closed-certificate copies, including 30,843,135 single-radical copies. This is call coverage, not an allocation count or measured speedup. Inline-slot experiments have now failed their scalar cost gates despite heap savings.',
            'candidate_prepared': False,
        },
        'runtime_admitted': False, 'native_runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'status': decision['status'], 'comparisons': len(summary),
                      'runtime_admitted': False}))


if __name__ == '__main__':
    main()
