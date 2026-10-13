"""Carry mixed scalar costs forward to native investigation, without admission."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'scalar-decision-v1.json'
    assert not destination.exists()
    assessment = json.loads((E / 'cost-assessment-v1.json').read_text())
    qualification = json.loads((E / 'correctness-v1.json').read_text())
    assert qualification['all_22_commands_passed']
    assert assessment['all_mathematical_witnesses_and_representation_inventories_match']
    timing, memory = assessment['timing_rows'], assessment['memory_rows']
    assert len(timing) == len(memory) == 68
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
    sizes = [r['delta_bytes'] for r in assessment['binary_sizes']]
    summary = {
        'timing_rows': 68, 'memory_rows': 68,
        'all_five_slower_rows': sum(r['slower_blocks'] == 5 for r in timing),
        'at_least_four_slower_rows': sum(r['slower_blocks'] >= 4 for r in timing),
        'all_five_faster_rows': sum(r['faster_blocks'] == 5 for r in timing),
        'at_least_four_faster_rows': sum(r['faster_blocks'] >= 4 for r in timing),
        'ratio_of_medians_above_one_rows': sum(r['ratio_of_medians'] > 1 for r in timing),
        'memory_fields': fields, 'probe_binary_delta_bytes_range': [min(sizes), max(sizes)],
    }
    assert summary['all_five_slower_rows'] == 3 and summary['all_five_faster_rows'] == 7
    assert all(row['increased_rows'] == 0 for row in fields.values())
    decision = {
        'decision': 'advance-closed-certificate-memo-to-native-investigation-with-scalar-regressions-open',
        'decision_runner_sha256': sha(Path(__file__)),
        'cost_assessment_sha256': sha(E / 'cost-assessment-v1.json'),
        'cost_result_sha256': sha(E / 'cost-v1/result.json'),
        'correctness_sha256': sha(E / 'correctness-v1.json'),
        'qualification': qualification['qualification'],
        'closed_sample_set': {'measurement_processes': 60, 'preflight_processes': 12,
                              'blocks': 5, 'timing_inputs_per_phase': 4096,
                              'allocation_inputs_per_phase': 1000, 'extended': False,
                              'all_witnesses_and_inventories_match': True},
        'summary': summary,
        'slower_in_all_five_blocks': [r for r in timing if r['slower_blocks'] == 5],
        'slower_in_at_least_four_blocks': [r for r in timing if r['slower_blocks'] >= 4],
        'faster_in_all_five_blocks': [r for r in timing if r['faster_blocks'] == 5],
        'rationale': 'The exact source change demonstrably removes a redundant closed-result memo copy while preserving opaque ownership, original memo reuse, retry behavior and every tested fact. Twenty-one phases reduce allocation calls/bytes, seventeen reduce peak requested bytes, and all six probe executables are smaller. Timing is mixed: seven phases are faster in every block and twenty-eight in at least four, but three phases are slower in every block and seven in at least four. This supports investigating native impact, not retaining the change or declaring the scalar performance concerns resolved.',
        'unresolved_costs': [
            'Affine-surd first and warm queries are slower in all five pairs: ratios of medians 1.107208 and 1.089591. Opaque-decline construction is also slower in all five pairs, ratio of medians 1.025741. The full arrays and ranges remain attached.',
            'Rational radical-Laurent construction is slower in four pairs, with a ratio of medians 1.153210 and variable samples. It is an adverse result, not discarded as noise.',
            'Some large apparent integer gains depend on variable baseline timings. Signed-word construction has ratio of medians 0.625348 but median paired ratio 1.041420 and is slower in three pairs. These statistics do not establish a large speedup.',
            'All three all-five-slower phases have identical requested-memory counters. The allocation savings elsewhere do not explain or excuse these timing costs. No causal attribution or statistical significance is established.',
        ],
        'retention_requirement': 'Review native results together with every scalar adverse row and inherited native tradeoff. A native timing improvement alone does not dismiss scalar regressions. Do not admit a lower-rung improvement at the expense of a higher rung; meaningful regressions must be resolved before retention.',
        'limits': assessment['limits'] + [
            'Requested retained-byte deltas match in every phase and all five pairs. This does not prove equal total memory or eventual reclamation.',
            'The uninstrumented parent fails its original board assertion at 133.31248412 worker-CPU seconds against 120; the separate 60-second neck check-body assertion passes. These are parent results, not candidate measurements.',
        ],
        'next_step': 'Run strict four-crate all-feature/all-target Clippy, full all-feature solver/curve/DRC suites, warning-denied four-crate docs and both original DRC assertions from a bound test executable in a fresh native target. Keep the two gates separate from the combined DRC command. Then use the fixed native controls below, complete unnormalized report identity and production CLI sizes if investigation continues. No runtime admission follows from qualification alone.',
        'native_fixed_block_order': [['baseline', 'candidate'], ['candidate', 'baseline'],
                                     ['baseline', 'candidate'], ['candidate', 'baseline'],
                                     ['baseline', 'candidate']],
        'native_cost_protocol': {'endpoint_time_inputs': 8192, 'endpoint_allocation_inputs': 1000,
                                 'endpoint_measurement_processes': 20, 'broad_measurement_processes': 10,
                                 'broad_witness_rows_per_process': 231,
                                 'final_abort_processes': 6, 'final_abort_rows_per_process': 210,
                                 'final_abort_blocks': 'first three predeclared blocks',
                                 'sample_extension_planned': False},
        'runtime_admitted': False, 'native_runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'decision': decision['decision'], 'summary': summary,
                      'runtime_admitted': False}))


if __name__ == '__main__':
    main()
