"""Reject the measured shared-storage version without extending its sample set."""
import hashlib
import json
from pathlib import Path

from qualify_v1 import verify

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'decision-v1.json'
    assert not destination.exists()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    verify(preparation)
    assessment = json.loads((E / 'cost-assessment-v1.json').read_text())
    qualification = json.loads((E / 'correctness-v1.json').read_text())
    assert qualification['all_26_commands_passed']
    assert assessment['all_mathematical_witnesses_and_representation_inventories_match']
    for name, digest in assessment['evidence_sha256'].items():
        assert sha(E / name) == digest, name
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
    assert summary['all_five_slower_rows'] == 3 and summary['all_five_faster_rows'] == 13
    assert fields['allocation_calls']['increased_rows'] == 30
    assert fields['retained_bytes_delta']['increased_rows'] == 15
    slow = [r for r in timing if r['slower_blocks'] == 5]
    assert [(r['suite'], r['name']) for r in slow] == [
        ('quadratic', 'conjugate_then_affine_' + phase) for phase in ['first', 'warm', 'cloned']]
    selected = [(r['suite'], r['name']) for r in slow] + [
        ('rational', 'conjugate_product_warm'), ('rational', 'quadratic_reciprocal_warm'),
        ('quadratic', 'pi_transport_warm'), ('quadratic', 'mixed_field_decline_first'),
        ('integer', 'wide_integer_affine_warm'),
    ]
    assert all(not (E / name).exists() for name in [
        'native-validation-v1', 'native-controls-v1', 'report-replay-v1'])
    decision = {
        'status': 'reject-shared-radical-storage-on-scalar-performance-and-memory-costs',
        'decision_runner_sha256': sha(Path(__file__)),
        'cost_assessment_sha256': sha(E / 'cost-assessment-v1.json'),
        'correctness_sha256': sha(E / 'correctness-v1.json'),
        'decision': 'Reject this shared-radical-storage version. It preserves the tested values, aliases, ownership and P4 facts, but conjugate-then-affine first, warm and cloned queries are slower in every fixed pair. Other repeated adverse phases, increased allocation calls and retained memory, and larger probes remain visible. Smaller coefficient layouts and thirteen all-five-faster phases do not resolve those costs under the governing ladder. No native run is advanced for this source.',
        'closed_sample_set': {
            'measurement_processes': 60, 'preflight_processes': 12, 'blocks': 5,
            'timing_inputs_per_phase': 4096, 'allocation_inputs_per_phase': 1000,
            'extended': False, 'all_witnesses_and_inventories_match': True,
        },
        'summary': summary, 'layouts': qualification['layouts'],
        'memo_inventories': qualification['memo_inventories'], 'sharing': qualification['sharing'],
        'slower_in_all_five_blocks': slow,
        'slower_in_at_least_four_blocks': [r for r in timing if r['slower_blocks'] >= 4],
        'faster_in_all_five_blocks': [r for r in timing if r['faster_blocks'] == 5],
        'selected_adverse_memory_rows': [r for r in memory if (r['suite'], r['name']) in selected],
        'limits': assessment['limits'] + [
            'Conjugate-then-affine first/warm/cloned ratios of medians are 1.116639, 1.176402 and 1.109723; median paired ratios are 1.107542, 1.171850 and 1.098514. The first/warm phases each add 5,048 requested allocations per 1,000 inputs; cloned adds 3,048 and 72,000 retained requested bytes.',
            'Allocation calls decrease in only three phases and increase in thirty. Allocated bytes decrease in thirty-two and increase in nineteen. Retained bytes decrease in six and increase in fifteen; peak bytes decrease in twenty and increase in twenty-eight. Every delta repeats across all five pairs. Smaller per-coefficient fields therefore do not establish lower total heap cost.',
            'All six probe executables grow by 1,584 to 2,616 bytes. Production CLI size is not measured.',
            'Some baseline samples vary substantially, including several fourth-block samples and negated-scaled-surd construction. No block is removed. That construction phase has ratio of medians 0.645156 but individual paired ratios vary widely; it does not establish a general thirty-five-percent improvement.',
            'No native validation, original native gate, endpoint/broad/abort control, complete-report replay or production CLI build was executed for this rejected source. The uninstrumented parent still fails the original board assertion at 133.31248412 worker-CPU seconds against 120 and passes the separate original 60-second neck check-body assertion.',
            'First-query phases use fresh graphs in processes that may already share constants; cloned-query phases do not include cloning. Requested retained bytes are phase-relative and are not a proof of eventual reclamation.',
        ],
        'next_step': {
            'status': 'source-review-pending',
            'task': 'Keep every completed source and sample set frozen. Review a distinct design that keeps mutable scratch radical arrays as owned Vecs and shares only immutable retained payloads. A write to a shared read view would produce an ordinary owned Vec rather than another reference-counted buffer. Establish publication, empty-state, consuming iteration, alias isolation, release and layout requirements before preparing a candidate from the unchanged endpoint/P4 parent.',
            'motivation': 'This version shares all nonempty temporary arrays, including newly built and mutable results. The source uses Arc::make_mut for writes and allocates a shared header for every new nonempty buffer; the measured call increases show that saving read copies did not reduce allocation traffic overall. Separating scratch ownership from retained sharing is a hypothesis, not an attributed cause for every adverse row or an established improvement.',
            'constraints': [
                'Do not combine the rejected memo-publication or inline-storage changes into the next baseline.',
                'Preserve Computable owner counts for opaque atoms, arithmetic/insertion order, all original bounds and mandatory original retry after releasing the first memo.',
                'A two-form storage enum may enlarge coefficient layouts; measure every layout, requested-memory field and probe size rather than hiding that tradeoff.',
                'Reuse mathematical alias, concurrency, cache-frontier and P4 controls; explicitly document any changed pointer-sharing inventory definition.',
            ],
            'candidate_prepared': False,
        },
        'source_parent_main_unchanged': True,
        'runtime_admitted': False, 'native_runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'status': decision['status'], 'summary': summary, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
