"""Reject the completed recursive-view trial while retaining every fixed sample."""
import hashlib
import json
from pathlib import Path

from qualify_v2 import verify

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'decision-v2.json'
    assert not destination.exists()
    verify(json.loads((E / 'preparation-v2/protocol.json').read_text()))
    assessment = json.loads((E / 'cost-assessment-v2.json').read_text())
    qualification = json.loads((E / 'correctness-v2.json').read_text())
    assert qualification['all_26_commands_passed']
    assert assessment['all_mathematical_witnesses_and_representation_inventories_match']
    for path, digest in assessment['evidence_sha256'].items():
        assert sha(E / path) == digest, path
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
        'timing_rows': len(timing), 'memory_rows': len(memory),
        'all_five_slower_rows': sum(r['slower_blocks'] == 5 for r in timing),
        'at_least_four_slower_rows': sum(r['slower_blocks'] >= 4 for r in timing),
        'all_five_faster_rows': sum(r['faster_blocks'] == 5 for r in timing),
        'at_least_four_faster_rows': sum(r['faster_blocks'] >= 4 for r in timing),
        'ratio_of_medians_above_one_rows': sum(r['ratio_of_medians'] > 1 for r in timing),
        'memory_fields': fields,
        'probe_binary_delta_bytes_range': [min(sizes), max(sizes)],
    }
    assert [summary[k] for k in ['all_five_slower_rows', 'at_least_four_slower_rows',
                                'all_five_faster_rows', 'at_least_four_faster_rows',
                                'ratio_of_medians_above_one_rows']] == [23, 43, 5, 13, 50]
    assert fields['allocation_calls']['reduced_rows'] == 9
    assert fields['allocation_calls']['increased_rows'] == 0
    assert fields['retained_bytes_delta']['increased_rows'] == 21
    assert fields['retained_bytes_delta']['reduced_rows'] == 0
    slow = [r for r in timing if r['slower_blocks'] == 5]
    for suite, prefix, phases in [
        ('rational', 'quadratic_reciprocal', ['first', 'warm', 'cloned']),
        ('quadratic', 'opaque_cancellation', ['first', 'warm']),
        ('integer', 'wide_integer_affine', ['first', 'warm', 'cloned']),
    ]:
        assert all(any(r['suite'] == suite and r['name'] == prefix + '_' + phase for r in slow) for phase in phases)
    assert not any(p.name.startswith(('native-', 'report-replay')) for p in E.iterdir())
    decision = {
        'status': 'reject-recursive-retained-polynomial-views-on-scalar-performance-and-retention-costs',
        'decision_runner_sha256': sha(Path(__file__)),
        'cost_assessment_sha256': sha(E / 'cost-assessment-v2.json'),
        'correctness_sha256': sha(E / 'correctness-v2.json'),
        'decision': 'Reject the measured version. It preserves tested values, ownership, aliases, arithmetic frontiers and P4 behavior, and avoids requested allocations in nine phases. However, twenty-three phases are slower in every fixed pair, retained/peak memory increases in twenty-one, and all probes grow. The supported scalar savings do not resolve the repeated adverse rows. This source does not advance to native validation or main runtime.',
        'closed_sample_set': {
            'measurement_processes': 60, 'preflight_processes': 12, 'blocks': 5,
            'timing_inputs_per_phase': 4096, 'allocation_inputs_per_phase': 1000,
            'extended': False, 'all_witnesses_and_inventories_match': True,
        },
        'summary': summary, 'layouts': qualification['layouts'],
        'retained_view_layout': qualification['retained_view_layout'],
        'memo_inventories': qualification['memo_inventories'],
        'ordinary_copy_sharing': qualification['sharing'],
        'slower_in_all_five_blocks': slow,
        'slower_in_at_least_four_blocks': [r for r in timing if r['slower_blocks'] >= 4],
        'faster_in_all_five_blocks': [r for r in timing if r['faster_blocks'] == 5],
        'limits': assessment['limits'] + [
            'Rational quadratic-reciprocal first/warm/cloned queries have ratios of medians 1.069731/1.076561/1.065831 and median paired ratios 1.091542/1.098132/1.076855. Every pair is adverse. No allocation calls are saved in those phases; first/cloned each add 16,000 allocated, retained and peak requested bytes per 1,000 inputs.',
            'Quadratic opaque-cancellation first/warm queries have ratios of medians and median paired ratios 1.091224/1.070052, adverse in every pair with unchanged requested-memory counters. Wide-integer first/warm/cloned queries are likewise adverse in every pair, with ratios of medians 1.091895/1.061425/1.067078 and median paired ratios 1.095414/1.063283/1.068369.',
            'Allocation calls decrease in nine phases, by up to 2,000 per 1,000 inputs, and never increase. Allocated bytes decrease in nine and increase in fifteen. Retained bytes increase in twenty-one and never decrease; peak bytes increase in twenty-one and decrease in one by 96 bytes. Every paired memory delta repeats five times. Private type sizes omit shared allocation headers and cannot establish total-memory improvement.',
            'Quadratic multiradical-square first/warm/cloned queries are faster in all five pairs, with ratios of medians 0.906561/0.923904/0.949935. Their requested allocations each decrease by 2,000 per 1,000 inputs. First/cloned still add 16,000 retained bytes. The other two all-five-faster rows are timer-scale radical-Laurent first/warm queries and are not treated as persuasive gains.',
            'All six probe executables grow by 4,352 to 4,368 bytes. Production CLI size was not measured.',
            'All timing samples and ranges remain visible, including large rational-suite candidate variation. Rational conjugate-product construction has ratio of medians 1.536804 but is adverse in four, not all five, pairs; neither that ratio nor any favorable outlier establishes an application effect. No sample is discarded and no significance claim is made.',
            'First-query phases use fresh graphs in processes that may already share constants; cloned-query phases do not include cloning. Requested retained bytes are relative to each phase and do not measure eventual reclamation or native C heaps, allocator bookkeeping and stack frames.',
            'No native qualification, original fixture gates, endpoint/broad/abort controls, complete-report replay or production CLI build ran for this rejected source. The uninstrumented parent still fails the original board assertion at 133.31248412 worker-CPU seconds against 120, and passes the separate 60-second neck check-body assertion. Historical standalone gates are not matched speed comparisons.',
            'Version 1 stopped at source preparation before compilation. Version 2 corrected three expected optional-comma fragments and reproduced every formatted v1 source byte. The failed preparation and successful qualification remain distinct evidence.',
        ],
        'next_step': {
            'status': 'source-review-pending',
            'task': 'Review direct read-only rational, pi and quadratic projection from PiLaurentValue, keeping compacting and other consuming operations owned. The current wrapper always materializes an owned polynomial before those read-only projections. Establish identical shape rejection, exact returned components, sign publication and original retry behavior before preparing the combined design.',
            'basis': 'The current source retains shared read views internally but materializes every root projection. Removing that reconstruction may offset costs in repeated scalar queries; this is a source-based hypothesis, not an attribution of the measured slowdowns. Prior parent instrumentation found only 0.6244 percent root retained-copy coverage on the board, so a root-only shortcut remains insufficient evidence for native performance.',
            'constraints': [
                'Keep the endpoint/P4 parent as the unchanged baseline. Include this qualified recursive-only version as a control for any combined projection candidate in a new fixed sample set; do not extend these samples or compare different experiments as matched measurements.',
                'No additional recognition prepass, cache fact, looser bound, changed arithmetic order or opaque Computable owner in the closed payload.',
                'Preserve every supported and declined shape, rational zero, pi powers, multiradical cancellation, sign retention, and the mandatory original retry after dropping the first memo.',
                'Exercise owned and retained values, publication races, live aliases, graph release, original term/radical frontiers, low/high/low P4 behavior and exact mathematical witnesses.',
                'Record every layout, requested-memory field and probe size; keep both original full-resolution native limits and all native admission controls.',
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
