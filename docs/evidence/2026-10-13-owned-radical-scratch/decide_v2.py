"""Record rejection of the completed owned-scratch trial without new samples."""
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
                                'ratio_of_medians_above_one_rows']] == [6, 24, 3, 17, 31]
    slow = [r for r in timing if r['slower_blocks'] == 5]
    assert [(r['suite'], r['name']) for r in slow] == [
        ('rational', 'quadratic_reciprocal_first'),
        ('quadratic', 'conjugate_then_affine_first'),
        ('quadratic', 'conjugate_then_affine_warm'),
        ('integer', 'signed_word_boundary_construction'),
        ('integer', 'signed_word_boundary_first'),
        ('integer', 'wide_integer_affine_construction'),
    ]
    assert fields['allocation_calls']['increased_rows'] == 11
    assert fields['retained_bytes_delta']['increased_rows'] == 14
    assert not any(p.name.startswith(('native-', 'report-replay')) for p in E.iterdir())
    decision = {
        'status': 'reject-owned-radical-scratch-on-scalar-performance-and-memory-costs',
        'decision_runner_sha256': sha(Path(__file__)),
        'cost_assessment_sha256': sha(E / 'cost-assessment-v2.json'),
        'correctness_sha256': sha(E / 'correctness-v2.json'),
        'decision': 'Reject the measured owned-scratch version. Its tested mathematical values, P4 facts, alias isolation and graph ownership are preserved, but repeated scalar slowdowns and increased allocation/retention costs remain. The smaller closed coefficient and three all-five-faster rows do not establish an acceptable tradeoff under the governing ladder. Do not advance this source to native validation or copy it into main.',
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
        'limits': assessment['limits'] + [
            'Quadratic-reciprocal first queries have ratio of medians 1.333721 and median paired ratio 1.157056, with paired ratios from 1.039856 to 1.515189. They add 1,000 requested allocations and 24,000 allocated/retained/peak bytes per 1,000 inputs. Ratio of medians and median paired ratio are distinct summaries.',
            'Conjugate-then-affine first and warm queries are slower in all five pairs: ratios of medians 1.045733/1.033518 and median paired ratios 1.052773/1.047584. The warm phase saves 285 allocation calls but still has adverse timing. Its cloned phase is slower in four pairs and adds 715 calls and 24,000 retained/peak bytes.',
            'Signed-word construction/first and wide-integer construction have smaller all-five adverse directions; these are retained as descriptive observations, not significance claims or the sole basis for rejection.',
            'Allocation calls decrease in four phases and increase in eleven. Allocated bytes decrease in thirteen and increase in eleven. Retained bytes decrease in seven and increase in fourteen; peak bytes decrease in eight and increase in fourteen. Every paired memory delta repeats five times. These phase-relative requested-heap counters are not eventual reclamation or total-memory measurements.',
            'All six probe executables grow by 768 to 800 bytes. Production CLI size was not measured.',
            'Every timing sample and range remains in the assessment, including substantial baseline and candidate variation. Rational pi-parameter transport cloned queries and quadratic multiradical-square/mixed-field-decline construction are faster in all five pairs; this does not establish an application speedup or resolve the adverse rows.',
            'First-query phases use fresh graphs in processes that may already share constants; cloned-query phases do not include cloning. Timer-scale radical-Laurent query rows are not treated as persuasive performance gains.',
            'No native qualification, endpoint/broad/abort controls, original fixture gates, complete-report replay or production CLI build ran for this rejected source. The uninstrumented parent still fails the 120-second board gate at 133.31248412 worker-CPU seconds and passes the separate 60-second neck check-body assertion. These are standalone historical gates, not matched speed comparisons.',
            'The failed v1 qualification and corrected v2 qualification remain distinct. No v1 costs ran and no v1 pass was relabelled as a v2 execution.',
        ],
        'next_step': {
            'status': 'source-review-pending',
            'task': 'Review how retained polynomial reads reconstruct the outer term Vec and radical arrays, and whether whole-polynomial immutable views can avoid those copies until arithmetic actually needs owned storage. Establish a concrete ownership and operation-order design before preparing any further candidate.',
            'basis': 'The rejected radical-only designs leave outer polynomial reconstruction intact. Prior instrumentation counted frequent recursive retained copies, but those counts are neither allocation measurements nor an attribution of the present slowdowns. A whole-polynomial view is an unmeasured hypothesis, not an accepted optimization.',
            'constraints': [
                'Start from the unchanged endpoint/P4 parent; do not combine rejected memo omission or storage candidates into a new baseline.',
                'Keep closed payloads graph-free, opaque Computable owner counts and local memo behavior intact, and prevent any mutable alias to retained coefficients.',
                'Preserve arithmetic/insertion order, eight-term/eight-radical bounds, original work/depth limits and the mandatory original retry after releasing its first memo.',
                'Review all consuming and mutable operations, projection lifetimes, lock scope, empty states, publication and buffer release before implementation.',
                'Measure layouts, all scalar phases, requested memory and sizes in a new fixed sample set; preserve both original native gates and full fixtures.',
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
