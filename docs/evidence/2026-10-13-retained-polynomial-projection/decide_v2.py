"""Close the fixed three-source trial and reject the measured projection design."""
import hashlib
import json
from pathlib import Path

from qualify_v2 import verify

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'decision-v2.json'
    assert not destination.exists()
    preparation = json.loads((E / 'preparation-v2/protocol.json').read_text())
    verify(preparation)
    assessment = json.loads((E / 'cost-assessment-v2.json').read_text())
    qualification = json.loads((E / 'correctness-v2.json').read_text())
    assert qualification['all_42_commands_passed']
    assert assessment['all_mathematical_witnesses_and_representation_inventories_match']
    for path, digest in assessment['evidence_sha256'].items():
        assert sha(E / path) == digest, path
    commands = json.loads((E / 'cost-v2/commands.json').read_text())
    assert len(commands) == 129 and all(r['exit_code'] == 0 for r in commands)
    for command in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(E / 'cost-v2' / (command['name'] + '.' + stream)) == command[stream + '_sha256']
    assert len(assessment['timing_rows']) == len(assessment['memory_rows']) == 204
    summaries, slow, adverse, fast = {}, {}, {}, {}
    expected_counts = {
        'recursive_vs_baseline': [1, 28, 7, 13, 43],
        'projected_vs_baseline': [9, 29, 4, 10, 41],
        'projected_vs_recursive': [10, 12, 11, 30, 24],
    }
    for comparison in assessment['comparisons']:
        name = comparison['variant'] + '_vs_' + comparison['reference']
        timing = [r for r in assessment['timing_rows'] if r['comparison'] == name]
        memory = [r for r in assessment['memory_rows'] if r['comparison'] == name]
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
        sizes = [r['delta_bytes'] for r in assessment['binary_sizes'] if r['comparison'] == name]
        assert len(sizes) == 6
        summary = {
            'timing_rows': 68, 'memory_rows': 68,
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
                                    'ratio_of_medians_above_one_rows']] == expected_counts[name]
        summaries[name] = summary
        slow[name] = [r for r in timing if r['slower_blocks'] == 5]
        adverse[name] = [r for r in timing if r['slower_blocks'] >= 4]
        fast[name] = [r for r in timing if r['faster_blocks'] == 5]
    assert summaries['recursive_vs_baseline']['memory_fields'] == summaries['projected_vs_baseline']['memory_fields']
    assert all(r['min_delta'] == r['max_delta'] == 0
               for r in summaries['projected_vs_recursive']['memory_fields'].values())
    projected = summaries['projected_vs_baseline']
    assert projected['memory_fields']['retained_bytes_delta']['increased_rows'] == 21
    assert projected['memory_fields']['allocation_calls']['reduced_rows'] == 9
    assert all(any(r['suite'] == 'rational' and r['name'] == 'conjugate_product_' + phase
                   for r in slow['projected_vs_baseline']) for phase in ['first', 'warm', 'cloned'])
    assert all(r['suite'] == 'integer' for r in slow['projected_vs_recursive'])
    assert not any(p.name.startswith(('native-', 'report-replay')) for p in E.iterdir())
    review_paths = [
        Path(preparation['parent_source']) / 'crates/hyperreal/src/computable/node/algebra.rs',
        Path(preparation['parent_source']) / 'crates/hyperreal/src/computable/node/representation.rs',
    ]
    decision = {
        'status': 'reject-direct-retained-polynomial-projection-on-scalar-costs',
        'decision_runner_sha256': sha(Path(__file__)),
        'cost_assessment_sha256': sha(E / 'cost-assessment-v2.json'),
        'correctness_sha256': sha(E / 'correctness-v2.json'),
        'decision': 'Reject the measured projected version. All scoped correctness checks and exact cost witnesses pass. Against the unchanged baseline, nine phases are slower in all five pairs, retained/peak bytes rise in twenty-one phases, and every probe grows. Against the recursive control, ten integer phases are slower in all five pairs and every requested-memory counter is unchanged. Favorable quadratic timings do not resolve the adverse rows. Neither experimental source advances to native validation or main runtime.',
        'closed_sample_set': {
            'measurement_processes': 90, 'preflight_processes': 18, 'blocks': 5,
            'timing_inputs_per_phase': 4096, 'allocation_inputs_per_phase': 1000,
            'comparisons_share_samples': True, 'independent_experiments': False,
            'extended': False, 'all_witnesses_and_inventories_match': True,
        },
        'summaries': summaries,
        'layouts': qualification['layouts'],
        'retained_view_layouts': qualification['retained_view_layouts'],
        'memo_inventories': qualification['memo_inventories'],
        'ordinary_copy_sharing': qualification['sharing'],
        'slower_in_all_five_blocks': slow,
        'slower_in_at_least_four_blocks': adverse,
        'faster_in_all_five_blocks': fast,
        'limits': assessment['limits'] + [
            'Projected conjugate-product first/warm/cloned queries have ratios of medians 1.051236/1.057450/1.037710 and median paired ratios 1.061595/1.059850/1.058736. Each pair is adverse, with paired ranges 1.043886..1.075333, 1.038217..1.121170 and 1.030894..1.117077. First/cloned each add 16,000 allocated, retained and peak requested bytes per 1,000 inputs; warm counters are unchanged.',
            'Projected integer canonical-boundary warm, signed-word first/warm/cloned and wide-integer first/cloned queries are also slower than baseline in all five pairs. Wide-integer first/cloned median paired ratios are 1.343974/1.183993, with large paired ranges 1.108414..1.803474 and 1.052410..1.810426. The complete samples and ranges remain visible; no significance or application-effect claim follows.',
            'Against the recursive control, all four requested-memory fields match in every one of 68 phases and all five blocks. This does not establish that direct projection never avoids copying; it establishes no additional measured heap saving on these probes.',
            'Against baseline, both experimental variants save allocation calls in nine phases and increase none. Allocated bytes decrease in nine and increase in fifteen; retained bytes rise in twenty-one and decrease in none; peak bytes rise in twenty-one and decrease in one by 96 bytes. All paired memory deltas repeat five times. These counters exclude native C heaps, stack frames and allocator bookkeeping.',
            'Projected quadratic multiradical-square first/warm/cloned queries are faster than baseline in all five pairs, with median paired ratios 0.903063/0.913401/0.924875 and 2,000 fewer allocations per 1,000 inputs. First/cloned still retain 16,000 additional requested bytes. Mixed-field-decline cloned is the fourth all-five-faster row. All favorable results are retained.',
            'Against the recursive control, eleven quadratic phases are faster in all five pairs, while ten integer phases are slower in all five. These contrasts share samples with both baseline comparisons and cannot be counted as independent evidence. No cause for the integer timing differences is established by this trial.',
            'The projected probes grow by 5,072 to 5,080 bytes against baseline and by 856 to 888 bytes against recursive control. Recursive probes grow by 4,192 to 4,216 bytes against baseline. Production CLI size is unmeasured.',
            'Fresh recursive-control timing directions differ from the earlier closed recursive-only run. The earlier results remain frozen and are not appended, pooled or treated as matched measurements here. Large within-set variation remains visible, and no sample is discarded.',
            'First-query phases use fresh graphs within processes that may already share constants. Cloned-query phases exclude cloning itself. Retained counters are phase-relative and do not measure eventual process reclamation.',
            'No native qualification, original fixture gates, endpoint/broad/abort controls, complete-report replay or production CLI build ran for this rejected source. The uninstrumented endpoint/P4 parent still fails the 120-second board assertion at 133.31248412 worker-CPU seconds and passes the separate 60-second neck check-body assertion. Historical standalone gates are not matched speed comparisons.',
            'Version 1 stopped in source recovery before compilation. Version 2 accounts for two rustfmt rewrites and reproduces all three formatted v1 source maps byte-for-byte. Every executed runner and source remains frozen.',
        ],
        'next_step': {
            'status': 'cache-local-borrowing-review-pending-no-candidate',
            'task': 'Review a scoped read callback for a closed right multiplication operand while retaining the parent Box-backed cache and owned polynomial return type. Determine whether the multiplication body and every helper it invokes can be proven non-reentrant with respect to the held cache lock before preparing a candidate.',
            'basis': 'The parent copies a closed payload under its cache read guard, then releases the guard. Its multiplication normalizes left before right. A cache-local borrowed product might avoid the right-side copy without an Arc header or a result enum on every recursive call. This is a source-based hypothesis, not an attribution of timing or a demonstrated improvement.',
            'source_review_sha256': {str(p.relative_to(ROOT)): sha(p) for p in review_paths},
            'constraints': [
                'Resolve read-lock reentrancy, writer blocking and callback lifetime before implementation; do not hold a guard across unchecked expression evaluation or publication.',
                'Preserve left-before-right normalization, cache-hit work accounting, reuse tracking, operation/insertion order, every opaque-owner decision and the original retry after releasing the first memo.',
                'Keep original Box payloads, scratch layouts, memo behavior, all exact/declined shapes and limits, with no new cache fact, approximation, dependency or recognition prepass.',
                'Include original cache/P4/frontier tests and explicit alias, lifetime, concurrent-reader/writer and fallback controls. Require fresh qualification and fixed costs; no old sample extension.',
                'Root retained-copy coverage of 0.6244 percent is not a cost estimate. Keep both full-resolution native gates and every remaining admission obligation.',
            ],
            'candidate_prepared': False,
        },
        'source_parent_main_unchanged': True,
        'runtime_admitted': False, 'native_runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'status': decision['status'], 'summaries': summaries, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
