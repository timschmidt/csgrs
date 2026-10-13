"""Carry mixed scalar evidence to native investigation without admitting runtime."""
import hashlib
import json
from pathlib import Path

from qualify_v1 import verify

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'scalar-decision-v1.json'
    assert not destination.exists()
    verify(json.loads((E / 'preparation-v1/protocol.json').read_text()))
    assessment = json.loads((E / 'cost-assessment-v1.json').read_text())
    qualification = json.loads((E / 'correctness-v1.json').read_text())
    assert qualification['all_28_commands_passed']
    assert assessment['all_mathematical_witnesses_and_representation_inventories_match']
    for path, digest in assessment['evidence_sha256'].items():
        assert sha(E / path) == digest
    commands = json.loads((E / 'cost-v1/commands.json').read_text())
    assert len(commands) == 86 and all(r['exit_code'] == 0 for r in commands)
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(E / 'cost-v1' / (row['name'] + '.' + stream)) == row[stream + '_sha256']
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
    assert [summary[k] for k in ['all_five_slower_rows', 'at_least_four_slower_rows', 'all_five_faster_rows', 'at_least_four_faster_rows', 'ratio_of_medians_above_one_rows']] == [2, 10, 5, 18, 28]
    assert all(r['increased_rows'] == 0 for r in fields.values())
    assert fields['allocation_calls']['reduced_rows'] == fields['allocated_bytes']['reduced_rows'] == 9
    assert fields['retained_bytes_delta']['reduced_rows'] == 0 and fields['peak_bytes_delta']['reduced_rows'] == 2
    decision = {
        'decision': 'advance-cache-local-products-to-native-investigation-with-scalar-costs-open',
        'decision_runner_sha256': sha(Path(__file__)),
        'cost_assessment_sha256': sha(E / 'cost-assessment-v1.json'),
        'cost_result_sha256': sha(E / 'cost-v1/result.json'),
        'correctness_sha256': sha(E / 'correctness-v1.json'),
        'qualification': qualification['qualification'],
        'closed_sample_set': {'measurement_processes': 60, 'preflight_processes': 12, 'blocks': 5,
            'timing_inputs_per_phase': 4096, 'allocation_inputs_per_phase': 1000,
            'extended': False, 'all_witnesses_and_inventories_match': True},
        'summary': summary,
        'slower_in_all_five_blocks': [r for r in timing if r['slower_blocks'] == 5],
        'slower_in_at_least_four_blocks': [r for r in timing if r['slower_blocks'] >= 4],
        'faster_in_all_five_blocks': [r for r in timing if r['faster_blocks'] == 5],
        'rationale': 'The typed operation avoids a closed right-operand copy without changing retained storage or measured exact behavior. Nine phases save requested calls/bytes, two reduce peak bytes, and no measured memory field increases. Timing remains mixed: five phases are faster in every pair and eighteen in at least four, while two are slower in every pair and ten in at least four. This justifies investigating the native consumers and original fixture gates, not retaining the source or treating allocation savings as resolution of timing regressions.',
        'unresolved_costs': [
            'Quadratic conjugate-then-affine cloned queries are slower in all five pairs: ratio of medians 1.058581, median paired ratio 1.054748, paired range 1.019898..1.420584. Warm queries are slower in four pairs: ratio of medians 1.089134, median paired ratio 1.086002, range 0.996013..1.170242. Both phases save 570 requested allocations and 22,800 allocated bytes per 1,000 inputs, with unchanged retained/peak bytes. Lower-rung savings do not resolve those timing costs.',
            'Rational radical-Laurent warm is the second all-five-slower phase: ratio of medians 1.014889 and median paired ratio 1.014827. The phase medians are 16,119 and 16,359 ns across 4,096 inputs; this timer-scale difference remains recorded and is not treated as persuasive evidence of a practical effect.',
            'Quadratic-reciprocal first/warm/cloned, quadratic pi-transport first/warm and opaque-decline first/cloned queries are also adverse in four pairs. Every row and range remains visible.',
            'Large timing variation affects both favorable and adverse rows. All first-block rational and many quadratic ratios reflect slower baseline samples. No samples are discarded, no earlier samples are appended and no significance or application-speedup claim is made.',
            'All six probes grow, by 2,848 to 2,880 bytes. Production CLI size remains unmeasured. The temporary product-lookup Result is 32 bytes; private type sizes do not measure stack frames or total memory.',
            'A product holds a cache read lock across rational arithmetic. Scoped source inspection and passing concurrent-reader/writer tests do not prove all possible schedules, bounded waiting or a universal lock-duration bound.',
        ],
        'favorable_costs': [
            'Requested allocation calls and bytes decrease in nine phases, by up to 2,000 calls and 96,000 bytes per 1,000 inputs. Retained bytes match in every phase and pair. Peak bytes decrease in two phases by up to 96 bytes. Every paired memory delta repeats five times, and none increases.',
            'Quadratic multiradical-square cloned queries are faster in every pair, median paired ratio 0.912608, with 2,000 fewer allocations and 96,000 fewer allocated bytes per 1,000 inputs. The other all-five-faster phases are rational multiradical-square, radical-Laurent and conjugate-product construction, and quadratic mixed-field-decline construction; their complete ranges remain attached.',
        ],
        'retention_requirement': 'Keep every scalar adverse row and inherited endpoint tradeoff open. Native timing gains alone do not dismiss them. Resolve meaningful regressions and complete original gates, matched native costs, complete report identity, production size and remaining workflow obligations before runtime retention.',
        'limits': assessment['limits'] + [
            'The baseline already includes the isolated endpoint changes and admitted P4 repair. These samples cannot establish a speedup over main or erase inherited endpoint tradeoffs.',
            'The uninstrumented parent fails the original board assertion at 133.31248412 worker-CPU seconds against 120 and passes the separate 60-second neck check-body assertion. These historical standalone results are not candidate measurements or matched speed comparisons.',
            'First-query phases use fresh graphs in a process that may share constants; cloned-query phases exclude cloning. Requested retained bytes are phase-relative and do not measure eventual reclamation.',
        ],
        'next_step': 'Run the committed native validation runner in a fresh target: strict four-crate all-feature/all-target Clippy, full solver/curve/DRC suites, warning-denied docs, then both original assertions separately from a bound executable. Record gate failures without enlarging limits. Fixed native costs, reports and production CLI size follow only if investigation continues.',
        'native_runners_sha256': {name: sha(E / name) for name in ['validate_native_v1.py', 'assess_native_v1.py']},
        'native_fixed_block_order': [['baseline', 'candidate'], ['candidate', 'baseline'], ['baseline', 'candidate'], ['candidate', 'baseline'], ['baseline', 'candidate']],
        'native_cost_protocol': {'endpoint_time_inputs': 8192, 'endpoint_allocation_inputs': 1000,
            'endpoint_measurement_processes': 20, 'broad_measurement_processes': 10,
            'broad_witness_rows_per_process': 231, 'final_abort_processes': 6,
            'final_abort_rows_per_process': 210, 'final_abort_blocks': 'first three predeclared blocks',
            'sample_extension_planned': False},
        'runtime_admitted': False, 'native_runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'decision': decision['decision'], 'summary': summary, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
