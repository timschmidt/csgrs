"""Reject the closed scalar trial while preserving its favorable and adverse data."""
import hashlib
import json
from pathlib import Path

from qualify_v2 import verify

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'decision-v1.json'
    assert not destination.exists()
    verify(json.loads((E / 'preparation-v2/protocol.json').read_text()))
    qualification = json.loads((E / 'correctness-v2.json').read_text())
    cost = json.loads((E / 'cost-assessment-v2.json').read_text())
    codegen = json.loads((E / 'codegen-v1/protocol.json').read_text())
    assert qualification['all_28_commands_passed']
    assert cost['all_mathematical_witnesses_and_representation_inventories_match']
    for name, digest in cost['evidence_sha256'].items():
        assert sha(E / name) == digest, name
    assert sha(E / 'inspect_codegen_v1.py') == codegen['runner_sha256']
    assert sha(E / 'cost-assessment-v2.json') == codegen['cost_assessment_sha256']
    assert sha(E / 'cost-v2/binaries.json') == codegen['binaries_sha256']
    for name, digest in codegen['artifacts_sha256'].items():
        assert sha(E / 'codegen-v1' / name) == digest, name
    for row in codegen['rows']:
        assert sha(Path(row['binary']['path'])) == row['binary']['sha256']
    timing, memory = cost['timing_rows'], cost['memory_rows']
    assert len(timing) == len(memory) == 68
    fields = {}
    for field in memory[0]['fields']:
        values = [row['fields'][field] for row in memory]
        assert all(len(set(row['block_deltas'])) == 1 for row in values)
        deltas = [row['median_block_delta'] for row in values]
        fields[field] = {
            'reduced_rows': sum(value < 0 for value in deltas),
            'increased_rows': sum(value > 0 for value in deltas),
            'min_delta': min(deltas), 'max_delta': max(deltas),
            'all_five_deltas_equal': True,
        }
    summary = {
        'timing_rows': 68, 'memory_rows': 68,
        'all_five_slower_rows': sum(row['slower_blocks'] == 5 for row in timing),
        'at_least_four_slower_rows': sum(row['slower_blocks'] >= 4 for row in timing),
        'all_five_faster_rows': sum(row['faster_blocks'] == 5 for row in timing),
        'at_least_four_faster_rows': sum(row['faster_blocks'] >= 4 for row in timing),
        'ratio_of_medians_above_one_rows': sum(row['ratio_of_medians'] > 1 for row in timing),
        'memory_fields': fields,
        'probe_binary_delta_bytes_range': [min(row['delta_bytes'] for row in cost['binary_sizes']),
                                         max(row['delta_bytes'] for row in cost['binary_sizes'])],
    }
    assert [summary[key] for key in ['all_five_slower_rows', 'at_least_four_slower_rows',
            'all_five_faster_rows', 'at_least_four_faster_rows', 'ratio_of_medians_above_one_rows']] == [8, 14, 10, 32, 23]
    assert all(row['increased_rows'] == 0 for row in fields.values())
    assert fields['allocation_calls']['reduced_rows'] == fields['allocated_bytes']['reduced_rows'] == 6
    assert fields['retained_bytes_delta']['reduced_rows'] == 0
    assert fields['peak_bytes_delta']['reduced_rows'] == 4
    slower = [row for row in timing if row['slower_blocks'] == 5]
    assert all(row['suite'] == 'quadratic' for row in slower)
    assert not any((E / name).exists() for name in ['native-validation-v1', 'native-validation-v2',
                                                  'native-controls-v1', 'report-replay-v1'])
    decision = {
        'status': 'reject-direct-same-radicand-products-on-scalar-timing-costs',
        'decision_runner_sha256': sha(Path(__file__)),
        'inputs_sha256': {name: sha(E / name) for name in [
            'preparation-v2/protocol.json', 'correctness-v2.json',
            'cost-assessment-v2.json', 'cost-v2/result.json', 'codegen-v1/protocol.json',
        ]},
        'decision': 'Reject this direct-product version for runtime admission. It preserves all tested exact witnesses and saves requested allocations in six phases, but eight quadratic phases are slower in every fixed pair and fourteen phases in at least four. Multiradical, pi-transport and mixed-field costs remain unresolved under the performance-before-memory ladder. The favorable rows and unchanged measured retention do not erase those timing costs.',
        'qualification': qualification['qualification'],
        'closed_sample_set': {
            'measurement_processes': 60, 'preflight_processes': 12, 'blocks': 5,
            'timing_inputs_per_phase': 4096, 'allocation_inputs_per_phase': 1000,
            'extended': False, 'all_witnesses_and_inventories_match': True,
        },
        'summary': summary,
        'slower_in_all_five_blocks': slower,
        'slower_in_at_least_four_blocks': [row for row in timing if row['slower_blocks'] >= 4],
        'faster_in_all_five_blocks': [row for row in timing if row['faster_blocks'] == 5],
        'memory_saving_rows': [row for row in memory if any(field['median_block_delta'] < 0 for field in row['fields'].values())],
        'adverse_costs': [
            'Quadratic multiradical-square first and cloned queries are slower in all five pairs: median paired ratios 1.035270 and 1.075481, with ranges 1.012632..1.062424 and 1.047692..1.241560. Their requested memory counters match.',
            'Quadratic pi-transport first/warm/cloned queries are slower in all five pairs: median paired ratios 1.039035, 1.035174 and 1.054946. Mixed-field-decline first/warm queries are also slower in all five, at 1.062335 and 1.060469. Their requested memory counters match.',
            'Quadratic opaque-decline construction is slower in all five pairs, median paired ratio 1.041496. Opaque warm and cloned queries are slower in four pairs, at median paired ratios 1.231191 and 1.521822; variation is substantial and every sample remains.',
            'Conjugate-then-affine cloned queries save allocations but are slower in three pairs. Their median paired ratio is 1.008369 while the ratio of medians is 0.981806. Both summaries and all paired observations remain visible.',
        ],
        'favorable_costs': [
            'Rational conjugate-product first/warm/cloned phases save 1,000 requested allocations and 64,000 allocated bytes per 1,000 inputs. Quadratic conjugate-then-affine first/warm/cloned save 857 calls and 54,848 bytes. Every paired memory delta repeats five times. Retained bytes match in all phases; four phases reduce peak requested bytes by 64.',
            'Quadratic conjugate-then-affine warm is faster in every pair, median paired ratio 0.944369. First queries are faster in three pairs; cloned queries in two. Rational conjugate-product first/warm/cloned are faster in four pairs each.',
            'Other all-five-faster phases include rational multiradical-square cloned, quadratic-reciprocal warm/cloned, pi-parameter-transport first/warm, quadratic multiradical-square construction and integer wide-affine first. Complete favorable ranges remain attached.',
            'Rational radical-Laurent warm/cloned are also faster in all five pairs, but their total phase medians are only about 15,000 to 16,500 ns across 4,096 inputs. These timer-scale rows are preserved without claiming a practical effect.',
        ],
        'code_generation_observation': {
            'evidence': 'codegen-v1/protocol.json',
            'scope': 'Existing baseline and candidate quadratic timing binaries only; no additional cost sample.',
            'coefficient_symbol_bytes': [2473, 4161],
            'polynomial_symbol_bytes': [2359, 2359],
            'first_stack_pointer_subtraction_bytes': [248, 264],
            'interpretation': 'Both binaries contain separate coefficient and polynomial multiplier symbols. The coefficient body grows by 1,688 bytes and its first stack-pointer subtraction by 16 bytes. This motivates testing a helper boundary; it does not establish lost inlining, total stack use or the cause of a timing change.',
        },
        'native': {
            'status': 'not-executed-after-scalar-cost-rejection',
            'original_fixture_gates_run': False, 'matched_native_costs_run': False,
            'complete_report_identity_established': False, 'production_cli_size_measured': False,
        },
        'next_step': {
            'status': 'helper-boundary-source-review-pending',
            'task': 'Keep all sources and samples frozen. Review extracting the qualified direct arithmetic into a private non-inlined helper while leaving the original opaque guard, eligibility conditions and general fallback in the entry method. Establish ownership and bounds for the helper call before preparing any new source.',
            'comparison': 'Use a fresh three-source experiment if that review succeeds: unchanged parent, this direct version as a control, and the extracted-helper candidate. Shared correctness tests and new fixed costs distinguish the source change from variation across separate historical runs.',
            'fixed_scalar_costs_if_qualified': {
                'variants': ['baseline', 'direct', 'outlined'],
                'block_order': [['baseline', 'direct', 'outlined'], ['outlined', 'direct', 'baseline'],
                                ['direct', 'baseline', 'outlined'], ['outlined', 'baseline', 'direct'],
                                ['baseline', 'outlined', 'direct']],
                'suites': ['rational', 'quadratic', 'integer'], 'modes': ['time', 'memory'],
                'measurement_processes': 90, 'preflight_processes': 18,
                'timing_inputs': 4096, 'allocation_inputs': 1000,
                'all_builds_before_probes': True, 'sample_extension_planned': False,
                'shared_comparisons_are_not_independent': True,
            },
            'candidate_prepared': False,
        },
        'limits': cost['limits'] + codegen['limits'] + [
            'All six probe binaries grow by 1,968 to 1,984 bytes. These are probe sizes, not production CLI sizes.',
            'Timing variation affects favorable and adverse rows. No sample is removed, appended or selectively repeated, and no statistical significance or native speedup is claimed.',
            'The version 1 test-import failure and all of its sources/logs remain preserved. Version 2 changed only that import before qualification; candidate arithmetic remained unchanged.',
            'Main runtime and all 35 original native paths remain unchanged. The full plan is incomplete; all 39 properties, 14 targets and crate future work remain in scope, with individual ledger statuses unchanged.',
        ],
        'source_parent_main_and_pending_native_unchanged': True,
        'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'status': decision['status'], 'summary': summary,
                      'native_executed': False, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
