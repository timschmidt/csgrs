"""Record rejection of the outlined product version with all fixed-trial contrasts."""
import hashlib
import json
from pathlib import Path

from qualify_v1 import verify

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
load = lambda p: json.loads(p.read_text())


def main():
    destination = E / 'decision-v1.json'
    assert not destination.exists()
    preparation = load(E / 'preparation-v1/protocol.json')
    verify(preparation)
    correctness = load(E / 'correctness-v1.json')
    assessment = load(E / 'cost-assessment-v2.json')
    codegen = load(E / 'codegen-v2/protocol.json')
    protocol = load(E / 'cost-v2/protocol.json')
    result = load(E / 'cost-v2/result.json')
    commands = load(E / 'cost-v2/commands.json')
    assert correctness['all_42_commands_passed'] and result['complete']
    assert len(commands) == 129 and result['measured_processes'] == 90
    assert protocol['preflight_processes'] == 18 and len(protocol['fixed_block_order']) == 5
    assert protocol['time_inputs'] == 4096 and protocol['memory_inputs'] == 1000
    assert sha(E / 'cost-setup-failure-v1.json') == protocol['setup_failure_sha256']
    assert load(E / 'cost-setup-failure-v1.json')['commands_executed'] == 0
    for name, digest in assessment['evidence_sha256'].items():
        assert sha(E / name) == digest, name
    for phase in commands:
        assert phase['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(E / 'cost-v2' / (phase['name'] + '.' + stream)) == phase[stream + '_sha256']
    assert sha(E / 'cost-assessment-v2.json') == codegen['cost_assessment_sha256']
    for name, digest in codegen['artifacts_sha256'].items():
        assert sha(E / 'codegen-v2' / name) == digest
    summaries = {}
    for contrast in assessment['comparisons']:
        name = contrast['variant'] + '_vs_' + contrast['reference']
        timing = [r for r in assessment['timing_rows'] if r['comparison'] == name]
        memory = [r for r in assessment['memory_rows'] if r['comparison'] == name]
        sizes = [r['delta_bytes'] for r in assessment['binary_sizes'] if r['comparison'] == name]
        assert len(timing) == len(memory) == 68 and len(sizes) == 6
        fields = {}
        for field in memory[0]['fields']:
            values = [r['fields'][field] for r in memory]
            deltas = [v['median_block_delta'] for v in values]
            fields[field] = {'reduced_rows': sum(v < 0 for v in deltas),
                'increased_rows': sum(v > 0 for v in deltas), 'min_delta': min(deltas),
                'max_delta': max(deltas), 'all_five_deltas_equal': all(len(set(v['block_deltas'])) == 1 for v in values)}
            assert fields[field]['all_five_deltas_equal'] and not fields[field]['increased_rows']
        summaries[name] = {'timing_rows': len(timing), 'memory_rows': len(memory),
            'all_five_slower_rows': sum(r['slower_blocks'] == 5 for r in timing),
            'at_least_four_slower_rows': sum(r['slower_blocks'] >= 4 for r in timing),
            'all_five_faster_rows': sum(r['faster_blocks'] == 5 for r in timing),
            'at_least_four_faster_rows': sum(r['faster_blocks'] >= 4 for r in timing),
            'ratio_of_medians_above_one_rows': sum(r['ratio_of_medians'] > 1 for r in timing),
            'memory_fields': fields, 'probe_binary_delta_bytes_range': [min(sizes), max(sizes)]}
    expected = {'direct_vs_baseline': (0, 5, 4, 23, 15), 'outlined_vs_baseline': (1, 4, 6, 21, 17), 'outlined_vs_direct': (1, 9, 3, 18, 35)}
    for name, values in expected.items():
        assert tuple(summaries[name][k] for k in ['all_five_slower_rows', 'at_least_four_slower_rows', 'all_five_faster_rows', 'at_least_four_faster_rows', 'ratio_of_medians_above_one_rows']) == values
    decisive = next(r for r in assessment['timing_rows'] if r['comparison'] == 'outlined_vs_baseline' and r['suite'] == 'quadratic' and r['name'] == 'conjugate_then_affine_cloned')
    assert decisive['slower_blocks'] == 5 and 1.067 < decisive['median_block_ratio'] < 1.068
    same_memory = [r for r in assessment['memory_rows'] if r['comparison'] == 'outlined_vs_direct']
    assert all(all(v['median_block_delta'] == 0 for v in r['fields'].values()) for r in same_memory)
    static = {}
    for row in codegen['rows']:
        assert sha(Path(row['binary']['path'])) == row['binary']['sha256']
        static[row['variant']] = {name: {key: value for key, value in symbol.items() if key in ['code_bytes', 'first_stack_pointer_subtraction_bytes', 'helper_named_transfer_lines']} for name, symbol in row['symbols'].items()}
    assert [static[v]['coefficient']['code_bytes'] for v in ['baseline', 'direct', 'outlined']] == [2473, 4161, 2800]
    assert [static[v]['coefficient']['first_stack_pointer_subtraction_bytes'] for v in ['baseline', 'direct', 'outlined']] == [248, 264, 264]
    assert static['outlined']['helper']['code_bytes'] == 1442
    assert static['outlined']['helper']['first_stack_pointer_subtraction_bytes'] == 104
    assert len(static['outlined']['coefficient']['helper_named_transfer_lines']) == 1
    assert '\tcall ' in static['outlined']['coefficient']['helper_named_transfer_lines'][0]
    old = ROOT / 'docs/evidence/2026-10-13-direct-same-radicand-products/decision-v1.json'
    assert load(old)['summary']['all_five_slower_rows'] == 8
    next_step = {
        'action': 'Keep all three sources and this closed trial frozen; review a helper that borrows already-checked coefficient fields rather than consuming and returning a whole RadicalCoefficient.',
        'motivation': 'The emitted call reduces the multiplier symbol but leaves its first stack subtraction at 264 bytes. The source helper moves a 56-byte coefficient in and out and repeats bounds checks justified by the caller. A narrower borrowed-field boundary may avoid those requirements; this is a hypothesis, not a timing attribution.',
        'required_review': [
            'Prove that disjoint mutable rational/radical fields and borrowed right fields preserve arithmetic and zero-skipping order without added scalar comparisons or owners.',
            'Keep the original opaque guard, singleton eligibility and exact general fallback; no unsafe access, public numeric panic, cache change, budget change or omitted retry.',
            'Preserve first-surviving radicand payload, cancellation cleanup and all stronger-fact controls; examine drop order before preparing source.',
            'Only after a sound review, prepare a distinct candidate with unchanged shared tests and a new fixed baseline/outlined/new-candidate comparison. Freeze its protocol before compilation and do not extend existing samples.',
        ],
        'new_candidate_prepared': False,
        'scope': 'The full plan remains active: all 39 properties, 14 targets and crate future work remain in scope; individual ledger statuses do not change.'}
    decision = {
        'status': 'reject-outlined-same-radicand-products-on-unresolved-scalar-costs',
        'decision_runner_sha256': sha(Path(__file__)),
        'inputs_sha256': {name: sha(E / name) for name in ['preparation-v1/protocol.json', 'correctness-v1.json', 'cost-assessment-v2.json', 'cost-v2/result.json', 'cost-setup-failure-v1.json', 'codegen-v2/protocol.json']},
        'decision': 'Reject this outlined version for runtime admission. It preserves all tested exact witnesses and the six allocation-saving phases, but conjugate-then-affine cloned queries are slower than baseline in all five blocks, median paired ratio 1.067268. Three more phases are slower in four blocks. The helper also increases probe size and adds no measured memory saving over the direct control. A smaller multiplier symbol does not establish the required performance improvement.',
        'qualification': correctness['qualification'],
        'closed_sample_set': {'measurement_processes': 90, 'preflight_processes': 18, 'blocks': 5,
            'timing_inputs_per_phase': 4096, 'allocation_inputs_per_phase': 1000, 'extended': False,
            'all_witnesses_and_inventories_match': True, 'shared_contrasts_are_not_independent': True},
        'summaries': summaries, 'decisive_adverse_row': decisive,
        'at_least_four_slower_rows': [r for r in assessment['timing_rows'] if r['slower_blocks'] >= 4],
        'all_five_faster_rows': [r for r in assessment['timing_rows'] if r['faster_blocks'] == 5],
        'nonzero_memory_rows': [r for r in assessment['memory_rows'] if any(v['median_block_delta'] for v in r['fields'].values())],
        'static_code_observations': static,
        'control_context': {'previous_direct_decision_sha256': sha(old), 'previous_direct_all_five_slower_rows': 8,
            'fresh_direct_all_five_slower_rows': 0, 'previous_rejection_unchanged': True,
            'interpretation': 'The fresh direct control has materially different directions from its previous closed trial. Both records remain; this shared comparison neither erases that rejection nor establishes a stable universal speed change. No historical timing sample is pooled into this run.'},
        'limits': assessment['limits'] + [
            'Substantial within-trial variation appears in favorable and adverse rows. All samples and both paired medians and ratios of medians remain; no significance or application-speedup claim is made.',
            'The outlined multiplier has 2800 code bytes plus a 1442-byte helper, compared with the direct 4161-byte multiplier. Its first stack subtraction remains 264 bytes, with 104 in the helper. These are static observations, not total/peak stack use or a causal explanation.',
            'Named transfers inside the helper include internal jumps to offsets within that function; only the coefficient multiplier record is used to identify its single emitted helper call.',
            'The first cost attempt failed before any build or probe. Correcting its output directory did not replace any timing sample or source.'
        ],
        'native': {'status': 'not-executed-after-scalar-cost-rejection', 'original_fixture_gates_run': False,
            'matched_native_costs_run': False, 'complete_report_identity_established': False,
            'production_cli_size_measured': False, 'original_limits': preparation['original_limits']},
        'next_step': next_step, 'main_and_35_pending_native_paths_unchanged': True,
        'runtime_admitted': False, 'plan_complete': False}
    verify(preparation)
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'status': decision['status'], 'summaries': summaries, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
