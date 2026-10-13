"""Reject borrowed coefficient fields and close the current helper-variant series."""
import gzip
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
    qualification = load(E / 'correctness-v1.json')
    assessment = load(E / 'cost-assessment-v1.json')
    protocol = load(E / 'cost-v1/protocol.json')
    result = load(E / 'cost-v1/result.json')
    commands = load(E / 'cost-v1/commands.json')
    codegen = load(E / 'codegen-v1/protocol.json')
    assert qualification['all_42_commands_passed'] and result['complete']
    assert len(commands) == 129 and result['measured_processes'] == 90
    assert protocol['preflight_processes'] == 18 and len(protocol['fixed_block_order']) == 5
    assert protocol['time_inputs'] == 4096 and protocol['memory_inputs'] == 1000
    for name, digest in assessment['evidence_sha256'].items():
        assert sha(E / name) == digest
    for phase in commands:
        assert phase['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(E / 'cost-v1' / (phase['name'] + '.' + stream)) == phase[stream + '_sha256']
    assert sha(E / 'cost-assessment-v1.json') == codegen['cost_assessment_sha256']
    for name, digest in codegen['artifacts_sha256'].items():
        assert sha(E / 'codegen-v1' / name) == digest
    summaries = {}
    for comparison in assessment['comparisons']:
        name = comparison['variant'] + '_vs_' + comparison['reference']
        timing = [r for r in assessment['timing_rows'] if r['comparison'] == name]
        memory = [r for r in assessment['memory_rows'] if r['comparison'] == name]
        sizes = [r['delta_bytes'] for r in assessment['binary_sizes'] if r['comparison'] == name]
        assert len(timing) == len(memory) == 68 and len(sizes) == 6
        fields = {}
        for field in memory[0]['fields']:
            values = [r['fields'][field] for r in memory]
            deltas = [v['median_block_delta'] for v in values]
            fields[field] = {'reduced_rows': sum(v < 0 for v in deltas), 'increased_rows': sum(v > 0 for v in deltas),
                'min_delta': min(deltas), 'max_delta': max(deltas),
                'all_five_deltas_equal': all(len(set(v['block_deltas'])) == 1 for v in values)}
            assert fields[field]['all_five_deltas_equal'] and not fields[field]['increased_rows']
        summaries[name] = {'timing_rows': 68, 'memory_rows': 68,
            'all_five_slower_rows': sum(r['slower_blocks'] == 5 for r in timing),
            'at_least_four_slower_rows': sum(r['slower_blocks'] >= 4 for r in timing),
            'all_five_faster_rows': sum(r['faster_blocks'] == 5 for r in timing),
            'at_least_four_faster_rows': sum(r['faster_blocks'] >= 4 for r in timing),
            'ratio_of_medians_above_one_rows': sum(r['ratio_of_medians'] > 1 for r in timing),
            'memory_fields': fields, 'probe_binary_delta_bytes_range': [min(sizes), max(sizes)]}
    expected = {'outlined_vs_baseline': (4, 18, 4, 21, 30), 'borrowed_vs_baseline': (6, 15, 3, 13, 33), 'borrowed_vs_outlined': (4, 14, 0, 13, 36)}
    keys = ['all_five_slower_rows', 'at_least_four_slower_rows', 'all_five_faster_rows', 'at_least_four_faster_rows', 'ratio_of_medians_above_one_rows']
    for name, values in expected.items():
        assert tuple(summaries[name][k] for k in keys) == values
    assert all(f['min_delta'] == f['max_delta'] == 0 for f in summaries['borrowed_vs_outlined']['memory_fields'].values())
    static = {}
    for row in codegen['rows']:
        assert sha(Path(row['binary']['path'])) == row['binary']['sha256']
        static[row['variant']] = {}
        for name, symbol in row['symbols'].items():
            raw = gzip.decompress((E / 'codegen-v1' / (row['variant'] + '-' + name + '.asm.gz')).read_bytes())
            assert hashlib.sha256(raw).hexdigest() == symbol['disassembly_stdout_sha256']
            static[row['variant']][name] = {key: symbol[key] for key in ['code_bytes', 'first_stack_pointer_subtraction_bytes']}
        static[row['variant']]['entry_helper_transfers'] = row['symbols']['coefficient']['helper_named_transfer_lines']
    assert [static[v]['coefficient']['code_bytes'] for v in ['baseline', 'outlined', 'borrowed']] == [2473, 2800, 2993]
    assert [static[v]['coefficient']['first_stack_pointer_subtraction_bytes'] for v in ['baseline', 'outlined', 'borrowed']] == [248, 264, 264]
    assert static['outlined']['helper']['code_bytes'] == 1442 and static['borrowed']['field_helper']['code_bytes'] == 1052
    assert static['outlined']['helper']['first_stack_pointer_subtraction_bytes'] == static['borrowed']['field_helper']['first_stack_pointer_subtraction_bytes'] == 104
    assert len(static['borrowed']['entry_helper_transfers']) == 1 and '\tcall ' in static['borrowed']['entry_helper_transfers'][0]
    prior = ROOT / 'docs/evidence/2026-10-13-outlined-same-radicand-products/decision-v1.json'
    old = load(prior)
    assert old['summaries']['outlined_vs_baseline']['all_five_slower_rows'] == 1
    decision = {
        'status': 'reject-borrowed-same-radicand-fields-on-scalar-timing-costs',
        'decision_runner_sha256': sha(Path(__file__)),
        'inputs_sha256': {name: sha(E / name) for name in ['preparation-v1/protocol.json', 'correctness-v1.json', 'cost-assessment-v1.json', 'cost-v1/result.json', 'codegen-v1/protocol.json']},
        'decision': 'Reject this borrowed-field version for runtime admission. It passes all scoped exactness/cache controls and retains the six allocation-saving phases, but six phases are slower than baseline in every fixed pair and fifteen in at least four. Four phases are slower than the outlined control in all five pairs. The smaller helper/probes and unchanged measured retention do not resolve those performance costs.',
        'qualification': qualification['qualification'],
        'closed_sample_set': {'measurement_processes': 90, 'preflight_processes': 18, 'blocks': 5,
            'timing_inputs_per_phase': 4096, 'allocation_inputs_per_phase': 1000, 'extended': False,
            'all_witnesses_and_inventories_match': True, 'shared_contrasts_are_not_independent': True},
        'summaries': summaries,
        'at_least_four_slower_rows': [r for r in assessment['timing_rows'] if r['slower_blocks'] >= 4],
        'all_five_faster_rows': [r for r in assessment['timing_rows'] if r['faster_blocks'] == 5],
        'nonzero_memory_rows': [r for r in assessment['memory_rows'] if any(v['median_block_delta'] for v in r['fields'].values())],
        'static_code_observations': static,
        'control_context': {'prior_outlined_decision_sha256': sha(prior), 'prior_outlined_all_five_slower_rows': 1,
            'fresh_outlined_all_five_slower_rows': 4, 'prior_rejection_unchanged': True,
            'interpretation': 'Fresh outlined timing directions differ from its earlier closed trial despite identical sources. Both records remain. No earlier sample is pooled, extended or used to erase an adverse result.'},
        'limits': assessment['limits'] + [
            'Substantial variation remains in favorable and adverse rows. For rational multiradical construction/first, median paired ratios are 1.083040/1.046091 while ratios of medians are 1.654946/1.606915. Both summaries and every observation remain; there is no significance or application-speedup claim.',
            'Borrowed versus outlined radical-Laurent cloned totals have medians 16519 versus 16079 ns over 4096 inputs. This timer-scale adverse row remains without claiming a practical effect.',
            'The borrowed helper shrinks from 1442 to 1052 code bytes, while the entry grows from 2800 to 2993. Their combined symbol bytes fall by 197, but both entry stack subtractions remain 264 bytes and both helper subtractions 104. These are static observations, not total/peak stack use or a causal explanation.',
            'Disassembly is losslessly compressed with independent raw-output hashes. Internal helper jumps are not additional multiplier-to-helper calls.',
        ],
        'native': {'status': 'not-executed-after-scalar-cost-rejection', 'original_fixture_gates_run': False,
            'matched_native_costs_run': False, 'complete_report_identity_established': False,
            'production_cli_size_measured': False, 'original_limits': preparation['original_limits']},
        'next_step': {
            'action': 'Close the current direct/outlined/borrowed same-radicand series and return to native cost attribution at the unchanged endpoint/P4 parent.',
            'review': 'Re-read the retained native profile and operation-count diagnostics alongside the parent normalizer/certificate-consumer code. Identify repeated work that can be removed before proposing another optimization; these helper changes preserve the expensive arithmetic and have not established an acceptable cost improvement.',
            'diagnostic_if_needed': 'If the existing evidence does not isolate avoidable work, prepare a separately frozen fixed profiling or counter run for the unchanged parent on the complete boards. Preserve the original assertion and label instrumented observations as attribution, not matched benchmark evidence.',
            'constraints': 'Keep all sources and samples frozen. Do not extend a closed trial, schedule another equivalent helper-boundary variant, alter proof/memo/budget semantics or raise the original fixture limits.',
            'new_source_or_diagnostic_prepared': False,
            'full_scope': 'All 39 properties, 14 targets and crate future work remain in scope. Individual ledger statuses remain unchanged; the goal is active and incomplete.'},
        'main_and_35_pending_native_paths_unchanged': True, 'runtime_admitted': False, 'plan_complete': False}
    verify(preparation)
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'status': decision['status'], 'summaries': summaries, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
