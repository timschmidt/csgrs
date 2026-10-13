"""Close the memo-publication experiment without extending either sample set."""
import hashlib
import json
from pathlib import Path

from qualify_v1 import verify

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def timing_summary(rows):
    return {
        'timing_rows': len(rows),
        'all_five_slower_rows': sum(r['slower_blocks'] == 5 for r in rows),
        'at_least_four_slower_rows': sum(r['slower_blocks'] >= 4 for r in rows),
        'all_five_faster_rows': sum(r['faster_blocks'] == 5 for r in rows),
        'at_least_four_faster_rows': sum(r['faster_blocks'] >= 4 for r in rows),
        'ratio_of_medians_above_one_rows': sum(r['ratio_of_medians'] > 1 for r in rows),
    }


def main():
    destination = E / 'decision-v1.json'
    assert not destination.exists()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    verify(preparation)
    scalar = json.loads((E / 'scalar-decision-v1.json').read_text())
    native = json.loads((E / 'native-assessment-v1.json').read_text())
    controls = json.loads((E / 'native-controls-assessment-v1.json').read_text())
    result = json.loads((E / 'native-controls-v1/result.json').read_text())
    for name, digest in controls['artifacts_sha256'].items():
        assert sha(E / name) == digest, name
    assert result['complete'] and result['all_variant_identities_match']
    assert controls['broad_and_arc_variant_witnesses_identical']
    assert controls['historical_comparison']['all_231_historical_witnesses_identical']
    assert controls['final_coordinate_abort']['complete_receipts_identical']
    assert not native['all_original_gates_passed']
    assert not (E / 'report-replay-v1').exists()
    assert not (E / 'report-assessment-v1.json').exists()
    row = controls['variants']['candidate']
    endpoint = timing_summary(row['arc_timing'])
    broad = timing_summary(row['broad_timing'])
    assert endpoint['timing_rows'] == 48 and broad['timing_rows'] == 231
    assert endpoint['all_five_slower_rows'] == 5 and broad['all_five_slower_rows'] == 18
    fields = {}
    for field in row['arc_memory'][0]['median_block_deltas']:
        values = [r['median_block_deltas'][field] for r in row['arc_memory']]
        assert all(len(set(r['block_deltas'][field])) == 1 for r in row['arc_memory'])
        fields[field] = {
            'reduced_rows': sum(value < 0 for value in values),
            'increased_rows': sum(value > 0 for value in values),
            'min_delta': min(values), 'max_delta': max(values),
            'all_five_deltas_equal': True,
        }
    assert len(row['arc_memory']) == 48
    assert all(r['increased_rows'] == 0 for r in fields.values())
    decision = {
        'status': 'reject-closed-certificate-memo-on-unresolved-scalar-and-native-costs',
        'decision_runner_sha256': sha(Path(__file__)),
        'inputs_sha256': {name: sha(E / name) for name in [
            'preparation-v1/protocol.json', 'correctness-v1.json',
            'cost-assessment-v1.json', 'scalar-decision-v1.json',
            'native-assessment-v1.json', 'native-controls-assessment-v1.json',
            'native-controls-v1/result.json', 'replay_reports_v1.py', 'assess_reports_v1.py',
        ]},
        'decision': 'Reject this memo-publication candidate. It removes a redundant closed-result copy and reduces requested allocations while preserving the tested facts and ownership controls. Three scalar phases, five endpoint phases and eighteen broad operations are slower in every fixed pair, and the original full-board assertion still fails. These unresolved performance costs prevent retention under the performance-before-memory ladder. The native gains and smaller probes do not erase adverse rows.',
        'closed_sample_sets': {
            'scalar': scalar['closed_sample_set'],
            'native': {
                'endpoint_measurement_processes': 20, 'broad_measurement_processes': 10,
                'final_abort_processes': 6, 'preflight_processes': 4, 'blocks': 5,
                'final_abort_blocks': 3, 'timing_inputs_per_phase': 8192,
                'allocation_inputs_per_phase': 1000, 'extended': False,
                'all_witnesses_and_receipts_match': True,
            },
        },
        'scalar_summary': scalar['summary'],
        'endpoint_summary': endpoint, 'broad_summary': broad,
        'endpoint_memory': fields,
        'native_probe_binary_sizes': row['probe_binary_sizes'],
        'scalar_slower_in_all_five_blocks': scalar['slower_in_all_five_blocks'],
        'endpoint_slower_in_all_five_blocks': row['arc_slower_in_all_five_blocks'],
        'broad_slower_in_all_five_blocks': row['broad_slower_in_all_five_blocks'],
        'native_gates': native['native_gates'],
        'reports': {
            'status': 'prepared-not-executed-after-cost-rejection',
            'complete_report_identity_established': False,
            'production_cli_size_measured': False,
            'reason': 'Further report and production-size qualification cannot resolve the existing cost rejection. The prepared runners remain preserved and unexecuted.',
        },
        'limits': controls['limits'] + scalar['unresolved_costs'] + [
            'Endpoint cloned near-endpoint off-circle queries are slower in all five pairs: ratio of medians 1.146813 and median paired ratio 1.152364. Warm queries for that family are also slower in every pair, with ratio of medians 1.064517. These adverse rows have unchanged requested-memory counters.',
            'Some native samples vary substantially within each source, including the fourth broad block. Every block remains in the assessment. No causal attribution, statistical significance or universal slowdown is claimed.',
            'Six endpoint phases reduce allocation calls and allocated bytes, and five reduce peak requested bytes. Retained requested-byte deltas match across all 48 phases and all five pairs. These counters do not measure native C heaps or prove equal total memory.',
            'Four native probe binaries shrink by 232 to 248 bytes. These are probe sizes, not production CLI sizes.',
            'The candidate board assertion fails at 132.328922579 worker-CPU seconds against 120. The separate neck assertion passes its original 60-second check-body wall limit without printing that internal passing duration. Whole-process wall times and historical standalone gates are not matched application benchmarks.',
        ],
        'next_step': {
            'status': 'source-review-pending',
            'task': 'Keep this candidate, the parent and every completed trial frozen. Review the recursive persistent-certificate read path for unnecessary reconstruction of immutable rational/radical storage. Establish mutation, aliasing, ownership-release and original-retry requirements before preparing a distinct candidate. Do not use this rejected memo change in that baseline.',
            'motivation': 'The retained-projection diagnostic observed 35,239,829 recursive closed-certificate copies, including 30,843,135 single-radical copies. This is call coverage, not allocation counts, time share or an expected speedup. The memo-publication experiment does not eliminate reconstruction on subsequent cache reads.',
            'candidate_prepared': False,
        },
        'source_parent_main_unchanged': True,
        'runtime_admitted': False, 'native_runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'status': decision['status'], 'endpoint': endpoint, 'broad': broad,
                      'endpoint_memory': fields, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
