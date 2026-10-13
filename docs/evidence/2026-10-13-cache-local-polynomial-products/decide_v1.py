"""Close the cache-local product trial after its original native gate fails."""
import hashlib
import json
from pathlib import Path

from qualify_v1 import verify

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'decision-v1.json'
    assert not destination.exists()
    verify(json.loads((E / 'preparation-v1/protocol.json').read_text()))
    scalar = json.loads((E / 'scalar-decision-v2.json').read_text())
    native = json.loads((E / 'native-assessment-v2.json').read_text())
    result = json.loads((E / 'native-validation-v2/result.json').read_text())
    assert result['complete'] and not result['all_passed']
    assert result['source_parent_main_and_binaries_unchanged']
    assert not native['all_original_gates_passed']
    assert native['strict_four_crate_clippy_passed_for_candidate']
    assert native['warning_denied_four_crate_docs_passed_for_candidate']
    for name, digest in native['artifacts_sha256'].items():
        assert sha(E / name) == digest, name
    cost = json.loads((E / 'cost-assessment-v1.json').read_text())
    assert cost['all_mathematical_witnesses_and_representation_inventories_match']
    for name, digest in cost['evidence_sha256'].items():
        assert sha(E / name) == digest, name
    board, neck = native['native_gates']
    assert board['label'] == 'full-board' and not board['passed']
    assert board['limit_seconds'] == 120 and board['metric'] == 'worker CPU'
    assert board['observed_assertion_seconds'] == 136.412555334
    assert neck['label'] == 'copper-neck' and neck['passed']
    assert neck['limit_seconds'] == 60 and neck['metric'] == 'check body wall time'
    assert neck['observed_assertion_seconds'] is None
    for name in ['native-controls-v1', 'native-controls-assessment-v1.json',
                 'report-replay-v1', 'report-assessment-v1.json']:
        assert not (E / name).exists(), name
    inputs = ['preparation-v1/protocol.json', 'correctness-v1.json',
              'cost-assessment-v1.json', 'cost-v1/result.json',
              'scalar-decision-v1.json', 'scalar-decision-v2.json',
              'review_scalar_decision_v1.py', 'native-assessment-v2.json',
              'native-validation-v2/result.json', 'native-cost-inputs-v1.json',
              'measure_native_v1.py', 'assess_native_controls_v1.py']
    decision = {
        'status': 'reject-cache-local-products-on-original-board-gate-and-unresolved-scalar-costs',
        'decision_runner_sha256': sha(Path(__file__)),
        'inputs_sha256': {name: sha(E / name) for name in inputs},
        'decision': 'Reject this cache-local product candidate for runtime admission. The original board assertion fails at 136.412555334 worker-CPU seconds against 120. Conjugate-then-affine cloned queries remain slower in every scalar pair and warm queries in four. Nine scalar phases save allocations without increased measured retention, but those lower-rung savings do not resolve the gate failure or timing concerns. This decision does not attribute the standalone board time to the candidate or claim a matched application slowdown.',
        'closed_sample_set': scalar['closed_sample_set'],
        'scalar_summary': scalar['summary'],
        'scalar_slower_in_all_five_blocks': scalar['slower_in_all_five_blocks'],
        'scalar_slower_in_at_least_four_blocks': scalar['slower_in_at_least_four_blocks'],
        'scalar_faster_in_all_five_blocks': scalar['faster_in_all_five_blocks'],
        'unresolved_scalar_costs': scalar['unresolved_costs'],
        'favorable_scalar_costs': scalar['favorable_costs'],
        'native_test_counts': native['test_counts'],
        'native_gates': native['native_gates'],
        'native_costs': {
            'status': 'prepared-not-executed-after-original-gate-rejection',
            'measurement_processes_executed': 0,
            'preflight_processes_executed': 0,
            'reason': 'The unchanged original gate and unresolved scalar costs prevent admission. The prepared fixed native controls remain intact and unexecuted; no native cost identity or speedup is claimed.',
        },
        'reports_and_production_size': {
            'complete_report_identity_established': False,
            'production_cli_size_measured': False,
            'finding_counts': [12, 197],
            'status': 'not-executed-for-rejected-candidate',
        },
        'limits': native['limits'] + [
            'The native assessment is a qualification record. Its provisional decision label awaiting controls is superseded by this rejection; the executed assessor and all logs remain unchanged.',
            'The original endpoint/P4 parent also failed a separate historical board assertion. These standalone runs are not a paired comparison and establish no causal regression or speedup.',
            'Passing copper-neck does not expose its internal measured duration. The recorded 123.42606854019687-second subprocess wall time includes setup and must not be compared with the 60-second check-body limit.',
            'All scalar samples remain closed. No samples were discarded, appended or selectively repeated. Ratios are descriptive and do not establish statistical significance.',
            'The 35 original pending native paths and main runtime remain unchanged. Neither this experiment nor its tested parent is admitted as a native runtime solution.',
        ],
        'next_step': {
            'status': 'source-review-pending',
            'task': 'Keep this trial frozen. Review the unchanged parent coefficient multiplier for a direct product of two single-radical coefficients with the same radicand. Establish exact operation order, zero/cancellation behavior, insertion bounds, opaque fallback and storage ownership before preparing a distinct candidate. Do not inherit cache-local borrowing, rejected sharing/storage changes or scaling branches.',
            'motivation': 'Prior diagnostics observed many single-radical recursive certificate reads, but those are read counts, not same-field multiplication coverage or time share. Review the multiplication-shape evidence before attributing potential benefit or deciding whether a runtime prototype is justified.',
            'candidate_prepared': False,
        },
        'source_parent_main_and_pending_native_unchanged': True,
        'runtime_admitted': False, 'native_runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'status': decision['status'], 'native_gates': decision['native_gates'],
                      'native_costs_executed': False, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
