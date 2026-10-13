"""Retain the scoped trial for investigation without admitting pending runtime."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    validation = json.loads((E / 'validation-assessment-v1.json').read_text())
    cost = json.loads((E / 'cost-assessment-v1.json').read_text())
    reports = json.loads((E / 'report-assessment-v1.json').read_text())
    preparation = json.loads((E / 'preparation.json').read_text())
    for assessment in [validation, cost, reports]:
        assert not assessment['runtime_admitted'] and not assessment['plan_complete']
        for name, digest in assessment['artifacts_sha256'].items():
            assert sha(E / name) == digest
    assert reports['complete_reports_byte_identical'] and reports['complete_report_values_identical']
    assert json.loads((E / 'report-replay-v1/protocol.json').read_text())['extracted_input_files'] == 108
    assert validation['test_counts']['curve-tests'] == {'summaries': 51, 'passed': 2489, 'failed': 0, 'ignored': 33}
    assert validation['test_counts']['drc-regressions']['passed'] == 26
    gates = {row['label']: row for row in validation['native_gates']}
    assert not gates['full-board']['passed'] and gates['copper-neck']['passed']
    assert gates['full-board']['observed_assertion_seconds'] == 146.02494178
    assert cost['memory_median_increases'] == []
    for row in cost['timing']:
        if row['case'] in ['unequal_sources', 'mixed_sources']:
            assert row['candidate_faster_pairs'] == 5
    for row in cost['memory']:
        for metric in ['retained_bytes', 'peak_extra_bytes']:
            assert row['metrics'][metric]['paired_deltas'] == [0] * 5
    original = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    current = original | preparation['source_changes_sha256']
    assert all(sha(Path(preparation['baseline_source']) / name) == digest for name, digest in original.items())
    assert all(sha(Path(preparation['candidate_source']) / name) == digest for name, digest in current.items())
    assert sources() == preparation['main_source_sha256']
    native = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert len(native) == 35 and all(sha(ROOT / name) == digest for name, digest in native.items())
    next_step = ('Profile the frozen metadata-first source with the complete original boards in a separate target, keeping the original assertions and all existing inputs. '
                 'Identify a remaining concrete caller or allocation site before proposing another optimization. Preserve the rejected point-equality and coefficient-merge experiments. '
                 'The 120-second full-board CPU gate, broader native validation, pending scalar-cache tradeoffs and accurate two-pass budget documentation still precede runtime admission.')
    decision = {
        'decision': 'retain-isolated-metadata-order-for-further-investigation',
        'goal_turn_classification': 'progress',
        'basis': [
            'The prior unchanged-query diagnostic observed source mismatch in 34023542 of 34056513 comparisons and 99.801% of instrumented geometry-query time. That is an opportunity observation, not predicted savings.',
            'The full curve run passes 2489 tests/doctests with 33 unchanged ignores; 26 focused DRC tests, strict consumer Clippy, formatting and warning-denied curve documentation pass.',
            'All fixed unequal/mixed metadata probe pairs are faster and allocate less. Every measured retained and peak-extra allocation pair agrees.',
            'Both complete original-board JSON reports match byte for byte with no normalization. The same findings, inputs and complete report fields are retained.',
        ],
        'remaining_tradeoffs': [
            'The original full-board worker CPU assertion fails at 146.02494178 seconds / 120. The separate original 60-second copper-neck assertion passes. No runtime is admitted.',
            'Absent-source first validation has a 1.030 median ratio with four slower pairs. Duplicate first validation has a 1.056 median including one 2.091 ratio sample; warm duplicate rejection is 1.022 with all five pairs slower. Preserve these observations.',
            'Local operation ratios and the four standalone report runs do not establish a matched whole-application speedup. Named regressions and report identity do not prove universal equality or cache completeness.',
            'Standalone replay peak RSS is 1958196 to 1960744 KiB for CPArti and 554272 to 556240 KiB for HVP. These observed increases are preserved; one run per variant/board does not isolate causality or establish unchanged process memory.',
            'The inherited persistent-cache and integer-leaf candidates retain their original resource, memory, API-documentation and admission obligations. The rejected coefficient merge is absent.',
        ],
        'validation': {key: validation[key] for key in ['test_counts', 'count_overlap', 'native_gates']},
        'cost_summary': {key: cost[key] for key in ['all_five_time_pairs_slower', 'all_five_time_pairs_faster', 'memory_median_increases']},
        'reports': reports['comparisons'],
        'production_binary_byte_delta': reports['production_binary_byte_delta'],
        'next_step': next_step,
        'evidence_sha256': {name: sha(E / name) for name in [
            'preparation.json', 'validation-assessment-v1.json', 'cost-assessment-v1.json', 'report-assessment-v1.json']},
        'pending_main_native_entries_unchanged': 35,
        'scope': 'All 39 properties, all 14 targets and crate-specific future work remain active. No property or target is declared complete here.',
        'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'decision-v1.json').write_text(json.dumps(decision, indent=2) + '\n')
    status_path = ROOT / 'docs/properties-status.json'
    status = json.loads(status_path.read_text())
    assert len(status['properties']) == 39 and len(status['targets']) == 14
    work = status['current_work']['isolated_region_metadata_first']
    work['status'] = decision['decision']
    work['decision_evidence'] = str((E / 'decision-v1.json').relative_to(ROOT))
    work['report_evidence'] = str((E / 'report-assessment-v1.json').relative_to(ROOT))
    work['report_summary'] = 'Both complete original-board reports are byte-identical without normalization across four fresh CLI processes reading the same 108 extracted inputs. Production executable size delta: ' + str(reports['production_binary_byte_delta']) + ' bytes. Standalone replay durations are not a matched speedup or a replacement for either original cap.'
    work['next_step'] = next_step
    status['current_work']['pending_native_geometry']['next_step'] = next_step
    status_path.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({'decision': decision['decision'], 'original_full_board_gate_passed': False,
                      'complete_reports_byte_identical': True, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
