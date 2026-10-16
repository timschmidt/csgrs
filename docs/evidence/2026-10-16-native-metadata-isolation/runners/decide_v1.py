"""Record the closed isolated trial without converting diagnostic evidence to admission."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parents[1]


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


assert not (E / 'decision-v1.json').exists()
qualification = json.loads((E / 'qualification-assessment-v1.json').read_text())
cost = json.loads((E / 'cost-assessment-v1.json').read_text())
reports = json.loads((E / 'report-replay-v1/result.json').read_text())
assert reports['complete']
assert cost['decision'] == 'fixed-operation-observations-complete'
gates = qualification['original_native_gates']
assert not gates['full-board']['passed'] and not gates['copper-neck']['passed']
assert gates['gerber']['passed']
assert all(row['exit_code'] == 0 for row in qualification['phases']['candidate-v1']['commands']
           if row['name'] not in {'full-board', 'copper-neck'})
binaries = json.loads((E / 'report-replay-v1/protocol.json').read_text())['binaries']
record = {
    'status': 'isolated-metadata-order-trial-closed-with-runtime-gates-failing',
    'decision': 'Do not admit this runtime candidate or the 35 pending native files on this evidence.',
    'decision_reason': 'All non-runtime qualification commands pass, but the isolated candidate fails both original native runtime assertions. Local constructor measurements and complete report comparison do not satisfy those workload limits.',
    'baseline_commit': '73472622dd4034b8b8777cb2a116c4020f6bb328',
    'owning_layer': 'Hypercurve retained region validation',
    'production_difference': 'Compare immutable arrangement-source records before geometric equality in the existing duplicate-evidence conjunction.',
    'production_or_test_edits': 0,
    'isolated_production_files_changed': 1,
    'identical_focused_tests_per_variant': 3,
    'pending_main_native_files_unchanged': 35,
    'phases': qualification['phases'],
    'original_native_gates': gates,
    'report_replay': reports,
    'local_cost': {
        'fresh_processes': 20, 'raw_rows': 360, 'timing_cases_and_phases': 18,
        'time_inputs_per_case': 1024, 'memory_inputs_per_case': 128,
        'all_timing_rows': [{key: row[key] for key in
                            ['case', 'phase', 'median_paired_ratio', 'candidate_faster_pairs']}
                           for row in cost['timing']],
        'all_five_time_pairs_slower': cost['all_five_time_pairs_slower'],
        'all_five_time_pairs_faster': cost['all_five_time_pairs_faster'],
        'memory_median_increases': cost['memory_median_increases'],
        'full_assessment': 'cost-assessment-v1.json',
    },
    'production_cli_binary_bytes': {variant: row['bytes'] for variant, row in binaries.items()},
    'production_cli_binary_delta_bytes': binaries['candidate']['bytes'] - binaries['baseline']['bytes'],
    'limitations': [
        'The baseline and candidate use the same 35 separately unadmitted native changes; this trial does not qualify that whole bundle.',
        'Feature and focused-test counts overlap. Existing ignored tests remain ignored and their obligations remain open.',
        'Exact authored regression facts and retained-proof checks supplement full report equality; report agreement alone is not an independent geometry oracle or a universal cache proof.',
        'Point2 equality still treats unresolved symbolic equality as false; this trial does not establish complete geometric equality for arbitrary expressions.',
        'The original timing assertions measure worker CPU or check-body wall time. Whole-command durations also include setup and compilation. These standalone application runs do not establish a matched application speedup.',
        'The fixed paired costs are local instrumented constructor controls, including expected duplicate rejection rather than successful modeling. All unfavorable rows are retained; TLS allocation deltas are not a global heap bound.',
        'External 1800-second command and 8-GiB per-process address-space containment do not supply internal total-work, cancellation or retained-heap contracts.',
        'Clean external distribution, MSRV and release checks, full workflow validation, formal verification, directed campaigns and instrumented coverage remain open.',
        'Historical missing artifacts remain inherited; no historical-completeness claim.',
    ],
    'next': 'Reproduce the format-boundary composition hypotheses in followup-inspection-v1.md under a fresh acceptance contract before further performance work. Native runtime admission remains open; preserve the original limits, full fixtures and rejected scalar experiments.',
    'evidence_sha256': {name: sha(E / name) for name in
                        ['contract-v1.md', 'qualification-assessment-v1.json',
                         'cost-assessment-v1.json', 'report-replay-v1/result.json']},
    'runtime_admitted': False, 'application_speedup_claimed': False,
    'full_plan_complete': False,
}
if cost['all_five_time_pairs_slower']:
    record['decision_reason'] += ' Local controls slower in all five pairs: ' + ', '.join(cost['all_five_time_pairs_slower']) + '.'
if not reports['all_reports_identical']:
    record['decision_reason'] += ' Complete report equality also fails.'
(E / 'decision-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'status': record['status'], 'gates': gates,
                  'all_reports_identical': reports['all_reports_identical'],
                  'all_five_time_pairs_slower': cost['all_five_time_pairs_slower']}), flush=True)
