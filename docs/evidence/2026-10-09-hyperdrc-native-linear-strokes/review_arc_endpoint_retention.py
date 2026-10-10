"""Close the endpoint experiment using every completed gate and adverse control."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
records = {number: read(EVIDENCE / f'diagnostics-{number}.json') for number in range(77, 84)}
verified = 0
for record in records.values():
    for name, artifact in record['artifacts'].items():
        path = EVIDENCE / name
        assert path.stat().st_size == artifact['bytes'], name
        assert digest(path) == artifact['sha256'], name
        verified += 1
native = read(EVIDENCE / 'native-exactness-gate/sources.json')['source_hashes']
assert all(digest(ROOT / name) == value for name, value in native.items())
assert sum(records[77]['identical_receipt_rows_per_variant'].values()) == 63652
assert records[77]['identical_query_receipts_per_variant'] == 445564
assert records[80]['final_query_abort_verified']
assert records[81]['historical_witnesses_per_process'] == 231
assert records[82]['all_231_historical_witnesses_preserved']
assert records[82]['no_adaptive_extension_or_discarded_samples']
assert all(not row['passed'] for row in records[83]['board_gates'].values())
rows = {row['name']: row for row in read(EVIDENCE / 'arc-endpoint-broad-followup/split-comparison.json')}
selected = ['operation_point_pairproximity_huge_centers', 'operation_point_pairreturn_huge_centers', 'operation_fixed_decimal_pi', 'operation_fixture_thermal_sparse_2002', 'operation_fixture_coverage_sparse_2002', 'operation_overlapping_duplicate_layer_area']
controls = {}
for name in selected:
    controls[name] = {stage: {key: row[key] for key in ['candidate_over_baseline_median', 'pairs_candidate_slower', 'pairs']} for stage, row in rows[name]['stages'].items()}
assert controls['operation_overlapping_duplicate_layer_area']['combined_fifteen']['pairs_candidate_slower'] == 14
assert controls['operation_overlapping_duplicate_layer_area']['additional_ten']['pairs_candidate_slower'] == 10
record = {
    'decision': 'not retained: focused endpoint savings do not establish an acceptable cross-workload performance tradeoff',
    'runtime_action': 'No endpoint candidate patch is applied to main. The isolated sources, exact patches, complete receipt/report evidence and every timing sample remain preserved. The admitted runtime remains the contact-index exactness checkpoint; the larger native prototype stays unadmitted.',
    'correctness': {
        'focused_receipt_rows_identical': 63652, 'focused_query_receipts_identical': 445564,
        'effective_final_coordinate_abort': '210 receipt rows identical in three fresh processes per variant; original intermediate-flag coverage correction preserved.',
        'full_scoped_production_suites': '2430 curve tests in 49 binaries with 28 documented ignores, 1559 DRC library tests, 35 DRC integration tests; strict curve/DRC Clippy and rustdoc and formatting pass.',
        'historical_witnesses': 'All 231 rows preserved in all thirty combined broad-replay processes.',
        'complete_production_reports': records[81]['reports'],
        'native_focused_tests_each_variant': 72,
        'limitation': 'No correctness mismatch was observed in this scope. Finite receipt corpora do not prove universal cache-frontier, abort-scheduling or budget equivalence. Matching native smoke counts are not full native report-byte preservation.',
    },
    'favorable_resource_evidence': {
        'targeted_queries': 'All fifteen intended non-endpoint first/warm/cloned query rows improve 18.4% to 42.3%; fourteen have nonoverlapping five-sample ranges. Construction and unchanged controls are not credited to endpoint reuse.',
        'allocation_overview': records[78]['allocation_overview'],
        'executable_sizes': records[78]['executable_sizes'],
    },
    'broad_resource_evidence': {'overview': records[82]['overview'], 'representative_controls': controls, 'all_rows': 'arc-endpoint-broad-followup/split-comparison.json'},
    'application_evidence': {
        'production_complete_report_runs': 'Baseline wall times 54.4/55.3 seconds and candidate 54.8/55.1 seconds do not demonstrate a compensating application speedup.',
        'native_original_board_gates': records[83]['board_gates'],
        'native_single_pair': records[83]['descriptive_single_pair'],
    },
    'assessment': 'The local exact-negative certificate has a sound value-level argument and preserves all recorded correctness witnesses. The narrow savings are therefore retained as evidence, but do not justify changing runtime globally: broad controls retain roughly 3-7% slower medians, sparse fixture coverage is 10.4% slower, and duplicate-layer area is slower in 14/15 pairs, including all ten additional pairs. Complete production report runs and the separate native pair show no compensating application gain. Both native versions still fail the original 120-second worker CPU gate at about 195 seconds. This is a conservative retention decision under P36 and T11, not proof that the source edit causally slows every adverse row or that it has a correctness defect.',
    'sampling_closed': 'The one fixed ten-pair extension is complete. No further sampling or selective deletion for these production binaries; no additional native timing pair is needed to close this candidate.',
    'limitations': 'Sample ranges overlap and are descriptive, not confidence intervals. The 231 workload comparisons are correlated and no statistical-significance claim is made. A single native pair cannot establish a universal performance ratio. No full native suite, complete repository pre-push validation, entire property/target or general scalar/geometry resource bound is claimed.',
    'next_step': 'Return to the recorded scalar alternate/scientific-formatting stall: reproduce the original unresolved-hypot input in bounded child processes and distinguish absolute decimal precision from scientific significant-digit requirements before choosing a typed completion contract. The complete native prototype and all other property/target work remain open.',
    'artifact_references_verified': verified, 'main_source_hashes_verified': len(native), 'main_runtime_unchanged': True,
    'inputs': {f'diagnostics-{number}.json': {'bytes': (EVIDENCE / f'diagnostics-{number}.json').stat().st_size, 'sha256': digest(EVIDENCE / f'diagnostics-{number}.json')} for number in records},
    'review_script_sha256': digest(Path(__file__)),
    'review_metadata_correction': {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted((EVIDENCE / 'arc-endpoint-retention-review-correction').iterdir())},
}
(EVIDENCE / 'diagnostics-84.json').write_text(json.dumps(record, indent=2) + '\n')
print(record['decision'])
print('Verified artifact references:', verified, '; main source hashes:', len(native))
