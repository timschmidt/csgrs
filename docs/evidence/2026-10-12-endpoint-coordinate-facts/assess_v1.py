"""Verify the immutable endpoint diagnostic and bound the supported next trial."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'diagnostic-v1'
PARENT = ROOT / 'docs/evidence/2026-10-12-region-metadata-first'
BASE = ROOT / 'target/region-metadata-first/source'
SOURCE = ROOT / 'target/endpoint-coordinate-facts/diagnostic-v1/source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    result = json.loads((OUT / 'result.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    parent = json.loads((PARENT / 'preparation.json').read_text())
    baseline = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes()))
    baseline |= parent['source_changes_sha256']
    expected = baseline | protocol['source_changes_sha256']
    assert result['complete'] and not result['runtime_admitted']
    assert sources() == parent['main_source_sha256']
    assert all(sha(BASE / name) == digest for name, digest in baseline.items())
    assert all(sha(SOURCE / name) == digest for name, digest in expected.items())
    assert sha(E / 'diagnose_v1.py') == protocol['runner_sha256']
    assert sha(OUT / 'instrumentation.tar.gz') == protocol['archive_sha256']
    assert sha(OUT / 'instrumentation.patch') == protocol['patch_sha256']
    binary = json.loads((OUT / 'binary.json').read_text())
    assert sha(Path(binary['path'])) == binary['sha256']
    assert Path(binary['manifest_path']) == SOURCE / 'crates/hyperdrc/Cargo.toml'
    for phase in result['phases']:
        assert phase['sources_unchanged']
        assert phase['exit_code'] == (101 if phase['name'] == 'full-board' else 0)
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (phase['name'] + '.' + stream)) == phase[stream + '_sha256']
    test_counts = {}
    for name, count in [('diagnostic-regressions', 3), ('point-equality-regressions', 5), ('arc-membership-regression', 1)]:
        text = (OUT / (name + '.stdout')).read_text()
        assert f'test result: ok. {count} passed; 0 failed; 0 ignored;' in text
        test_counts[name] = count
    stderr = (OUT / 'full-board.stderr').read_text()
    stdout = (OUT / 'full-board.stdout').read_text()
    assert stderr.count('hyperdrc: finished emit report') == 2
    assert 'skipping complex project fixture test' not in stdout + stderr
    worker = float(re.search(r'smoke suite took ([0-9.]+)s of worker time', stderr)[1])
    rows = json.loads((OUT / 'counters.json').read_text())
    assert len(rows) == result['rows'] == 19
    assert {row['policy'] for row in rows} == {0}
    assert sum(row['calls'] for row in rows) == result['observed_endpoint_calls'] * 2
    for endpoint in [0, 1]:
        assert sum(row['calls'] for row in rows if row['endpoint'] == endpoint) == result['observed_endpoint_calls']
    negative = [row for row in rows if row['initial_proof'] == 2 and row['policy_result'] != 3]
    unknown = [row for row in rows if row['initial_proof'] == 3 and row['policy_result'] != 3]
    strengthened = [row for row in unknown if row['policy_result'] == 2]
    assert all(row['repeated_proof'] == 2 and row['policy_result'] == 1 for row in negative)

    def summarize(selected):
        return {key: sum(row[key] for row in selected) for key in ['calls', 'initial_nanos', 'retry_nanos']}

    totals = summarize(rows)
    negative_totals = summarize(negative)
    next_step = (
        'Fork the frozen metadata-first parent into a separate source and target. Expose only the tri-state result of the same lazy coordinate zero queries used by Point2 equality, '
        'without changing global PartialEq. In contains_endpoint, keep shared/either-endpoint success first and skip the existing policy retry only after an actual NonZero coordinate proof. '
        'Keep all Unknown retries, the later whole endpoint query, temporary drop boundaries and policy receipts. '
        'Qualify negative/unknown/shared/independently equal cases, cold/warm/cloned facts, active and cleared abort, bounded/shared budgets, preview/approximation consumption, '
        'historical capability witnesses, fixed costs and memory, full curve validation, complete reports and both original native limits. Correct the inherited two-pass normal-form budget documentation before admission.'
    )
    assessment = {
        'decision': 'advance-isolated-negative-coordinate-proof-reuse-trial',
        'scope': 'Existing query order and short circuits preserved; only returned coordinate facts observed. This is an instrumented full-board diagnostic, not runtime qualification.',
        'observed_endpoint_calls': result['observed_endpoint_calls'],
        'per_endpoint_rows': len(rows),
        'totals': totals,
        'negative_followed_by_policy_retry': negative_totals,
        'negative_retry_percent_of_all_retry_nanos': 100 * negative_totals['retry_nanos'] / totals['retry_nanos'],
        'initial_unknown_followed_by_policy_retry': summarize(unknown),
        'initial_unknown_later_policy_true': summarize(strengthened),
        'negative_coordinate_contradictions_observed': 0,
        'policy_categories_observed': [0],
        'validation': test_counts,
        'format_and_strict_release_all_feature_all_target_curve_clippy_pass': True,
        'full_board': {'worker_cpu_seconds': worker, 'limit_seconds': 120, 'passed': False, 'both_reports_emitted': True},
        'limits': [
            'Instrumentation changes timing and allocation behavior. The measured retry durations are not predicted net savings or matched release speedup.',
            'Rows are marginal per-endpoint observations, not joint start/end proofs. An overall true answer after a negative coordinate proof may be caused by the other endpoint.',
            'Observed policy category 0 has no preview, approximate-terminal permission or bounded-pass flag. Flags indicate permission, not consumed approximation receipts. Other policies still require dedicated controls.',
            'Only exiting-thread counters are collected. Focused diagnostic, equality and arc regressions pass; no complete curve suite or full report identity replay is claimed here.',
            'The earlier rejected trial omitted a second whole endpoint call after radius classification. This next trial only proposes preserving the negative facts from the first coordinate comparisons; the rejected omission remains excluded.',
            'All 39 properties, 14 targets and crate-specific future work remain in scope. The original full-board failure and all earlier cost tradeoffs remain open.',
        ],
        'next_step': next_step,
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'main_runtime_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    destination = E / 'assessment-v1.json'
    assert not destination.exists()
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = json.loads(ledger_path.read_text())
    assert len(ledger['properties']) == 39 and len(ledger['targets']) == 14
    ledger['current_work']['isolated_endpoint_coordinate_facts'] = {
        'status': assessment['decision'], 'evidence': str(destination.relative_to(ROOT)),
        'result': '2010162 completed endpoint calls; 2616583 per-endpoint policy retries follow an already certified negative coordinate result. Their instrumented time is 11.004793030 seconds of 15.762601851 total retry seconds. No observed negative coordinate proof was contradicted by its same-endpoint retry.',
        'required_unknown_retries': '113161 retries follow inconclusive coordinate comparisons; 158 later policy queries prove equality. These retries must remain.',
        'validation': test_counts, 'full_board': assessment['full_board'],
        'limits': 'Only ordinary exact policy category 0 was observed in the full-board rows. Instrumented time is not a speedup estimate. Marginal endpoint counts are not joint facts.',
        'next_step': next_step, 'runtime_admitted': False,
    }
    ledger['current_work']['pending_native_geometry']['next_step'] = next_step
    ledger_path.write_text(json.dumps(ledger, indent=2) + '\n')
    print(json.dumps({key: assessment[key] for key in ['decision', 'negative_followed_by_policy_retry', 'initial_unknown_later_policy_true', 'full_board']}))


if __name__ == '__main__':
    main()
