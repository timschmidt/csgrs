"""Apply the predeclared rejection rule without changing any implementation."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import RUNTIME, W, dump, read, sha

build = read(RUNTIME / 'build-v1/result.json')
preflight = read(RUNTIME / 'cost-preflight-v1/result.json')
matched = read(RUNTIME / 'cost-matched-v1/result.json')
assert all(row['complete'] and row['output_closed'] and not row['command_failures']
           for row in [build, preflight, matched])
assert preflight['process_samples'] == 12 and matched['process_samples'] == 84
assert matched['phase_observations'] == 168
assert {(row['count'], row['phase']) for row in matched['decisive_rejection_cases']} == {
    (4, 'cold'), (8, 'cold'), (9, 'cold')}
previous = W.parents[2] / 'docs/evidence/2026-10-17-circle-support-guarded/decision-v1.json'
old = read(previous)
inputs = [Path(__file__), W / 'contract-v1.md', W / 'progress-v1.md', W / 'build_v1.py',
          W / 'runner_v1.py', W / 'measure_v1.py', W / 'small_batch_probe_v1.rs', previous]
for name in ['build-v1', 'cost-preflight-v1', 'cost-matched-v1']:
    inputs += [RUNTIME / name / filename for filename in ['protocol.json', 'commands.json', 'result.json']]
inputs += [RUNTIME / 'cost-matched-v1' / filename for filename in ['samples.json', 'summary.json', 'witnesses.json']]
dump(W / 'decision-v1.json', {
    'main_parent_commit': '6479d3ba982777a1e9ecc8ce24ef83752d2e474b',
    'status': 'rejected: guarded index has stable 16–19.5% cold regressions on completed small batches',
    'retention_decision': 'rejected', 'cache_index_admitted': False,
    'source_changes_applied': False, 'main_sources_modified': False, 'implementation_sha256': {},
    'proposal_sha256': old['proposal_sha256'], 'previous_qualification_decision': str(previous),
    'input_sha256': {str(path): sha(path) for path in inputs},
    'results': {'build': build, 'preflight': preflight, 'matched': matched,
                'complete_phase_summary': read(RUNTIME / 'cost-matched-v1/summary.json')},
    'native_disposition_preserved': True, 'pending_native_paths_remaining': 0,
    'full_archived_integration_admitted': False, 'native_runtime_admitted': False,
    'full_plan_complete': False,
    'limitations': [
        'Private-kernel no-feature matched diagnostic, not an end-to-end or shipped-package speed claim.',
        'No new allocation or size claim; E42 evidence remains preserved.',
        'Remaining downstream, full Cargo feature/target/manifest, native/report and real binding qualification is not run for this rejected version.',
        'No internal total-work or cache-byte bound; external containment and fixed resource reserves apply.',
        '27,093 inherited physical artifact paths remain missing; no whole-history preservation claim.',
        'Full release, pre-push, coverage, directed campaign, formal proof and properties-plan work remains open.',
    ],
    'next': [
        'Prototype lazy index construction only in a fresh isolated revision; reproduce and preserve these small-batch results, the earlier shared-coefficient regression and the full exact support corpus.',
        'Require a successor to pass the original native workload limits, exact report contracts, full affected consumers and real bindings before any admission.',
        'Continue ordinary strict-mode completion, typed failures, end-to-end evidence and clean external distribution under the complete properties plan.',
    ],
})
print('Recorded rejection of the unchanged E42 guarded proposal.')
