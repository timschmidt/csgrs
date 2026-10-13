"""Checkpoint terminal preparation and focused checks while broad validation runs."""
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
    preparation = json.loads((E / 'preparation.json').read_text())
    protocol = json.loads((E / 'validation-v1/protocol.json').read_text())
    live = json.loads((E / 'validation-v1/result.json').read_text())
    expected = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes())) | preparation['source_changes_sha256']
    assert sources() == preparation['main_source_sha256']
    assert all(sha(Path(preparation['candidate_source']) / name) == digest for name, digest in expected.items())
    assert sha(E / 'prepare_v1.py') == preparation['runner_sha256']
    assert sha(E / 'validate_v1.py') == protocol['runner_sha256']
    assert sha(E / 'candidate-delta.tar.gz') == preparation['candidate_archive_sha256']
    assert sha(E / 'candidate.patch') == preparation['candidate_patch_sha256']
    phases = [row for row in live['phases'] if row['name'] in ['format', 'endpoint-regressions', 'strict-consumer-clippy']]
    assert len(phases) == 3
    artifacts = {}
    for phase in phases:
        assert phase['exit_code'] == 0 and phase['sources_unchanged']
        for stream in ['stdout', 'stderr']:
            path = E / 'validation-v1' / (phase['name'] + '.' + stream)
            assert sha(path) == phase[stream + '_sha256']
            artifacts[str(path.relative_to(E))] = sha(path)
    assert 'test result: ok. 5 passed; 0 failed; 0 ignored;' in (E / 'validation-v1/endpoint-regressions.stdout').read_text()
    next_step = 'Complete the running full curve suite, focused DRC tests, warning-denied scalar/curve docs and both unchanged native gates; then execute the prepared fixed paired cost, 231-witness, effective final-abort and full-report protocols. Preserve all failed gates and adverse timing rows. No runtime admission yet.'
    assessment = {
        'decision': 'isolated-endpoint-proof-preflight-passed-full-qualification-running',
        'runtime_delta': 'Private coordinate_equality retains the same lazy x/y zero facts. contains_endpoint keeps both early positive checks, then reuses a known negative only while edge preview and approximate terminals are inactive. Unknown retries, all approximate/preview retries and the later whole endpoint call remain. No owners are retained and global Point2 PartialEq is unchanged.',
        'documentation_delta': 'Normal-form APIs now describe at most two 512-expansion recognition passes and distinguish that bound from integer operand sizes; no rejected eight-node quadratic prepass is described or inherited.',
        'phases': phases, 'regression_functions_passed': 5,
        'coverage': [
            'Coordinate equality distinguishes true, certified false and unknown; an unknown x does not query y. Warm and cloned unknown coordinates remain unknown.',
            'Five representation families, eight endpoint/off-endpoint queries and seven policy contexts, each queried fresh and after preparation/repetition, agree with independently constructed controls executing the original endpoint sequence.',
            'Unknown comparisons still reach approximate and preview endpoint decisions; the terminal case retains Approximate512Consumed.',
            'Aborts are attached to final query coordinates; active/cleared/reenabled stages preserve shared equality, unknown facts and interval unavailability.',
            'Certified negative facts remain usable under zero work budgets; nested reuse cannot hide an exhausted outer budget.',
        ],
        'limitations': 'These are focused regressions, strict release all-feature all-target curve/DRC Clippy and formatting only. The full curve suite is still running. No native, full-report, cost or universal completeness claim is made by this checkpoint.',
        'artifacts_sha256': artifacts, 'next_step': next_step,
        'main_runtime_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    destination = E / 'preflight-assessment-v1.json'
    assert not destination.exists()
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = json.loads(ledger_path.read_text())
    assert len(ledger['properties']) == 39 and len(ledger['targets']) == 14
    ledger['current_work']['isolated_endpoint_coordinate_reuse'] = {
        'status': assessment['decision'], 'evidence': str(destination.relative_to(ROOT)),
        'implementation': assessment['runtime_delta'], 'qualification': assessment['limitations'],
        'documentation': assessment['documentation_delta'], 'next_step': next_step, 'runtime_admitted': False,
    }
    ledger['current_work']['pending_native_geometry']['next_step'] = next_step
    ledger_path.write_text(json.dumps(ledger, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'terminal_phases': len(phases), 'regressions_passed': 5}))


if __name__ == '__main__':
    main()
