"""Verify scalar qualification without admitting either scaling variant."""
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
    assert not (E / 'correctness-v1.json').exists()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    protocol = json.loads((E / 'validation-v1/protocol.json').read_text())
    result = json.loads((E / 'validation-v1/result.json').read_text())
    original = json.loads(gzip.decompress((E / 'preparation-v1/parent-source-hashes.json.gz').read_bytes()))
    assert result['complete']
    assert sha(E / 'prepare_v1.py') == preparation['runner_sha256']
    assert sha(E / 'validate_v1.py') == protocol['runner_sha256']
    assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
    for phase in result['phases']:
        assert phase['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(E / 'validation-v1' / (phase['name'] + '.' + stream)) == phase[stream + '_sha256']
    for variant, row in preparation['variants'].items():
        assert all(sha(Path(row['source']) / name) == digest for name, digest in (original | row['source_changes_sha256']).items())
        assert sha(E / 'preparation-v1' / (variant + '-delta.tar.gz')) == row['delta_archive_sha256']
        assert sha(E / 'preparation-v1' / (variant + '.patch')) == row['patch_sha256']
    assert all(sha(Path(preparation['parent_source']) / name) == digest for name, digest in original.items())
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in preparation['pending_native_sha256'].items())
    tests = result['tests']
    assert next(row for row in tests if row['variant'] == 'baseline')['passed'] == 4
    for variant in ['right', 'both']:
        assert {row['phase']: row['passed'] for row in tests if row['variant'] == variant} == {
            'scaling-regressions': 4, 'cache-frontiers': 6, 'scalar-library': 867, 'scalar-doctests': 28}
    assessment = {
        'decision': 'both-isolated-scaling-variants-pass-scalar-correctness-await-fixed-costs',
        'assessor_sha256': sha(Path(__file__)), 'tests': tests,
        'implementation': {
            'baseline': 'Original runtime body plus the shared four-test module.',
            'right': 'Consume and scale the left coefficient when the right coefficient has no radical/opaque vectors.',
            'both': 'Also clone and scale the borrowed right coefficient when only the left is rational. This can change scalar operand/cache work and requires its own cost evidence.',
        },
        'preserved_source': 'The general coefficient multiplication body, ordered polynomial insertion, recognition budgets, retained normal-form cache and mandatory original fallback are unchanged. The two candidate deltas contain only the multiplier branches and the shared test module.',
        'regression_scope': 'Forty exact component cases in both directions; unsupported symbolic cross-product declines; opaque-owner reclamation; cold/warm/cloned cancellation to a quadratic certificate. The same four functions pass on the unchanged baseline. Six prior cache-frontier functions remain included in each full scalar total.',
        'checks': 'Both candidates pass full release all-feature scalar library and doctests, strict all-target scalar Clippy, workspace formatting and warning-denied scalar documentation.',
        'limits': 'Focused test counts overlap the 867 library results. This does not establish performance, memory, default/all-workflow consumer qualification, universal cache semantics, complete report identity or either original native limit.',
        'next_step': 'Run the unchanged rational/quadratic/integer probes in separate time and memory modes, with five preregistered three-variant blocks and dedicated cost targets. Keep every adverse row. Advance only supported variants to native and complete-report qualification.',
        'evidence_sha256': {str(path.relative_to(E)): sha(path) for path in [
            E / 'preparation-v1/protocol.json', E / 'validation-v1/protocol.json',
            E / 'validation-v1/result.json', E / 'validation-v1/commands.json']},
        'source_parent_and_main_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'correctness-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    p = ROOT / 'docs/properties-status.json'
    status = json.loads(p.read_text())
    status['current_work']['isolated_coefficient_scaling'] = {
        'status': assessment['decision'], 'evidence': str((E / 'correctness-v1.json').relative_to(ROOT)),
        'result': 'Right-scalar and bidirectional variants each pass 867 scalar library results, 28 doctests, four included new regressions and six included cache-frontier regressions. The four new tests also pass on the unchanged runtime baseline. Clippy, formatting and warning-denied docs pass.',
        'next_step': assessment['next_step'], 'runtime_admitted': False}
    status['current_work']['pending_native_geometry']['next_step'] = assessment['next_step']
    assert status['current_work']['plan_complete'] is False
    p.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'full_scalar_results_each': 867, 'doctests_each': 28}))


if __name__ == '__main__':
    main()
