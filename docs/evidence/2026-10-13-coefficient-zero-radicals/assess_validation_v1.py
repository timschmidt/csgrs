"""Verify zero-radical scalar qualification without runtime admission."""
import gzip
import hashlib
import json
from pathlib import Path
import re
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
    assert result['complete'] and result['variants'] == ['baseline', 'candidate']
    assert sha(E / 'prepare_v1.py') == preparation['runner_sha256']
    assert sha(E / 'coefficient_zero_radical_tests.rs') == preparation['test_input_sha256']
    assert sha(E / 'validate_v1.py') == protocol['runner_sha256'] == preparation['validation_runner_sha256']
    assert sha(Path(__file__)) == protocol['assessor_sha256'] == preparation['validation_assessor_sha256']
    assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
    assert result['phases'] == json.loads((E / 'validation-v1/commands.json').read_text())
    assert [row['name'] for row in result['phases']] == [
        'baseline-format', 'baseline-zero-radical-regressions',
        'candidate-format', 'candidate-zero-radical-regressions', 'candidate-cache-frontiers',
        'candidate-strict-clippy', 'candidate-scalar-library', 'candidate-scalar-doctests', 'candidate-scalar-docs',
    ]
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
    assert tests == json.loads((E / 'validation-v1/tests.json').read_text())
    assert tests == [
        {'variant': 'baseline', 'phase': 'zero-radical-regressions', 'passed': 7, 'failed': 0, 'ignored': 0},
        {'variant': 'candidate', 'phase': 'zero-radical-regressions', 'passed': 7, 'failed': 0, 'ignored': 0},
        {'variant': 'candidate', 'phase': 'cache-frontiers', 'passed': 6, 'failed': 0, 'ignored': 0},
        {'variant': 'candidate', 'phase': 'scalar-library', 'passed': 870, 'failed': 0, 'ignored': 0},
        {'variant': 'candidate', 'phase': 'scalar-doctests', 'passed': 28, 'failed': 0, 'ignored': 0},
    ]
    for test in tests:
        raw = (E / 'validation-v1' / (test['variant'] + '-' + test['phase'] + '.stdout')).read_text()
        assert re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw) == [(str(test['passed']), '0', '0')]
    assessment = {
        'decision': 'isolated-zero-radical-guards-pass-scalar-correctness-await-fixed-costs',
        'assessor_sha256': sha(Path(__file__)), 'tests': tests,
        'implementation': 'Skip only the two borrowed radical linear loops when the opposite rational scale is stored zero. The baseline has the original runtime body and identical seven-test module.',
        'preserved_source': 'Removing the two guard statements and test declaration recovers the original multiplier exactly. Unsupported products, nonzero operations, radical cross-products, opaque ownership and operations, insertion order, recognition budgets, retained normal forms and mandatory original fallback are unchanged.',
        'regression_scope': 'Forty rational-scaling component cases in both directions; nine independently expanded mixed-radical products with cancellation and insertion order; eight/nine-term pure cross-product frontier and earlier linear declines; unsupported opaque cross-products; zero/nonzero opaque-owner release; child warming and cloned pure-radical product facts; scaled opaque cancellation. The same seven functions pass on the original baseline. Six existing cache-frontier functions remain included in the full scalar total.',
        'checks': 'Full release all-feature scalar library and doctests, strict all-target/all-feature scalar Clippy, workspace formatting and warning-denied scalar docs pass.',
        'limits': 'Focused test counts overlap the 870 library results. These checks do not establish a performance or memory improvement, default/all-workflow consumer qualification, universal cache semantics, complete report identity or either original native limit.',
        'next_step': 'Run the unchanged rational/quadratic/integer probes in separate time and memory modes with five fixed baseline/candidate blocks and dedicated targets. Preserve every adverse row; advance only if scalar costs support native qualification.',
        'evidence_sha256': {str(path.relative_to(E)): sha(path) for path in [
            E / 'preparation-v1/protocol.json', E / 'validation-v1/protocol.json',
            E / 'validation-v1/result.json', E / 'validation-v1/commands.json']},
        'source_parent_and_main_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'correctness-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'full_scalar_results': 870, 'doctests': 28}))


if __name__ == '__main__':
    main()
