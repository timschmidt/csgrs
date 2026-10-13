"""Audit the isolated P4 repair's focused, scalar, solver and probe evidence."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'qualification-v5'
Q = ROOT / 'docs/evidence/2026-10-13-quadratic-tower-sign-transfer'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    destination = E / 'correctness-v1.json'
    assert not destination.exists()
    protocol = json.loads((OUT / 'protocol.json').read_text())
    result = json.loads((OUT / 'result.json').read_text())
    preparation = json.loads((E / 'preparation-v4/protocol.json').read_text())
    baseline = json.loads((E / 'baseline-v1/protocol.json').read_text())
    assert result['complete'] and result['fresh_probe_processes'] == 12
    assert sha(E / 'qualify_v5.py') == protocol['runner_sha256'] == preparation['qualification_runner_sha256']
    assert sha(E / 'preparation-v4/protocol.json') == protocol['preparation_protocol_sha256']
    assert sha(E / 'quadratic_tower_sign_retention_tests_v4.rs') == preparation['test_sha256']
    assert sha(Q / 'probe_v2.rs') == protocol['probe_sha256']
    assert sha(Q / 'observation-v2/observations.json') == protocol['original_observations_sha256']
    assert sources() == baseline['main_source_sha256']
    for directory, manifest in [
        (ROOT, 'baseline-v1/original-source-hashes.json.gz'),
        (Path(baseline['source']), 'baseline-v1/baseline-source-hashes.json.gz'),
        (Path(preparation['source']), 'preparation-v4/candidate-source-hashes.json.gz'),
        (Path(preparation['control_source']), 'preparation-v4/control-source-hashes.json.gz'),
    ]:
        entries = json.loads(gzip.decompress((E / manifest).read_bytes()))
        assert all(sha(directory / name) == digest for name, digest in entries.items())
    assert all(sha(ROOT / name) == digest for name, digest in baseline['pending_native_sha256'].items())
    commands = json.loads((OUT / 'commands.json').read_text())
    names = ['fmt', 'build-baseline-controls', 'baseline-controls', 'build-tests', 'focused-regressions',
             'all-feature-tests', 'default-tests', 'doctests', 'default-clippy', 'all-feature-clippy', 'docs',
             'solver-tests', 'solver-clippy', 'build-library', 'link-probe']
    probe_names = [f'probe-{index:02d}-{case["family"]}-{case["shape"]}-{case["scale"]}'
                   for index, case in enumerate(protocol['cases'], 1)]
    assert [row['name'] for row in commands] == names + probe_names
    assert len(commands) == result['qualification_commands'] == 27
    totals = {}
    for row in commands:
        assert row['exit_code'] == row['expected_exit_code'] == (101 if row['name'] == 'baseline-controls' else 0)
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
        raw = (OUT / (row['name'] + '.stdout')).read_text()
        assert re.findall(r'^test result: .+$', raw, re.MULTILINE) == row['test_summaries']
        if row['test_summaries']:
            counts = [tuple(map(int, re.search(r'(\d+) passed; (\d+) failed; (\d+) ignored;', line).groups()))
                      for line in row['test_summaries']]
            totals[row['name']] = {'passed': sum(x[0] for x in counts), 'failed': sum(x[1] for x in counts),
                                   'ignored': sum(x[2] for x in counts), 'groups': len(counts)}
    assert totals['baseline-controls'] == {'passed': 5, 'failed': 6, 'ignored': 0, 'groups': 1}
    for name, expected in [('focused-regressions', 11), ('all-feature-tests', 983), ('default-tests', 878), ('doctests', 28), ('solver-tests', 974)]:
        assert totals[name]['passed'] == expected and totals[name]['failed'] == totals[name]['ignored'] == 0
    bindings = json.loads((OUT / 'bindings.json').read_text())
    for name, row in bindings.items():
        assert sha(Path(row['path'])) == row['sha256'] and Path(row['path']).stat().st_size == row['bytes']
        if name == 'probe':
            continue
        variant_source = preparation['control_source'] if name == 'baseline-control-tests' else preparation['source']
        variant_target = preparation['control_target'] if name == 'baseline-control-tests' else preparation['target']
        assert Path(row['manifest_path']) == Path(variant_source) / 'crates/hyperreal/Cargo.toml'
        assert Path(row['path']).is_relative_to(Path(variant_target))
        assert row['features'] == bindings['library']['features'] and row['target']['kind'] == ['lib']
        assert row['profile']['test'] == (name != 'library')
    observations = json.loads((OUT / 'observations.json').read_text())
    originals = [row['row'] for row in json.loads((Q / 'observation-v2/observations.json').read_text()) if row['variant'] == 'main']
    assert len(observations) == len(originals) == 12
    for observed, original, case, name in zip(observations, originals, protocol['cases'], probe_names, strict=True):
        assert observed['name'] == name and observed['case'] == case
        row = observed['row']
        assert json.loads((OUT / (name + '.stdout')).read_text()) == row
        assert {key: row[key] for key in case} == case
        assert row['compact_covered'] and row['repeated_covered'] and row['all_definite_signs_agree']
        assert row['difference_tower_sign'] == 'zero'
        expected = 'negative' if case['scale'] < 0 else 'positive'
        for field in ['compact_immediate', 'compact_zero', 'compact_512', 'clone_zero', 'repeated_zero']:
            assert row[field] == expected
        for field in ['delta_before', 'delta_warmed', 'original_immediate', 'original_zero', 'original_clone_zero', 'original_after']:
            assert row[field] == original[field]
    assessment = {
        'decision': 'isolated-tower-sign-retention-passes-scoped-correctness-await-fixed-costs',
        'assessor_sha256': sha(Path(__file__)), 'test_results': totals,
        'source': preparation['source'], 'runtime_matches_preparation_v3': preparation['runtime_matches_preparation_v3'],
        'probe_result': 'All twelve formerly weakened immediate facts are retained. All eight formerly lost precision-0/-512 signs remain available. Clone/repeated facts agree, all exact differences are zero, and original facts match the frozen main observations.',
        'proof': preparation['proof'],
        'preserved_failures': ['The original output-format compile failure.', 'Two candidate preparation format-identity failures.',
                               'The ambiguous same-name Cargo artifact selection.', 'The original denested normalization-control assumption.',
                               'All six original sign-retention regressions, repeated on the unchanged eleven-test baseline.'],
        'limits': ['Configurations and focused tests overlap; their totals are not additive unique coverage.',
                   'This covers scalar default/all-feature tests and the direct hypersolve consumer, not every repository workflow or native application gate.',
                   'No universal cache/concurrency/error-bound proof, performance result, whole-plan completion or runtime admission is claimed.'],
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'sources_main_and_bound_artifacts_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'tests': totals, 'fresh_probe_cases': 12}))


if __name__ == '__main__':
    main()
