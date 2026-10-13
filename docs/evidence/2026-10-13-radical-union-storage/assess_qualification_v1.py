"""Verify the completed three-source qualification and recorded type layouts."""
import hashlib
import json
from pathlib import Path
import re

from qualify_v1 import verify

E = Path(__file__).resolve().parent
OUT = E / 'qualification-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'correctness-v1.json'
    assert not destination.exists()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    result = json.loads((OUT / 'result.json').read_text())
    verify(preparation)
    assert sha(E / 'qualify_v1.py') == protocol['runner_sha256']
    assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
    assert result['complete'] and result['source_parent_and_main_unchanged']
    commands = json.loads((OUT / 'commands.json').read_text())
    assert commands == result['phases'] and len(commands) == 30
    assert [row['name'] for row in commands] == [variant + '-' + label for variant in protocol['variant_order'] for label, _, _ in protocol['commands_per_variant']]
    tests, layouts = [], {}
    for phase in commands:
        assert phase['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (phase['name'] + '.' + stream)) == phase[stream + '_sha256']
        expected = next(row for row in protocol['commands_per_variant'] if row[0] == phase['phase'])
        assert phase['command'] == ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *expected[1]]
        variant = phase['variant']
        assert phase['cwd'] == preparation['variants'][variant]['source']
        assert phase['extra_environment'] == {'CARGO_TARGET_DIR': preparation['variants'][variant]['target']}
        raw = (OUT / (phase['name'] + '.stdout')).read_text()
        if expected[2] is not None:
            groups = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
            assert groups == [(str(n), '0', '0') for n in expected[2]]
            tests.append({'variant': variant, 'phase': phase['phase'], 'passed': sum(expected[2]), 'failed': 0, 'ignored': 0})
        if phase['phase'] == 'storage-regressions':
            values = re.findall(r'TEMPORARY_LAURENT_LAYOUT\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)', raw)
            assert len(values) == 1
            layouts[variant] = dict(zip(['Rational', 'RadicalCoefficient', 'PiLaurentPolynomial', 'ClosedRadicalCoefficient', 'ClosedPiLaurentPolynomial', 'Computable', 'Node'], map(int, values[0])))
    assert tests == result['tests'] == json.loads((OUT / 'tests.json').read_text())
    assert layouts == result['layouts'] == json.loads((OUT / 'layouts.json').read_text())
    features = json.loads((OUT / 'smallvec-features.json').read_text())
    assert features == result['smallvec_features']
    for variant, entries in features.items():
        assert entries
        for entry in entries:
            path = Path(entry['path'])
            assert path.is_relative_to(Path(preparation['variants'][variant]['target']))
            assert sha(path) == entry['sha256']
            assert json.loads(json.loads(path.read_text())['features']) == entry['features'] == preparation['variants'][variant]['smallvec_scalar_features']
    assessment = {
        'status': 'three-variants-pass-scoped-scalar-qualification-costs-next',
        'assessor_sha256': sha(Path(__file__)), 'tests': tests, 'layouts': layouts,
        'smallvec_features': {v: row['smallvec_scalar_features'] for v, row in preparation['variants'].items()},
        'same_coefficient_size_as_baseline': layouts['radicals']['RadicalCoefficient'] == layouts['baseline']['RadicalCoefficient'], 'locked_package_inventory_unchanged': True,
        'source_parent_and_main_unchanged': True, 'all_30_commands_passed': True,
        'qualification': 'Each variant passes eight shared storage controls, six inherited cache-frontier tests, eleven admitted P4 tests, 882 all-feature scalar library tests, 795 default scalar library tests and 28 doctests. Formatting, strict default/all-feature/all-target Clippy and warning-denied scalar docs pass. Counts overlap; these are not unique or full-workflow totals.',
        'remaining': 'Fixed scalar time/allocation/size controls, native qualification and original full-fixture gates, endpoint/broad/abort controls, complete report bytes and production CLI size. Larger temporary layouts are not total-memory savings; no runtime is admitted.',
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'qualified_variants': list(layouts), 'layouts': layouts, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
