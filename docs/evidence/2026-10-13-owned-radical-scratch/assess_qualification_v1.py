"""Verify the completed two-source qualification and recorded type layouts."""
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
    assert commands == result['phases'] and len(commands) == 26
    assert [row['name'] for row in commands] == [variant + '-' + label for variant in protocol['variant_order'] for label, _, _ in protocol['commands_per_variant']]
    tests, layouts, memo_inventories, sharing = [], {}, {}, {}
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
        counts = expected[2][variant] if isinstance(expected[2], dict) else expected[2]
        if counts is not None:
            groups = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
            assert groups == [(str(n), '0', '0') for n in counts]
            tests.append({'variant': variant, 'phase': phase['phase'], 'passed': sum(counts), 'failed': 0, 'ignored': 0})
        if phase['phase'] == 'storage-regressions':
            values = re.findall(r'TEMPORARY_LAURENT_LAYOUT\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t(\d+)', raw)
            assert len(values) == 1
            layouts[variant] = dict(zip(['Rational', 'RadicalCoefficient', 'PiLaurentPolynomial', 'ClosedRadicalCoefficient', 'ClosedPiLaurentPolynomial', 'Computable', 'Node'], map(int, values[0])))
        if phase['phase'] == 'memo-regressions':
            values = re.findall(r'CLOSED_MEMO_INVENTORY\t(retained|original|opaque)\t(\d+)', raw)
            assert len(values) == 3
            actual = {name: int(count) for name, count in values}
            assert actual == {'retained': 1, 'original': 1, 'opaque': 2}
            memo_inventories[variant] = actual
        if phase['phase'] == 'alias-regressions':
            values = re.findall(r'RADICAL_STORAGE_SHARING\t([01])\t([01])', raw)
            assert values == [('0', '0' if variant == 'baseline' else '1')]
            sharing[variant] = dict(zip(['source_shares_closed', 'views_share_closed'], map(int, values[0])))
    assert sharing == result['sharing'] == json.loads((OUT / 'sharing.json').read_text())
    assert tests == result['tests'] == json.loads((OUT / 'tests.json').read_text())
    assert layouts == result['layouts'] == json.loads((OUT / 'layouts.json').read_text())
    assert memo_inventories == result['memo_inventories'] == json.loads((OUT / 'memo-inventories.json').read_text())
    assert layouts['baseline']['RadicalCoefficient'] == 56
    assert layouts['baseline']['ClosedRadicalCoefficient'] == 32 and layouts['candidate']['ClosedRadicalCoefficient'] == 16
    for name in ['Rational', 'PiLaurentPolynomial', 'ClosedPiLaurentPolynomial', 'Computable', 'Node']:
        assert layouts['baseline'][name] == layouts['candidate'][name]
    assessment = {
        'status': 'two-variants-pass-scoped-scalar-qualification-costs-next',
        'assessor_sha256': sha(Path(__file__)), 'tests': tests, 'layouts': layouts,
        'memo_inventories': memo_inventories, 'sharing': sharing, 'coefficient_layouts_recorded': True,
        'manifests_and_lockfiles_unchanged': True,
        'source_parent_and_main_unchanged': True, 'all_26_commands_passed': True,
        'qualification': 'Each variant passes three shared alias-isolation controls, three shared publication/memo controls, eight shared storage controls, six inherited cache-frontier tests, eleven admitted P4 tests and 28 doctests. The baseline passes 888 all-feature and 801 default scalar library tests; the candidate passes 890 and 803, including two additional buffer-ownership controls. Formatting, strict default/all-feature/all-target Clippy and warning-denied scalar docs pass. Counts overlap; these are not unique or full-workflow totals.',
        'remaining': 'Fixed scalar time/allocation/size controls, native qualification and original full-fixture gates, endpoint/broad/abort controls, complete report bytes and production CLI size. Private coefficient layouts are not total-memory or performance evidence; no runtime is admitted.',
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'qualified_variants': list(layouts), 'layouts': layouts, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
