"""Assess the combined native source, retaining every original gate outcome."""
import hashlib
import json
from pathlib import Path
import re

from qualify_v2 import counts, verify

E = Path(__file__).resolve().parent
OUT = E / 'qualification-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'qualification-assessment-v2.json'
    assert not destination.exists()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    result = json.loads((OUT / 'result.json').read_text())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    verify(preparation, protocol, binaries)
    assert result['complete'] and result['source_parent_main_and_binaries_unchanged']
    phases = result['phases']
    assert phases == json.loads((OUT / 'commands.json').read_text())
    assert [row['name'] for row in phases] == [row[0] for row in protocol['commands']] + [row['label'] for row in protocol['gates']]
    assert len(phases) == 15
    for row in phases:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    tests = []
    for (name, command, expected), phase in zip(protocol['commands'], phases):
        assert phase['exit_code'] == 0
        if expected is not None:
            actual = counts((OUT / (name + '.stdout')).read_text())
            assert [actual['passed'], actual['ignored'], actual['groups']] == expected
            tests.append({'name': name} | actual)
    assert tests == result['tests'] == json.loads((OUT / 'tests.json').read_text())
    assert [(row['label'], row['limit_seconds'], row['metric']) for row in protocol['gates']] == [
        ('full-board', 120, 'worker CPU'), ('copper-neck', 60, 'check body wall time')]
    gates = []
    for gate, phase in zip(protocol['gates'], phases[-2:]):
        raw = (OUT / (gate['label'] + '.stdout')).read_text() + (OUT / (gate['label'] + '.stderr')).read_text()
        assert 'skipping complex project fixture test' not in raw
        if gate['label'] == 'full-board':
            assert raw.count('hyperdrc: finished emit report') == 2
            assert [int(value) for value in re.findall(r'finished run \((\d+) active finding', raw)] == [12, 197]
            match = re.search(r'KiCad smoke suite took ([0-9.]+)s of worker time', raw)
        else:
            match = re.search(r'complex project copper neck regression fixture took ([0-9.]+)s', raw)
        passed = phase['exit_code'] == 0
        assert ('1 passed' if passed else '1 failed') in raw
        if not passed:
            assert phase['exit_code'] == 101 and match and float(match[1]) >= gate['limit_seconds']
        gates.append(gate | {'passed': passed, 'observed_assertion_seconds': float(match[1]) if match else None,
                             'process_wall_seconds': phase['seconds'], 'timing_note': protocol['gate_duration_note']})
    result = {
        'decision': 'combined-native-qualification-complete-review-original-gates-and-open-costs',
        'assessor_sha256': sha(Path(__file__)), 'test_counts': tests,
        'format_strict_stack_clippy_and_docs_passed': True, 'native_gates': gates,
        'all_original_gates_passed': all(row['passed'] for row in gates),
        'source_parent_main_and_binaries_unchanged': True,
        'limits': [
            'The focused test totals overlap the full suites. The 33 ignored curve results remain ignored.',
            'This covers the listed all-feature release configurations, not the full workflow/feature/target matrix.',
            'Process wall measurements include setup and never replace the original native assertion metrics.',
            'The complete boards retain 12 and 197 findings; these counts do not establish complete report identity.',
            'This combined qualification does not settle inherited endpoint cost tradeoffs or establish an application speedup.',
            'All pending native work and all properties, targets and crate-specific future work remain open.',
        ],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'reused_successful_phases': 8, 'newly_executed_phases': 7,
        'native_runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'test_counts': tests, 'gates': gates, 'native_runtime_admitted': False}))


if __name__ == '__main__':
    main()
