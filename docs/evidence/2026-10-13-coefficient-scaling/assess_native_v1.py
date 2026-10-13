"""Assess complete native qualification without reinterpreting gate durations."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'native-validation-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    destination = E / 'native-assessment-v1.json'
    assert not destination.exists()
    result = json.loads((OUT / 'result.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    identities = json.loads((OUT / 'binaries.json').read_text())
    assert result['complete'] and result['source_parent_main_and_binaries_unchanged']
    assert sha(E / 'validate_native_v1.py') == protocol['runner_sha256']
    assert sha(E / 'preparation-v1/protocol.json') == protocol['preparation_sha256']
    assert sha(E / 'scalar-decision-v1.json') == protocol['scalar_decision_sha256']
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in preparation['pending_native_sha256'].items())
    original = json.loads(gzip.decompress((E / 'preparation-v1/parent-source-hashes.json.gz').read_bytes()))
    assert all(sha(Path(preparation['parent_source']) / name) == digest for name, digest in original.items())
    for row in preparation['variants'].values():
        assert all(sha(Path(row['source']) / name) == digest
                   for name, digest in (original | row['source_changes_sha256']).items())
    phases = {row['name']: row for row in result['phases']}
    assert len(phases) == 14
    for name, phase in phases.items():
        if phase['phase'] not in ['full-board', 'copper-neck']:
            assert phase['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (name + '.' + stream)) == phase[stream + '_sha256']
    tests, gates = [], []
    for variant in ['right', 'both']:
        identity = identities[variant]
        assert sha(Path(identity['path'])) == identity['sha256']
        assert Path(identity['path']).is_relative_to(Path(protocol['variants'][variant]['target']))
        assert Path(identity['manifest_path']) == Path(protocol['variants'][variant]['source']) / 'crates/hyperdrc/Cargo.toml'
        assert identity['features'] == identities['right']['features']
        for phase in ['curve-tests', 'drc-tests-except-separate-gates']:
            raw = (OUT / (variant + '-' + phase + '.stdout')).read_text()
            matches = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw)
            assert matches and all(int(row[1]) == 0 for row in matches)
            row = {'variant': variant, 'phase': phase, 'passed': sum(int(row[0]) for row in matches),
                   'failed': 0, 'ignored': sum(int(row[2]) for row in matches), 'result_groups': len(matches)}
            assert row in result['tests']
            if phase == 'curve-tests':
                assert row['passed'] == 2494 and row['ignored'] == 33
            tests.append(row)
        for gate in protocol['gates']:
            name = variant + '-' + gate['label']
            raw = (OUT / (name + '.stdout')).read_text() + (OUT / (name + '.stderr')).read_text()
            assert 'skipping complex project fixture test' not in raw
            if gate['label'] == 'full-board':
                assert raw.count('finished emit report') == 2
                assert [int(value) for value in re.findall(r'finished run \((\d+) active finding', raw)] == [12, 197]
                match = re.search(r'KiCad smoke suite took ([0-9.]+)s of worker time', raw)
            else:
                match = re.search(r'complex project copper neck regression fixture took ([0-9.]+)s', raw)
            passed = phases[name]['exit_code'] == 0
            assert ('1 passed' if passed else '1 failed') in raw
            if not passed:
                assert match and float(match[1]) >= gate['limit_seconds']
            gates.append(gate | {'variant': variant, 'passed': passed,
                                 'observed_assertion_seconds': float(match[1]) if match else None,
                                 'process_wall_seconds': phases[name]['seconds'],
                                 'timing_note': protocol['gate_duration_note']})
    assessment = {
        'decision': 'native-qualification-recorded-await-fixed-controls-and-complete-reports',
        'assessor_sha256': sha(Path(__file__)), 'test_counts': tests,
        'strict_consumer_clippy_passed_for_both': True,
        'warning_denied_scalar_curve_drc_docs_passed_for_both': True,
        'formatting': 'Previously passed on the same unchanged source in validation-v1.',
        'native_gates': gates,
        'all_original_gates_passed': all(gate['passed'] for gate in gates),
        'limits': [
            'Full all-feature release curve and DRC suites ran, with the two named DRC limits executed separately. This does not qualify every workspace/default/feature/workflow combination.',
            'The 33 ignored curve results remain ignored, not covered by this run.',
            'The native assertions retain their original metrics and limits. Process wall times include setup and cannot replace assertion measurements.',
            'The original complete boards produced 12 and 197 findings. This test output does not compare complete report bytes.',
            'Standalone native gates do not establish a matched application speedup. Scalar and inherited endpoint timing tradeoffs remain open.',
        ],
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'main_and_all_35_pending_native_paths_unchanged': True,
        'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    ledger = ROOT / 'docs/properties-status.json'
    status = json.loads(ledger.read_text())
    status['current_work']['isolated_coefficient_scaling'].update({
        'status': assessment['decision'], 'evidence': str(destination.relative_to(ROOT)),
        'native_tests': tests, 'native_gates': gates,
        'next_step': 'Complete the preregistered fixed native controls and complete unnormalized report comparison. Preserve all scalar and inherited endpoint tradeoffs and every failed original native assertion; neither variant is admitted.',
    })
    ledger.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({'test_counts': tests, 'gates': gates, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
