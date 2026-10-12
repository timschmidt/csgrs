"""Preserve the stale-count reader failure and finish the two unrun format phases."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())


def test_count(path, class_name):
    tree = ast.parse(path.read_text())
    classes = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name]
    assert len(classes) == 1
    return sum(isinstance(n, ast.FunctionDef) and n.name.startswith('test_') for n in classes[0].body)


def main():
    prior_protocol = E / 'dependency-binding-validation-protocol-v2434.json'
    prior_result = E / 'dependency-binding-validation-v2434.json'
    protocol = E / 'dependency-binding-validation-correction-v2438.json'
    output = E / 'dependency-binding-validation-v2438.json'
    assert not protocol.exists() and not output.exists()
    old_protocol, old_result = read(prior_protocol), read(prior_result)
    assert not old_result['complete'] and len(old_result['phases']) == 2
    expected_source = '650385802de02964ad26d0f60ea407294d451b2a538f1de86f94074c1821e236'
    tests = SOURCE / 'crates/hyperreal/verification'
    identity_count = test_count(tests / 'test_dependency_identity.py', 'DependencyIdentityTests')
    acceptance_count = test_count(tests / 'test_acceptance.py', 'AcceptanceTests')
    assert (identity_count, acceptance_count) == (19, 7)
    assert 'from test_dependency_identity import DependencyIdentityTests' in (tests / 'test_acceptance.py').read_text()
    results = old_result['phases'].copy()
    for index, row in enumerate(old_protocol['runs'][:2]):
        phase = E / row['phase']
        observed = read(phase / 'result.json')
        assert observed == results[phase.name]['result']
        assert observed['exit_code'] == 0 and observed['source_unchanged'] and observed['main_unchanged']
        assert sha(phase / 'source-hashes.json.gz') == expected_source
        count = identity_count if index == 0 else identity_count + acceptance_count
        text = (phase / 'stderr.log').read_text()
        assert f'Ran {count} tests in ' in text and text.endswith('\nOK\n')
    runs = old_protocol['runs'][2:]
    assert len(runs) == 2 and all(not (E / row['phase']).exists() for row in runs)
    protocol.write_text(json.dumps({
        'status': 'Frozen before completing the previously unrun format commands',
        'runner_sha256': sha(Path(__file__)), 'prior_protocol_sha256': sha(prior_protocol),
        'failed_reader_result_sha256': sha(prior_result), 'failed_reader_sha256': sha(E / 'validate_dependency_binding_v2434.py'),
        'source_record_sha256': expected_source, 'dependency_identity_tests': identity_count,
        'acceptance_only_tests': acceptance_count, 'acceptance_entrypoint_tests': identity_count + acceptance_count,
        'completed_runs_reused': list(results), 'remaining_runs': runs,
        'reason': 'The acceptance entry point imports DependencyIdentityTests. It ran nineteen identity plus seven acceptance tests successfully; the old expected aggregate of ten caused only the evidence reader assertion to fail. Preserve both successful child executions and the failed reader record. Do not rerun already passing tests or alter SOURCE.',
    }, indent=2) + '\n')
    for row in runs:
        phase = E / row['phase']
        subprocess.run([sys.executable, str(E / 'validate_candidate_v3.py'), phase.name, '--exec', *row['command']], cwd=ROOT, check=True)
        result = read(phase / 'result.json')
        results[phase.name] = row | {'result': result, 'source_record_sha256': sha(phase / 'source-hashes.json.gz')}
        output.write_text(json.dumps({'protocol_sha256': sha(protocol), 'source_record_sha256': expected_source,
            'phases': results, 'complete': len(results) == 4 and result['exit_code'] == 0,
            'standalone_dependency_tests': identity_count, 'acceptance_entrypoint_tests': identity_count + acceptance_count,
            'distinct_unit_tests': identity_count + acceptance_count,
            'scope': 'The nineteen dependency tests ran both directly and through the acceptance entry point; this is twenty-six distinct tests, not forty-five.'}, indent=2) + '\n')
        assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
        assert result['source_catalog_rechecked'] and result['source_snapshot_error'] is None
        assert sha(phase / 'source-hashes.json.gz') == expected_source
        print(json.dumps({'phase': phase.name, 'passed': True}), flush=True)


if __name__ == '__main__':
    main()
