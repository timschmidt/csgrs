"""Validate the applied dependency checker, acceptance gate and unchanged Rust formatting."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())


def main():
    protocol = E / 'dependency-binding-validation-protocol-v2434.json'
    output = E / 'dependency-binding-validation-v2434.json'
    assert not protocol.exists() and not output.exists()
    proof = E / 'typed-api-prototype-v2433-dependency-binding-verus'
    expected_source = '650385802de02964ad26d0f60ea407294d451b2a538f1de86f94074c1821e236'
    assert read(proof / 'result.json')['exit_code'] == 0
    assert sha(proof / 'source-hashes.json.gz') == expected_source
    evidence = read(proof / 'proof-reports/evidence.json')
    assert evidence['verification_results']['verified'] == 736 and evidence['verification_results']['errors'] == 0
    assert len(evidence['scope']['verified_executable_contracts']) == 98
    assert len(evidence['scope']['verified_model_theorems']) == 178
    acceptance = 'typed-api-prototype-v2435-dependency-binding-acceptance'
    runs = [
        {'phase': 'typed-api-prototype-v2434-dependency-identity-tests',
         'command': [sys.executable, 'crates/hyperreal/verification/test_dependency_identity.py', '-v']},
        {'phase': acceptance,
         'command': [sys.executable, str(E / 'run_candidate_proof_checks.py'), acceptance, 'test_acceptance.py']},
        {'phase': 'typed-api-prototype-v2436-dependency-binding-workspace-fmt',
         'command': read(E / 'typed-api-prototype-v2383-word-scan-workspace-fmt/command.json')['command']},
        {'phase': 'typed-api-prototype-v2437-dependency-binding-included-fmt',
         'command': read(E / 'typed-api-prototype-v2384-word-scan-included-fmt/command.json')['command']},
    ]
    assert all(not (E / row['phase']).exists() for row in runs)
    protocol.write_text(json.dumps({
        'status': 'Frozen before the applied-checker, acceptance and format commands',
        'runner_sha256': sha(Path(__file__)), 'wrapper_sha256': sha(E / 'validate_candidate_v3.py'),
        'source_record_sha256': expected_source, 'proof_result_sha256': sha(proof / 'result.json'),
        'runs': runs, 'dependency_identity_tests': 19, 'acceptance_tests': 10, 'included_format_files': 112,
        'scope': 'No Rust implementation or proof body changes. The dependency checker must operate from its final SOURCE path. Prior negative proofs and runtime evidence require an exact unchanged-input/tool/dependency audit rather than being relabelled as fresh runs.',
    }, indent=2) + '\n')
    results = {}
    for row in runs:
        phase = E / row['phase']
        subprocess.run([sys.executable, str(E / 'validate_candidate_v3.py'), phase.name, '--exec', *row['command']], cwd=ROOT, check=True)
        result = read(phase / 'result.json')
        results[phase.name] = row | {'result': result, 'source_record_sha256': sha(phase / 'source-hashes.json.gz')}
        (output).write_text(json.dumps({'protocol_sha256': sha(protocol), 'source_record_sha256': expected_source,
            'phases': results, 'complete': len(results) == len(runs) and result['exit_code'] == 0}, indent=2) + '\n')
        assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
        assert result['source_catalog_rechecked'] and result['source_snapshot_error'] is None
        assert sha(phase / 'source-hashes.json.gz') == expected_source
        if 'identity-tests' in phase.name:
            text = (phase / 'stderr.log').read_text()
            assert 'Ran 19 tests in ' in text and text.endswith('\nOK\n')
        if phase.name == acceptance:
            text = (phase / 'stderr.log').read_text()
            assert 'Ran 10 tests in ' in text and '\nOK\n' in text
        print(json.dumps({'phase': phase.name, 'passed': True}), flush=True)


if __name__ == '__main__':
    main()
