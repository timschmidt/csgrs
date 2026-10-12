"""Check complete input capture and command-time additions without SOURCE edits."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())


def main():
    wrapper = E / 'validate_candidate_v3.py'
    tests = E / 'test_candidate_snapshots_v2429.py'
    preparation = E / 'candidate-snapshot-preparation-v2428.json'
    protocol = E / 'candidate-snapshot-validation-protocol-v2430.json'
    output = E / 'candidate-snapshot-validation-v2430.json'
    phase = E / 'typed-api-prototype-v2430-source-catalog-units'
    assert all(not p.exists() for p in [protocol, output, phase])
    assert read(preparation)['successor_sha256'] == sha(wrapper)
    command = [sys.executable, str(wrapper), phase.name, '--exec', sys.executable, str(tests), '-v']
    protocol.write_text(json.dumps({
        'status': 'Frozen before the complete-catalog unit tests and wrapper run',
        'runner_sha256': sha(Path(__file__)), 'wrapper_sha256': sha(wrapper), 'tests_sha256': sha(tests),
        'preparation_sha256': sha(preparation), 'command': command, 'expected_tests': 13,
        'scope': 'Three tests execute the actual wrapper with real child commands against temporary source trees, including added extensionless inputs and deleted original inputs. Other tests cover input types, source edits/removals, symlinks and generated-directory pruning. The outer wrapper must preserve the retained SOURCE/main files and reproduce the v2386 source snapshot exactly.',
    }, indent=2) + '\n')
    result = subprocess.run(command, cwd=ROOT, env=os.environ | {'PYTHONDONTWRITEBYTECODE': '1'}, capture_output=True, text=True)
    (E / 'candidate-snapshot-wrapper-v2430.stdout.log').write_text(result.stdout)
    (E / 'candidate-snapshot-wrapper-v2430.stderr.log').write_text(result.stderr)
    observed = read(phase / 'result.json')
    stderr = (phase / 'stderr.log').read_text()
    expected_source = '67e3d14b5d7a319778d56b701498e97225c16134eca65bb14a7586c727f2fb5a'
    data = {
        'protocol_sha256': sha(protocol), 'wrapper_exit_code': result.returncode, 'phase_result': observed,
        'phase_result_sha256': sha(phase / 'result.json'), 'source_record_sha256': sha(phase / 'source-hashes.json.gz'),
        'thirteen_tests_passed': result.returncode == 0 and observed['exit_code'] == 0
            and 'Ran 13 tests in ' in stderr and stderr.endswith('\nOK\n'),
        'retained_source_snapshot_identical': sha(phase / 'source-hashes.json.gz') == expected_source,
        'source_changed': False, 'retained_coverage_changed': False,
    }
    output.write_text(json.dumps(data, indent=2) + '\n')
    assert data['thirteen_tests_passed'] and data['retained_source_snapshot_identical']
    assert observed['source_unchanged'] and observed['main_unchanged'] and observed['source_catalog_rechecked']
    assert observed['source_snapshot_error'] is None
    assert sha(wrapper) == read(protocol)['wrapper_sha256'] and sha(tests) == read(protocol)['tests_sha256']
    print(json.dumps(data))


if __name__ == '__main__':
    main()
