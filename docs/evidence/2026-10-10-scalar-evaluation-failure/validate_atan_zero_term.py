"""Run the nine frozen atan regressions separately under the retained limits."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    build = E/'typed-api-prototype-v1516-atan-zero-term-build'
    result = read(build/'result.json')
    assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
    paths = re.findall(r'Executable unittests src/lib\.rs \(([^)]+)\)', (build/'stderr.log').read_text())
    assert len(paths) == 1
    binary = Path(paths[0])
    binary_hash = sha(binary)
    rows = {}
    offset = 1517
    for record, module, prefix in [
        ('atan-numeric-survey-caps-v1506.json', 'atan_numeric_probe_tests', 'atan_numeric_'),
        ('atan-kernel-baseline-caps-v1511.json', 'atan_kernel_tests', 'atan_kernel_'),
    ]:
        baseline = read(E/record)['tests']
        source = Path('crates/hyperreal/src/computable/node')/(module+'.rs')
        assert sha(build/'inputs'/source) == sha(E/'typed-api-prototype-v1513-atan-baseline-build/inputs'/source)
        tests = re.findall(r'    fn ('+prefix+r'\w+)\(\)', (build/'inputs'/source).read_text())
        assert tests == [row['test'] for row in baseline.values()]
        for test, old_phase in zip(tests, baseline):
            phase = f'typed-api-prototype-v{offset}-atan-zero-term-cap'
            offset += 1
            assert not (E/phase).exists()
            selected = 'computable::node::'+module+'::'+test
            subprocess.run([sys.executable, str(E/'validate_candidate_v2.py'), phase, '--exec',
                sys.executable, str(E/'run_limited_unit_test.py'), str(binary), binary_hash,
                selected, '--exact', '--nocapture', '--test-threads=1'], cwd=ROOT, check=True)
            result = read(E/phase/'result.json')
            assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
            assert sha(E/phase/'source-hashes.json.gz') == sha(build/'source-hashes.json.gz')
            log = (E/phase/'stdout.log').read_text()
            cap = json.loads(log[log.rfind('{"command":'):])
            assert cap['exit_code'] == 0 and not cap['wall_timeout']
            assert cap['command'] == [str(binary.resolve()), selected, '--exact', '--nocapture', '--test-threads=1']
            assert cap['helper_sha256'] == sha(E/'run_limited_unit_test.py')
            assert sha(E/phase/'runner.py') == sha(E/'validate_candidate_v2.py')
            assert cap['address_space_bytes'] == 192*1024*1024
            assert cap['cpu_seconds_soft_hard'] == [2, 3] and cap['wall_seconds_limit'] == 4
            assert cap['binary_sha256'] == binary_hash
            observations = lambda p: [line.split(', allocations=')[0]
                for line in (E/p/'stderr.log').read_text().splitlines() if line.startswith('atan ')]
            assert observations(phase) == observations(old_phase)
            rows[phase] = {'test': test, 'baseline_phase': old_phase, 'result': result, 'cap': cap,
                           'observations_except_allocations_identical': True}
            print(phase, 'passed; observations preserved', flush=True)
    assert len(rows) == 9 and offset == 1526
    out = E/'atan-zero-term-caps-v1525.json'
    assert not out.exists()
    out.write_text(json.dumps({'source_record': str((build/'source-hashes.json.gz').relative_to(E)),
        'source_record_sha256': sha(build/'source-hashes.json.gz'), 'tests': rows}, indent=2)+'\n')


if __name__ == '__main__':
    main()
