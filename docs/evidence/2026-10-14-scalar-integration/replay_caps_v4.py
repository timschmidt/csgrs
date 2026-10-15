"""Replay all 39 historical unit-test cap schedules on the integrated binary."""
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('extended_v4', E / 'validate_extended_v4.py')
x = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(x)
p, q = x.p, x.q
OUT = E / 'unit-caps-v4'


def main():
    assert not OUT.exists()
    extended = json.loads((E / 'extended-v4/result.json').read_text())
    assert extended['complete'] and extended['commands_executed'] == extended['commands_expected'] == 5
    old = p.ROOT / 'docs/evidence/2026-10-10-scalar-evaluation-failure'
    reference = old / 'eager-bound-decoder-retention-v2335.json'
    retained = json.loads(reference.read_text())
    schedules = retained['fresh_capped_validations']
    assert len(schedules) == 39
    helper = old / 'run_limited_unit_test.py'
    assert p.sha(helper) == '15aeb2646f69af11385ebf272832cb3f6c573f66667f09843fc65e8598876617'
    matches = re.findall(r'Running unittests src/lib.rs \((/[^\n]+)\)',
                         (E / 'qualification-v4/all-feature-tests.stderr').read_text())
    assert len(matches) == 1
    binary = Path(matches[0])
    binary_hash = p.sha(binary)
    source = p.read_map(E / 'freeze-v4/source-hashes.json.gz')
    main_sources = p.read_map(p.MAIN_RECORD)
    protocol_inputs = {str(path.relative_to(p.ROOT)): p.sha(path)
                       for path in [reference, helper, Path(__file__), E / 'extended-v4/result.json']}

    def guard():
        assert x.source_inventory() == source
        assert all(p.sha(p.ROOT / name) == digest for name, digest in main_sources.items())
        assert all(p.sha(p.ROOT / name) == digest for name, digest in protocol_inputs.items())
        assert p.sha(binary) == binary_hash

    guard()
    OUT.mkdir()
    commands = []
    for name, row in schedules.items():
        cap = row['cap']
        assert cap['address_space_bytes'] == 201326592 and cap['cpu_seconds_soft_hard'] == [2, 3]
        assert cap['wall_seconds_limit'] == 4 and cap['helper_sha256'] == p.sha(helper)
        commands.append({'reference_phase': name, 'arguments': cap['command'][1:],
                         'reference_test_counts': row['test_counts']})
    p.dump(OUT / 'protocol.json', {
        'commands': commands, 'inputs_sha256': protocol_inputs, 'binary': str(binary), 'binary_sha256': binary_hash,
        'environment': q.ENV, 'cwd': str(q.SOURCE),
        'address_space_bytes': 201326592, 'cpu_seconds_soft_hard': [2, 3], 'wall_seconds_limit': 4,
        'scope': 'All 39 unchanged historical unit-test schedules, with the same helper and process caps. The separate algebraic cost-process SIGXCPU failures, numerical/public replay matrix, native geometry and full-board gates remain open.',
        'stop_on_failure': True, 'no_retry_resume_filter_or_pool': True,
    })
    rows = []
    for index, specification in enumerate(commands):
        guard()
        command = [sys.executable, str(helper), str(binary), binary_hash, *specification['arguments']]
        stdout_path, stderr_path = OUT / f'{index:02}.stdout', OUT / f'{index:02}.stderr'
        with stdout_path.open('wb') as stdout, stderr_path.open('wb') as stderr:
            result = subprocess.run(command, cwd=q.SOURCE, env=os.environ | q.ENV, stdout=stdout, stderr=stderr)
        guard()
        output = stdout_path.read_text()
        cap = json.loads(output.rstrip().splitlines()[-1])
        assert cap['command'] == [str(binary), *specification['arguments']]
        assert cap['binary_sha256'] == binary_hash and cap['helper_sha256'] == p.sha(helper)
        batches = [dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'], map(int, match)))
                   for match in re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out', output)]
        # The final batch belongs to the top-level command. Earlier batches
        # are child-process tests, including the existing cache-poison fixture.
        counts = {'top_level_passed': batches[-1]['passed'] if batches else 0,
                  'child_process_passed': sum(row['passed'] for row in batches[:-1])}
        counts_match = counts == specification['reference_test_counts']
        passed = result.returncode == cap['exit_code'] == 0 and not cap['wall_timeout'] and counts_match
        row = {'index': index, 'reference_phase': specification['reference_phase'], 'cap': cap,
               'helper_process_exit_code': result.returncode, 'test_counts': counts,
               'test_counts_match_reference': counts_match, 'passed': passed,
               'stdout_sha256': p.sha(stdout_path), 'stderr_sha256': p.sha(stderr_path)}
        rows.append(row)
        p.dump(OUT / 'commands.json', rows)
        p.dump(OUT / 'result.json', {'complete': len(rows) == 39 and all(row['passed'] for row in rows),
                                   'commands_executed': len(rows), 'commands_expected': 39,
                                   'top_level_tests_passed': sum(row['test_counts']['top_level_passed'] for row in rows),
                                   'child_tests_passed': sum(row['test_counts']['child_process_passed'] for row in rows),
                                   'runtime_admitted': False})
        print(json.dumps({'index': index, 'passed': passed, 'exit_code': cap['exit_code']}), flush=True)
        if not passed:
            raise SystemExit(result.returncode if result.returncode > 0 else 1)


if __name__ == '__main__':
    main()
