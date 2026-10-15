"""Seal completed sanitizer, dependency-proof and historical cap results."""
import importlib.util
import json
from pathlib import Path
import re
import shutil
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('extended_v4', E / 'validate_extended_v4.py')
x = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(x)
p, q = x.p, x.q


def main():
    assert not (E / 'extended-closure-v4.json').exists()
    source = p.read_map(E / 'freeze-v4/source-hashes.json.gz')
    assert x.source_inventory() == source
    extended = E / 'extended-v4'
    caps = E / 'unit-caps-v4'
    assert json.loads((extended / 'result.json').read_text())['complete']
    cap_result = json.loads((caps / 'result.json').read_text())
    assert cap_result['complete'] and cap_result['commands_executed'] == 39
    assert (cap_result['top_level_tests_passed'], cap_result['child_tests_passed']) == (156, 1)
    commands = json.loads((extended / 'commands.json').read_text())
    assert len(commands) == 5 and all(row['exit_code'] == 0 and not row['sanitizer_error_markers'] for row in commands)
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert p.sha(extended / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    binaries = json.loads((extended / 'binaries.json').read_text())
    assert all(p.sha(Path(name)) == digest for name, digest in binaries.items())
    evidence = json.loads((extended / 'proof-reports/evidence.json').read_text())
    proof = evidence['verification_results']
    assert proof['verified'] == 737 and proof['errors'] == 0 and proof['success']
    scope = evidence['scope']
    assert (len(scope['verified_executable_contracts']), len(scope['verified_model_theorems'])) == (99, 178)
    assert len(scope['obligations']) == 13 and all(row['status'] != 'complete' for row in scope['obligations'])
    dependency = evidence['dependencies']['verified_metadata']
    configuration = Path(dependency['config'])
    assert p.sha(configuration) == dependency['config_sha256']
    config = json.loads(configuration.read_text())
    result = json.loads(Path(config['result']).read_text())
    assert p.sha(Path(config['result'])) == dependency['result_sha256']
    assert result['gate_passed'] and result['exit_code'] == 0 and result['warnings'] == 0 and result['verified'] == 453
    assert p.sha(Path(dependency['export'])) == dependency['export_sha256']
    assert all(p.sha(Path(name)) == digest for name, digest in dependency['library_artifacts'].items())
    archive = E / 'dependency-proof-v4'
    archive.mkdir()
    for path in [configuration, *[Path(config[name]) for name in ['result', 'stdout', 'stderr', 'invocation']]]:
        shutil.copy2(path, archive / path.name)
    # Concurrent libtest output spliced a parent "ok" into this child summary.
    # Preserve the raw bytes and record the parser limitation separately.
    default_output = (extended / 'asan-default-tests.stdout').read_text()
    interleaved = 'test result: okok\n. 1 passed; 0 failed; 0 ignored; 0 measured; 954 filtered out;'
    assert default_output.count(interleaved) == 1
    all_output = (extended / 'asan-all-feature-tests.stdout').read_text()
    assert len(re.findall(r'test result: ok\. 1 passed; 0 failed; 0 ignored; 0 measured; 1087 filtered out;', all_output)) == 1
    counts = {}
    for name in ['asan-default-tests', 'asan-all-feature-tests']:
        row = next(row for row in commands if row['name'] == name)
        counts[name] = {'top_level_passed': sum(b['passed'] for b in row['test_batches'] if b['filtered'] == 0),
                        'child_process_passed': 1}
    assert counts['asan-default-tests']['top_level_passed'] == 1052
    assert counts['asan-all-feature-tests']['top_level_passed'] == 1203
    p.dump(E / 'extended-closure-v4.json', {
        'extended_driver_terminal_exit_code_observed': 0, 'cap_driver_terminal_exit_code_observed': 0,
        'complete': True, 'runner_sha256': p.sha(Path(__file__)),
        'source_record_sha256': p.sha(E / 'freeze-v4/source-hashes.json.gz'),
        'extended_artifacts_sha256': p.inventory(extended), 'cap_artifacts_sha256': p.inventory(caps),
        'dependency_proof_artifacts_sha256': p.inventory(archive),
        'sanitizer_tests': counts, 'sanitizer_error_markers': [],
        'default_child_summary': {'raw_interleaved_summary': interleaved,
                                  'per_command_regex_missed_this_child_summary': True},
        'conditional_scalar_proof': {'verified_units': 737, 'production_contracts': 99, 'model_theorems': 178,
                                     'open_obligation_groups': 13},
        'actual_dependency_proof': {'verified_units': 453, 'warnings': 0,
                                    'units_include_shared_models_and_are_not_additive_unique_scalar_coverage': True},
        'historical_unit_caps': cap_result,
        'limits': [
            'ASan instruments Rust and the rebuilt standard library; native GMP/MPFR is not claimed instrumented.',
            'The formal target remains conditional. Constructors, caller premises, full approximation semantics and concurrent publication remain open.',
            'Proof dependency binding covers the configured workspace and proof manifest; standalone/package release binding remains a separate obligation.',
            'The 39 unit schedules do not discharge the separate algebraic cost-process SIGXCPU failures or numerical/public replay, matched cost, memory, size and native/full-board gates.',
            'No complete negative-control sweep was rerun for this unchanged proof target; the v2498 controls remain separately preserved.',
        ],
        'runtime_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'sealed': True, 'native_asan_tests_per_configuration': [1052, 1203],
                      'capped_schedules': 39, 'conditional_proof_units': 737, 'dependency_units_separate': 453}))


if __name__ == '__main__':
    main()
