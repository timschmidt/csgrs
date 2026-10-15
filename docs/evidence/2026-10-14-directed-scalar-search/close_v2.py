"""Close the second scalar-search trial at its exact serialization failure."""
import json
from pathlib import Path

from close_v1 import E, ROOT, mapping, read, sha

OUT = E / 'qualification-v2'


def main():
    assert not (E / 'closure-v2.json').exists() and not (OUT / 'result.json').exists()
    failure, protocol = read(OUT / 'failure.json'), read(OUT / 'protocol.json')
    commands = read(OUT / 'commands.json')
    assert len(commands) == 10 and [row['exit_code'] for row in commands] == [0] * 9 + [1]
    assert failure['failed_command'] == commands[-1] and commands[-1]['name'] == 'campaign'
    for name, digest in failure['artifacts_sha256'].items():
        assert sha(OUT / name) == digest, name
    for name, digest in protocol['inputs_sha256'].items():
        assert sha(E / name) == digest, name
    for version in [1, 2]:
        source = ROOT / f'target/directed-scalar-search-v{version}/source'
        table = mapping(E / f'source-hashes-v{version}.json.gz')
        assert {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()} == table
    roots = mapping(E / 'main-source-before.json.gz')
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    for name, digest in read(E / 'closure-v1.json')['trial_artifacts_sha256'].items():
        assert sha(E / 'qualification-v1' / name) == digest
    binaries = read(OUT / 'binaries.json')
    for row in [*binaries.values(), *protocol['preserved_binaries']]:
        assert sha(Path(row['path'])) == row['sha256']
    for name in ['probe-features-tests', 'all-features-tests']:
        assert '5 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out' in (OUT / (name + '.stdout')).read_text()
    assert 'Ran 21 tests' in (OUT / 'python-tests.stderr').read_text()
    campaign = OUT / 'campaign'
    partial = read(campaign / 'failure.json')
    assert partial['completed_probes'] == 4 and not (campaign / 'result.json').exists()
    for name, digest in partial['artifacts_sha256'].items():
        assert sha(campaign / name) == digest
    processes = [read(p) for p in campaign.glob('round-*.process.json')]
    assert len(processes) == 5 and all(p['exit_code'] == 0 and p['status'] == 'exited' for p in processes)
    assert not list(campaign.glob('*-selection.json'))
    stem = 'round-000-ConstOffset-c062ea341e2c50e6e70f99a97e333451814ff2da78462186f5371feb6a070224'
    original = read(campaign / (stem + '.input.json'))
    response = read(campaign / (stem + '.stdout'))
    assert original == {'bucket': 'ConstOffset', 'genome': [47, 128, 223, 71, 133, 255, 155, 107]}
    assert response['recipe']['input'] == original and response['status'] == 'ok'
    assert response['oracle']['interval'][0] in partial['error']
    assert ' ' in response['oracle']['interval'][0] and 'Invalid literal for Fraction' in partial['error']
    all_logs = ''.join(p.read_text() for p in OUT.rglob('*') if p.suffix in ['.stdout', '.stderr'])
    assert all(marker not in all_logs for marker in [
        'ERROR: AddressSanitizer', 'ERROR: LeakSanitizer', 'SUMMARY: AddressSanitizer',
        'LeakSanitizer has encountered a fatal error', 'DEADLYSIGNAL'])
    report = {
        'status': 'closed-failed', 'driver_terminal_exit_code_observed': 1,
        'commands_executed': 10, 'command_exit_codes': [row['exit_code'] for row in commands],
        'python_tests_passed': 21, 'rust_tests_passed_each_configuration': 5,
        'rust_test_configurations': ['serde,dispatch-trace', 'all-features'],
        'campaign_probes_exited_zero': 5, 'campaign_responses_accepted_before_failure': 4,
        'campaign_selection_events': 0, 'campaign_complete': False,
        'original_input': original, 'original_recipe': response['recipe'],
        'finding': 'The Rust probe passed its numeric checks and exited zero. Its rational Display output uses mixed-number notation for some values; the Python exact-fraction reader rejects that notation. This is an evidence wire-format defect, not a discovered arithmetic contradiction.',
        'next_candidate_requirement': 'Keep the complete original input and raw response. Serialize every numeric rational endpoint and normal form as an explicit signed numerator over a positive denominator; add cross-language regression coverage.',
        'sources_verified': {'v1': 10161, 'v2': 10163, 'main': len(roots)},
        'executed_binaries': binaries,
        'trial_artifacts_sha256': {str(p.relative_to(OUT)): sha(p) for p in OUT.rglob('*') if p.is_file()},
        'driver_logs_sha256': {name: sha(E / name) for name in ['driver-v2.stdout', 'driver-v2.stderr']},
        'closure_sources_sha256': {name: sha(E / name) for name in ['close_v1.py', 'close_v2.py']},
        'sanitizer_error_markers_observed': False,
        'competitive_benchmark_complete': False, 'admitted': False,
        'P38_complete': False, 'plan_complete': False,
        'closed_trial_retried_resumed_filtered_extended_or_pooled': False,
    }
    (E / 'closure-v2.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'commands': len(commands),
                      'closure_sha256': sha(E / 'closure-v2.json')}))


if __name__ == '__main__':
    main()
