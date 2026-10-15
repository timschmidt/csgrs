"""Close the regression-reader failure; retain every source and test result."""
import json
from pathlib import Path

from close_v1 import E, ROOT, mapping, read, sha

OUT = E / 'qualification-v3'


def main():
    assert not (E / 'closure-v3.json').exists() and not (OUT / 'result.json').exists()
    commands, protocol = read(OUT / 'commands.json'), read(OUT / 'protocol.json')
    failure = read(OUT / 'failure.json')
    assert len(commands) == 6 and [row['exit_code'] for row in commands] == [0] * 5 + [101]
    assert failure['failed_command'] == commands[-1]
    for name, digest in failure['artifacts_sha256'].items():
        assert sha(OUT / name) == digest
    for name, digest in protocol['inputs_sha256'].items():
        assert sha(E / name) == digest
    for version in [1, 2, 3]:
        source = ROOT / f'target/directed-scalar-search-v{version}/source'
        table = mapping(E / f'source-hashes-v{version}.json.gz')
        assert {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()} == table
        if version < 3:
            for name, digest in read(E / f'closure-v{version}.json')['trial_artifacts_sha256'].items():
                assert sha(E / f'qualification-v{version}' / name) == digest
    roots = mapping(E / 'main-source-before.json.gz')
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    binaries = read(OUT / 'binaries.json')
    for row in [*binaries.values(), *protocol['preserved_binaries']]:
        assert sha(Path(row['path'])) == row['sha256']
    stdout = (OUT / 'probe-features-tests.stdout').read_text()
    assert '6 passed; 1 failed; 0 ignored; 0 measured; 0 filtered out' in stdout
    assert 'original_mixed_number_failure_uses_exact_fraction_wire_output ... FAILED' in stdout
    assert 'BadFraction' in stdout and 'signed_and_mixed_fractions_round_trip_through_independent_readers ... ok' in stdout
    assert 'Ran 23 tests' in (OUT / 'python-tests.stderr').read_text()
    assert not (OUT / 'campaign').exists()
    all_logs = ''.join(p.read_text() for p in OUT.rglob('*') if p.suffix in ['.stdout', '.stderr'])
    assert all(marker not in all_logs for marker in [
        'ERROR: AddressSanitizer', 'ERROR: LeakSanitizer', 'SUMMARY: AddressSanitizer',
        'LeakSanitizer has encountered a fatal error', 'DEADLYSIGNAL'])
    report = {
        'status': 'closed-failed', 'driver_terminal_exit_code_observed': 1,
        'commands_executed': 6, 'command_exit_codes': [row['exit_code'] for row in commands],
        'python_tests_passed': 23, 'rust_tests_passed': 6, 'rust_tests_failed': 1,
        'finding': 'The regression test incorrectly assumed Hyperreal FromStr accepts the mixed-number form emitted by human Display. The failed unwrap is in the test reader for the old response, after the new probe and fraction encoder succeeded.',
        'next_candidate_requirement': 'Preserve the complete old input and response. Reconstruct each mixed display value independently as sign*(whole+fraction) using exact rational arithmetic, then compare it with the new explicit fraction output. Do not change production parsing or display.',
        'sources_verified': {'v1': 10161, 'v2': 10163, 'v3': 10166, 'main': len(roots)},
        'executed_binaries': binaries,
        'trial_artifacts_sha256': {str(p.relative_to(OUT)): sha(p) for p in OUT.rglob('*') if p.is_file()},
        'driver_logs_sha256': {name: sha(E / name) for name in ['driver-v3.stdout', 'driver-v3.stderr']},
        'closure_sources_sha256': {name: sha(E / name) for name in ['close_v1.py', 'close_v3.py']},
        'sanitizer_error_markers_observed': False, 'campaign_started': False,
        'competitive_contract_v1': {'sha256': sha(E / 'competitive-contract-v1.json'),
            'status': 'dependency-failed-benchmark-not-started',
            'reason': 'The contract required a completed v3 campaign; v3 closed during unit tests.'},
        'competitive_benchmark_complete': False, 'admitted': False,
        'P38_complete': False, 'plan_complete': False,
        'closed_trial_retried_resumed_filtered_extended_or_pooled': False,
    }
    (E / 'closure-v3.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'closure_sha256': sha(E / 'closure-v3.json')}))


if __name__ == '__main__':
    main()
