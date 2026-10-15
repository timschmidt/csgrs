"""Close the failed first qualification without rerunning or altering its artifacts."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'qualification-v1'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def mapping(path):
    return json.loads(gzip.decompress(path.read_bytes()))


def main():
    assert not (E / 'closure-v1.json').exists()
    assert not (OUT / 'result.json').exists()
    failure = read(OUT / 'failure.json')
    protocol = read(OUT / 'protocol.json')
    commands = read(OUT / 'commands.json')
    assert len(commands) == 6 and [r['exit_code'] for r in commands] == [0] * 5 + [101]
    assert failure['failed_command'] == commands[-1]
    assert commands[-1]['name'] == 'probe-features-tests'
    for name, digest in failure['artifacts_sha256'].items():
        assert sha(OUT / name) == digest, name
    for name, digest in protocol['inputs_sha256'].items():
        assert sha(E / name) == digest, name
    source = Path(protocol['source'])
    sources = mapping(E / 'source-hashes-v1.json.gz')
    assert {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()} == sources
    roots = mapping(E / 'main-source-before.json.gz')
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    binaries = read(OUT / 'binaries.json')
    for row in [*binaries.values(), *protocol['preserved_binaries']]:
        assert sha(Path(row['path'])) == row['sha256']
        assert Path(row['path']).stat().st_size == row['bytes']
    stdout = (OUT / 'probe-features-tests.stdout').read_text()
    stderr = (OUT / 'probe-features-tests.stderr').read_text()
    assert '2 passed; 1 failed; 0 ignored; 0 measured; 0 filtered out' in stdout
    assert 'genome: [0, 0, 0, 0, 0, 0, 0, 0]' in stdout and 'pi_offset: "31/10"' in stdout
    assert 'left: "Irrational"' in stdout and 'right: "ConstOffset"' in stdout
    assert all(marker not in stdout + stderr for marker in [
        'ERROR: AddressSanitizer', 'ERROR: LeakSanitizer', 'SUMMARY: AddressSanitizer',
        'LeakSanitizer has encountered a fatal error', 'DEADLYSIGNAL'])
    classification = ROOT / 'crates/hyperreal/src/real/arithmetic/classification.rs'
    assert 'threshold <= Rational::new(3)' in classification.read_text()
    report = {
        'status': 'closed-failed', 'driver_terminal_exit_code_observed': 1,
        'commands_executed': 6, 'command_exit_codes': [row['exit_code'] for row in commands],
        'python_tests_passed': 13, 'rust_tests_passed': 2, 'rust_tests_failed': 1,
        'original_input': {'bucket': 'ConstOffset', 'genome': [0] * 8},
        'exact_expression': '(pi - 31/10) * 1',
        'observed_private_class': 'Irrational', 'incorrect_expected_private_class': 'ConstOffset',
        'finding': 'Generator contract mismatch: the production ConstOffset constructor deliberately requires a cheap positive inner-value certificate. Negative offsets from pi are admitted only when their magnitude is at most three; near cancellation remains a generic computable.',
        'constructor_source': str(classification.relative_to(ROOT)),
        'constructor_source_sha256': sha(classification),
        'next_candidate_requirement': 'Preserve this exact failure and the full near-pi parameter range as generic Irrational cases; add representation-preserving ConstOffset recipes without changing the production constructor.',
        'source_files_verified': len(sources), 'main_source_files_verified': len(roots),
        'executed_binaries': binaries,
        'trial_artifacts_sha256': {str(p.relative_to(OUT)): sha(p) for p in OUT.rglob('*') if p.is_file()},
        'driver_logs_sha256': {name: sha(E / name) for name in ['driver-v1.stdout', 'driver-v1.stderr']},
        'closure_script_sha256': sha(Path(__file__)),
        'sanitizer_error_markers_observed_in_executed_test_logs': False,
        'complete_sanitizer_qualification': False, 'campaign_started': False,
        'competitive_benchmark_complete': False, 'admitted': False,
        'P38_complete': False, 'plan_complete': False,
        'closed_trial_retried_resumed_filtered_extended_or_pooled': False,
    }
    (E / 'closure-v1.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'commands': len(commands),
                      'closure_sha256': sha(E / 'closure-v1.json')}))


if __name__ == '__main__':
    main()
