"""Qualify the isolated sign-transfer repair before measuring or admitting it.

All commands are sequential. Main, the frozen baseline, candidate sources and
executed inputs are checked before and after each command. The original public
probe is linked to the final all-feature candidate library and runs its twelve
unchanged cases in fresh processes after all builds/tests have terminated.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'qualification-v5'
Q = ROOT / 'docs/evidence/2026-10-13-quadratic-tower-sign-transfer'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    OUT.mkdir(exist_ok=False)
    preparation = json.loads((E / 'preparation-v4/protocol.json').read_text())
    baseline = json.loads((E / 'baseline-v1/protocol.json').read_text())
    source, target = Path(preparation['source']), Path(preparation['target'])
    maps = {name: json.loads(gzip.decompress((E / path).read_bytes())) for name, path in [
        ('original', 'baseline-v1/original-source-hashes.json.gz'),
        ('baseline', 'baseline-v1/baseline-source-hashes.json.gz'),
        ('candidate', 'preparation-v4/candidate-source-hashes.json.gz'),
        ('control', 'preparation-v4/control-source-hashes.json.gz'),
        ('previous-candidate', 'preparation-v3/candidate-source-hashes.json.gz')]}
    env = baseline['environment'] | {'CARGO_TARGET_DIR': str(target)}
    original_probe = json.loads((Q / 'observation-v2/protocol.json').read_text())
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'prior_qualification_failure_sha256': sha(E / 'qualification-failure-v4.json'),
                'preparation_protocol_sha256': sha(E / 'preparation-v4/protocol.json'),
                'failed_preparation_sha256': sha(E / 'preparation-failure-v1.json'),
                'second_failed_preparation_sha256': sha(E / 'preparation-failure-v2.json'),
                'baseline_protocol_sha256': sha(E / 'baseline-v1/protocol.json'),
                'baseline_result_sha256': sha(E / 'baseline-v1/result.json'),
                'probe_sha256': sha(Q / 'probe_v2.rs'), 'original_observations_sha256': sha(Q / 'observation-v2/observations.json'),
                'source': str(source), 'target': str(target), 'source_count': len(maps['candidate']),
                'environment': env, 'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'cases': original_probe['cases'], 'command_timeout_seconds': 1800, 'probe_timeout_seconds': 120,
                'full_pre_push_matrix': False, 'runtime_admitted': False}
    assert protocol['runner_sha256'] == preparation['qualification_runner_sha256']
    assert protocol['probe_sha256'] == original_probe['probe_sha256']
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands, bindings, observations = [], {}, []

    def verify():
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / 'qualification-failure-v4.json') == protocol['prior_qualification_failure_sha256']
        failure = json.loads((E / 'qualification-failure-v4.json').read_text())
        assert all(sha(E / name) == digest for name, digest in failure['artifacts_sha256'].items())
        previous = json.loads((E / 'preparation-v3/protocol.json').read_text())
        assert all(sha(Path(previous['source']) / name) == digest for name, digest in maps['previous-candidate'].items())
        assert all(sha(Path(preparation['control_source']) / name) == digest for name, digest in maps['control'].items())
        assert sha(E / 'preparation-v4/protocol.json') == protocol['preparation_protocol_sha256']
        assert sha(E / 'preparation-failure-v1.json') == protocol['failed_preparation_sha256']
        assert sha(E / 'preparation-failure-v2.json') == protocol['second_failed_preparation_sha256']
        second_failed = json.loads((E / 'preparation-failure-v2.json').read_text())
        second_map = json.loads(gzip.decompress((E / 'preparation-v2/failed-source-hashes.json.gz').read_bytes()))
        assert all(sha(Path(second_failed['failed_source']) / name) == digest for name, digest in second_map.items())
        failed = json.loads((E / 'preparation-failure-v1.json').read_text())
        failed_map = json.loads(gzip.decompress((E / 'preparation-v1/failed-source-hashes.json.gz').read_bytes()))
        assert all(sha(Path(failed['failed_source']) / name) == digest for name, digest in failed_map.items())
        assert sha(E / 'baseline-v1/protocol.json') == protocol['baseline_protocol_sha256']
        assert sha(E / 'baseline-v1/result.json') == protocol['baseline_result_sha256']
        assert sha(E / 'quadratic_tower_sign_retention_tests_v4.rs') == preparation['test_sha256']
        assert sha(Q / 'probe_v2.rs') == protocol['probe_sha256']
        assert sha(Q / 'observation-v2/observations.json') == protocol['original_observations_sha256']
        assert sources() == baseline['main_source_sha256']
        for directory, entries in [(ROOT, maps['original']), (Path(baseline['source']), maps['baseline']), (source, maps['candidate'])]:
            assert all(sha(directory / name) == digest for name, digest in entries.items())
        assert all(sha(ROOT / name) == digest for name, digest in baseline['pending_native_sha256'].items())
        assert all(sha(Path(row['path'])) == row['sha256'] for row in bindings.values())

    def run(name, command, extra_env=None, probe=False, expected=0, working_directory=None):
        verify()
        wrapped = ['timeout', '--signal=TERM', '--kill-after=10s', '120' if probe else '1800', *command]
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(wrapped, cwd=working_directory or source, env=os.environ | env | (extra_env or {}), stdout=stdout, stderr=stderr).returncode
        raw = (OUT / (name + '.stdout')).read_text()
        row = {'name': name, 'command': wrapped, 'cwd': str(working_directory or source), 'expected_exit_code': expected, 'extra_environment': extra_env or {},
               'exit_code': code, 'seconds': time.monotonic() - started,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr')),
               'test_summaries': re.findall(r'^test result: .+$', raw, re.MULTILINE)}
        commands.append(row)
        (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps({key: row[key] for key in ['name', 'exit_code', 'seconds', 'test_summaries']}), flush=True)
        verify()
        assert code == expected, name
        return raw

    def bind_artifact(name, artifact, path, binding_source=None, binding_target=None):
        path = Path(path)
        assert path.is_relative_to(binding_target or target)
        assert Path(artifact['manifest_path']) == (binding_source or source) / 'crates/hyperreal/Cargo.toml'
        bindings[name] = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
                          'manifest_path': artifact['manifest_path'], 'features': artifact['features'],
                          'profile': artifact['profile'], 'target': artifact['target']}
        (OUT / 'bindings.json').write_text(json.dumps(bindings, indent=2) + '\n')

    run('fmt', ['cargo', 'fmt', '--all', '--', '--check'])
    control_source, control_target = Path(preparation['control_source']), Path(preparation['control_target'])
    control_env = {'CARGO_TARGET_DIR': str(control_target)}
    raw = run('build-baseline-controls', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--no-run', '--message-format=json'], control_env, working_directory=control_source)
    selected = [event for event in map(json.loads, raw.splitlines()) if event.get('reason') == 'compiler-artifact'
                and event['target']['name'] == 'hyperreal' and event['target']['kind'] == ['lib'] and event.get('executable')]
    assert len(selected) == 1 and selected[0]['profile']['test']
    bind_artifact('baseline-control-tests', selected[0], selected[0]['executable'], control_source, control_target)
    raw = run('baseline-controls', [bindings['baseline-control-tests']['path'], 'quadratic_tower_sign_retention_tests', '--nocapture', '--test-threads=1'], control_env, expected=101, working_directory=control_source)
    assert '5 passed; 6 failed;' in raw
    expected_failures = json.loads((E / 'baseline-v1/result.json').read_text())['regression_failures']
    actual_failures = re.findall(r'^test (\S+) \.\.\. FAILED$', raw, re.MULTILINE)
    assert sorted(actual_failures) == sorted(expected_failures)
    raw = run('build-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--tests', '--no-run', '--message-format=json'])
    artifacts = [event for event in map(json.loads, raw.splitlines()) if event.get('reason') == 'compiler-artifact']
    selected = [event for event in artifacts if event['target']['name'] == 'hyperreal' and event['target']['kind'] == ['lib'] and event.get('executable')]
    assert len(selected) == 1 and selected[0]['profile']['test']
    bind_artifact('unit-tests', selected[0], selected[0]['executable'])
    raw = run('focused-regressions', [bindings['unit-tests']['path'], 'quadratic_tower_sign_retention_tests', '--nocapture', '--test-threads=1'])
    assert '11 passed; 0 failed;' in raw
    run('all-feature-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--tests', '--', '--test-threads=2'])
    run('default-tests', ['cargo', 'test', '--locked', '-p', 'hyperreal', '--lib', '--tests', '--', '--test-threads=2'])
    run('doctests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--doc'])
    run('default-clippy', ['cargo', 'clippy', '--locked', '-p', 'hyperreal', '--all-targets', '--', '-D', 'warnings'])
    run('all-feature-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--all-targets', '--', '-D', 'warnings'])
    run('docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '--all-features', '--no-deps'], {'RUSTDOCFLAGS': '-D warnings'})
    # Exercise the direct production consumer in hypersolve::tensor_resultant.
    run('solver-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypersolve', '--all-features', '--lib', '--tests', '--', '--test-threads=2'])
    run('solver-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypersolve', '--all-features', '--all-targets', '--', '-D', 'warnings'])
    raw = run('build-library', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--message-format=json'])
    selected = [event for event in map(json.loads, raw.splitlines()) if event.get('reason') == 'compiler-artifact'
                and event['target']['name'] == 'hyperreal' and any(path.endswith('.rlib') for path in event['filenames'])]
    assert len(selected) == 1 and not selected[0]['profile']['test']
    bind_artifact('library', selected[0], next(path for path in selected[0]['filenames'] if path.endswith('.rlib')))
    assert bindings['library']['features'] == bindings['unit-tests']['features']
    binary = target.parent / 'observation-probe'
    run('link-probe', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(Q / 'probe_v2.rs'),
                       '-L', 'dependency=' + str(target / 'release/deps'), '--extern', 'hyperreal=' + bindings['library']['path'], '-o', str(binary)])
    bindings['probe'] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    (OUT / 'bindings.json').write_text(json.dumps(bindings, indent=2) + '\n')
    original_rows = [row['row'] for row in json.loads((Q / 'observation-v2/observations.json').read_text()) if row['variant'] == 'main']
    for index, (case, original_row) in enumerate(zip(protocol['cases'], original_rows, strict=True), 1):
        name = f'probe-{index:02d}-{case["family"]}-{case["shape"]}-{case["scale"]}'
        raw = run(name, [str(binary), case['family'], case['shape'], str(case['scale'])], probe=True)
        row = json.loads(raw)
        observations.append({'name': name, 'case': case, 'row': row})
        (OUT / 'observations.json').write_text(json.dumps(observations, indent=2) + '\n')
        assert {key: row[key] for key in case} == case
        assert row['compact_covered'] and row['repeated_covered']
        assert row['difference_tower_sign'] == 'zero' and row['all_definite_signs_agree']
        expected = 'negative' if case['scale'] < 0 else 'positive'
        for field in ['compact_immediate', 'compact_zero', 'compact_512', 'clone_zero', 'repeated_zero']:
            assert row[field] == expected
        for field in ['delta_before', 'delta_warmed', 'original_immediate', 'original_zero', 'original_clone_zero', 'original_after']:
            assert row[field] == original_row[field]
    verify()
    result = {'complete': True, 'focused_tests': 11, 'baseline_control_passes': 5, 'baseline_regression_failures': 6, 'fresh_probe_processes': len(observations),
              'all_previously_lost_signs_retained': True, 'all_probe_equalities_proven': True,
              'original_probe_facts_unchanged': True, 'sources_main_inputs_and_bindings_unchanged': True,
              'qualification_commands': len(commands), 'runtime_admitted': False,
              'remaining': 'Independent resource costs, broader/native qualification as warranted, and retention decision. Existing native failures, 35 pending entries and all properties/targets/future work remain open.'}
    (OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
