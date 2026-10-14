"""Qualify the same tests on unchanged runtime and the isolated eligibility candidate."""
import gzip
import json
import os
from pathlib import Path
import re
import subprocess

from eligibility_common_v1 import E, PARENT, PREP, ENV, sha, dump, verify

PREFIX = 'computable::node::laurent_cache_eligibility_tests::'
TESTS = {PREFIX + name for name in [
    'zero_budget_proofs_survive_dropping_other_owners_for_all_publishing_kinds',
    'leaf_and_opaque_queries_preserve_other_cache_payloads_and_answers',
    'serde_resets_proof_storage_and_preserves_node_shape_and_value']}
DEFAULT_TESTS = {name for name in TESTS if '::serde_' not in name}


def records(raw):
    return [json.loads(line) for line in raw.splitlines() if line.startswith(b'{')]


def test_names(raw):
    return {name.decode() for name in re.findall(rb'^test ([^ ]+) \.\.\. ok$', raw, flags=re.MULTILINE)}


def main():
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    review = json.loads((E / 'source-review-v1.json').read_text())
    assert review['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert review['all_changed_paths_reviewed']
    assert review['candidate_patch_sha256'] == prep['variants']['candidate']['source_patch_sha256']
    assert prep['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert prep['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    parent_lib = json.loads((PARENT / 'qualification-v1/library.json').read_text())
    out = E / 'qualification-v1'
    out.mkdir(exist_ok=False)
    protocol = {'runner_sha256': sha(E / 'qualify_v1.py'), 'review_sha256': sha(E / 'source-review-v1.json'),
        'preparation_sha256': sha(PREP / 'protocol.json'), 'environment': ENV,
        'expected_commands': 10, 'test_threads': 2, 'cost_samples': 0,
        'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'protocol.json', protocol)
    commands, binaries, library = [], {}, None

    def guard(full=False):
        verify(prep, full=full)
        assert sha(E / 'source-review-v1.json') == protocol['review_sha256']
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        if library:
            assert sha(library['path']) == library['sha256']

    def run(name, command, group, features='all'):
        guard()
        variant = prep['variants'][group]
        source = Path(variant['source'])
        env = ENV | {'CARGO_TARGET_DIR': variant['target_' + features]}
        print(json.dumps({'start': name}), flush=True)
        done = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
            cwd=source, env=os.environ | env, capture_output=True)
        row = {'name': name, 'command': command, 'cwd': str(source), 'environment': env, 'exit_code': done.returncode}
        for stream in ['stdout', 'stderr']:
            path = out / (name + '.' + stream + '.gz')
            path.write_bytes(gzip.compress(getattr(done, stream), mtime=0))
            row[stream + '_sha256'] = sha(path)
        commands.append(row)
        dump(out / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': done.returncode}), flush=True)
        assert done.returncode == 0, name
        guard()
        return done.stdout

    def build_tests(group, features='all'):
        args = ['--all-features'] if features == 'all' else []
        raw = run(group + '-build-tests-' + features, ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal',
            *args, '--lib', '--no-run', '--message-format=json'], group, features)
        artifacts = [r for r in records(raw) if r.get('reason') == 'compiler-artifact' and r.get('executable')
            and r['target']['name'] == 'hyperreal' and r['target']['kind'] == ['lib'] and r['profile']['test']]
        assert len(artifacts) == 1
        artifact = artifacts[0]
        path = Path(artifact['executable'])
        variant = prep['variants'][group]
        assert path.is_relative_to(variant['target_' + features])
        assert Path(artifact['manifest_path']) == Path(variant['source']) / 'crates/hyperreal/Cargo.toml'
        if features == 'all':
            assert artifact['features'] == parent_lib['features']
        else:
            assert artifact['features'] == ['default']
        binaries[group + '-' + features] = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
            'features': artifact['features'], 'manifest_path': artifact['manifest_path']}
        dump(out / 'binaries.json', binaries)
        return str(path)

    run('baseline-clippy-all', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal',
        '--all-features', '--all-targets', '--', '-D', 'warnings'], 'baseline')
    binary = build_tests('baseline')
    raw = run('baseline-controls-all', [binary, 'laurent_cache_eligibility_tests::', '--test-threads=2'], 'baseline')
    assert test_names(raw) == TESTS and b'test result: ok. 3 passed; 0 failed; 0 ignored;' in raw
    run('candidate-clippy-all', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal',
        '--all-features', '--all-targets', '--', '-D', 'warnings'], 'candidate')
    binary = build_tests('candidate')
    raw = run('candidate-scalar-all', [binary, '--test-threads=2'], 'candidate')
    parent_tests = test_names(gzip.decompress((PARENT / 'qualification-v1/scalar-library.stdout.gz').read_bytes()))
    names = test_names(raw)
    assert len(parent_tests) == 876 and names == parent_tests | TESTS and len(names) == 879
    assert b'test result: ok. 879 passed; 0 failed; 0 ignored;' in raw
    assert sum('quadratic_tower_sign_retention_tests::' in name for name in names) == 11
    assert sum('persistent_laurent_cache_tests::' in name for name in names) == 6
    assert sum('borrowed_integer_tests::' in name for name in names) == 2
    dump(out / 'scalar-tests.json', {'all_features_passed': len(names), 'parent_876_retained': True,
        'new_controls': sorted(TESTS), 'P4_tests': 11, 'persistent_cache_tests': 6, 'borrowed_integer_tests': 2})
    run('candidate-clippy-default', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal',
        '--all-targets', '--', '-D', 'warnings'], 'candidate', 'default')
    binary = build_tests('candidate', 'default')
    raw = run('candidate-controls-default', [binary, 'laurent_cache_eligibility_tests::', '--test-threads=2'],
        'candidate', 'default')
    assert test_names(raw) == DEFAULT_TESTS and b'test result: ok. 2 passed; 0 failed; 0 ignored;' in raw
    raw = run('candidate-build-library-all', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal',
        '--all-features', '--lib', '--message-format=json'], 'candidate')
    artifacts = [r for r in records(raw) if r.get('reason') == 'compiler-artifact'
        and r['target']['name'] == 'hyperreal' and any(p.endswith('.rlib') for p in r['filenames'])]
    assert len(artifacts) == 1
    artifact = artifacts[0]
    path = Path(next(p for p in artifact['filenames'] if p.endswith('.rlib')))
    variant = prep['variants']['candidate']
    assert path.is_relative_to(variant['target_all']) and artifact['features'] == parent_lib['features']
    assert Path(artifact['manifest_path']) == Path(variant['source']) / 'crates/hyperreal/Cargo.toml'
    library = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
        'features': artifact['features'], 'manifest_path': artifact['manifest_path']}
    dump(out / 'library.json', library)
    assert len(commands) == 10
    guard(full=True)
    result = {'complete': True, 'commands': len(commands), 'all_features_scalar_tests': 879,
        'baseline_all_features_new_controls': 3, 'candidate_default_new_controls': 2,
        'P4_tests': 11, 'persistent_cache_tests': 6, 'borrowed_integer_tests': 2,
        'strict_clippy': ['baseline all features/all targets', 'candidate all features/all targets',
                          'candidate default features/all targets'],
        'all_original_sources_artifacts_and_main_runtime_unchanged': True,
        'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'cost_samples': 0, 'native_gates_run': False, 'runtime_candidate': True,
        'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
