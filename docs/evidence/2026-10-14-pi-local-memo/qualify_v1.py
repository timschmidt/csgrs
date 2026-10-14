"""Qualify Pi memo semantics under all and default features before costs."""
import gzip
import json
import os
from pathlib import Path
import re
import subprocess

from common_v1 import E, PREP, ENV, NEW_TESTS, LAYOUT_FIELDS, sha, dump, verify


def records(raw):
    return [json.loads(line) for line in raw.splitlines() if line.startswith(b'{')]


def names(raw):
    return {name.decode() for name in re.findall(rb'^test ([^ ]+) \.\.\. ok$', raw, re.MULTILINE)}


def main():
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    review = json.loads((E / 'source-review-v1.json').read_text())
    assert review['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert review['all_changed_paths_reviewed']
    variant = prep['variants']['candidate']
    assert review['candidate_patch_sha256'] == variant['source_patch_sha256']
    assert prep['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert prep['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    out = E / 'qualification-v1'
    out.mkdir(exist_ok=False)
    protocol = {'runner_sha256': sha(__file__), 'assessor_sha256': sha(E / 'assess_qualification_v1.py'),
        'review_sha256': sha(E / 'source-review-v1.json'), 'preparation_sha256': sha(PREP / 'protocol.json'),
        'environment': ENV, 'expected_commands': 8, 'test_threads': 2,
        'expected_all_tests': 882, 'expected_default_tests': prep['expected_default_tests'],
        'full_suites': ['all', 'default'], 'cost_samples': 0,
        'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'protocol.json', protocol)
    commands, binaries, tests, layouts = [], {}, {}, {}
    library = None
    source = Path(variant['source'])

    def guard(full=False):
        verify(prep, full=full)
        assert sha(E / 'source-review-v1.json') == protocol['review_sha256']
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        if library:
            assert sha(library['path']) == library['sha256']

    def run(name, command, features='all'):
        guard()
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

    run('format-inputs', ['rustfmt', '--check', '--edition', '2024', '--config-path',
        str(source / 'crates/.rustfmt.toml'), str(E / 'pi_memo_tests_v1.rs')])
    for features in ['all', 'default']:
        flags = ['--all-features'] if features == 'all' else []
        expected_features = prep['expected_all_features'] if features == 'all' else ['default']
        run('candidate-clippy-' + features, ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal',
            *flags, '--all-targets', '--', '-D', 'warnings'], features)
        raw = run('candidate-build-tests-' + features, ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal',
            *flags, '--lib', '--no-run', '--message-format=json'], features)
        artifacts = [r for r in records(raw) if r.get('reason') == 'compiler-artifact' and r.get('executable')
            and r['target']['name'] == 'hyperreal' and r['target']['kind'] == ['lib'] and r['profile']['test']]
        assert len(artifacts) == 1
        artifact = artifacts[0]
        path = Path(artifact['executable'])
        assert path.is_relative_to(variant['target_' + features])
        assert Path(artifact['manifest_path']) == source / 'crates/hyperreal/Cargo.toml'
        assert artifact['features'] == expected_features
        binaries[features] = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
            'features': artifact['features'], 'manifest_path': artifact['manifest_path']}
        dump(out / 'binaries.json', binaries)
        raw = run('candidate-scalar-' + features, [str(path), '--test-threads=2', '--show-output'], features)
        previous = (set(prep['parent_all_test_names']) if features == 'all'
                    else set(prep['parent_default_inventory']['test_names']))
        actual = names(raw)
        assert actual == previous | NEW_TESTS
        assert len(actual) == prep['expected_' + features + '_tests']
        assert f'test result: ok. {len(actual)} passed; 0 failed; 0 ignored;'.encode() in raw
        expected_groups = {'quadratic_tower_sign_retention_tests::': 11,
            'persistent_laurent_cache_tests::': 6, 'borrowed_integer_tests::': 2,
            'pi_memo_tests::': 3, 'laurent_cache_eligibility_tests::': 3 if features == 'all' else 2}
        assert all(sum(fragment in name for name in actual) == count for fragment, count in expected_groups.items())
        layout_lines = re.findall(rb'^PI_MEMO_LAYOUT\t([^\n]+)$', raw, re.MULTILINE)
        assert len(layout_lines) == 1
        values = [int(n) for n in layout_lines[0].split(b'\t')]
        assert len(values) == len(LAYOUT_FIELDS) and all(n > 0 for n in values)
        layouts[features] = dict(zip(LAYOUT_FIELDS, values, strict=True))
        tests[features] = {'passed': len(actual), 'all_parent_test_names_retained': True,
                           'new_tests': sorted(NEW_TESTS), 'groups': expected_groups}
        dump(out / 'layouts.json', layouts)
        dump(out / 'scalar-tests.json', tests)
    raw = run('candidate-build-library-all', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal',
        '--all-features', '--lib', '--message-format=json'])
    artifacts = [r for r in records(raw) if r.get('reason') == 'compiler-artifact'
        and r['target']['name'] == 'hyperreal' and any(p.endswith('.rlib') for p in r['filenames'])]
    assert len(artifacts) == 1
    artifact = artifacts[0]
    path = Path(next(p for p in artifact['filenames'] if p.endswith('.rlib')))
    assert path.is_relative_to(variant['target_all']) and artifact['features'] == prep['expected_all_features']
    assert Path(artifact['manifest_path']) == source / 'crates/hyperreal/Cargo.toml'
    library = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
        'features': artifact['features'], 'manifest_path': artifact['manifest_path']}
    dump(out / 'library.json', library)
    assert len(commands) == 8
    guard(full=True)
    result = {'complete': True, 'commands': 8, 'all_features_scalar_tests': 882,
        'default_features_scalar_tests': tests['default']['passed'],
        'all_parent_test_names_preserved_in_both_suites': True,
        'strict_clippy': ['all features/all targets', 'default features/all targets'],
        'layouts': layouts, 'all_original_sources_artifacts_and_main_runtime_unchanged': True,
        'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'cost_samples': 0, 'native_gates_run': False, 'runtime_candidate': True,
        'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
