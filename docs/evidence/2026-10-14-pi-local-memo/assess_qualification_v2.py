"""Verify the complete two-feature qualification and measured memo layouts."""
import gzip
import json
from pathlib import Path
import re

from pi_memo_common_v2 import E, BASE, PREP, NEW_TESTS, LAYOUT_FIELDS, sha, dump, verify


def names(raw):
    return set(re.findall(r'^test ([^ ]+) \.\.\. ok$', raw, re.MULTILINE))


def main():
    destination = E / 'qualification-assessment-v2.json'
    assert not destination.exists()
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    out = E / 'qualification-v2'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 8
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    assert {p.name for p in out.iterdir() if p.is_file()} == set(result['artifact_sha256']) | {'result.json'}
    protocol = json.loads((out / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(E / 'qualify_v2.py') and protocol['assessor_sha256'] == sha(__file__)
    assert protocol['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert protocol['review_sha256'] == sha(E / 'source-review-v2.json')
    commands = json.loads((out / 'commands.json').read_text())
    expected = ['format-inputs', 'candidate-clippy-all', 'candidate-build-tests-all', 'candidate-scalar-all',
                'candidate-clippy-default', 'candidate-build-tests-default', 'candidate-scalar-default',
                'candidate-build-library-all']
    assert [r['name'] for r in commands] == expected and all(r['exit_code'] == 0 for r in commands)
    raw = {}
    for command in commands:
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[command['name'], stream] = gzip.decompress(path.read_bytes()).decode()
        if 'clippy' in command['name']:
            assert command['command'][-3:] == ['--', '-D', 'warnings']
            assert '--all-targets' in command['command']
            assert ('--all-features' in command['command']) == command['name'].endswith('-all')
    old_all = names(gzip.decompress((BASE / 'qualification-v1/candidate-scalar-all.stdout.gz').read_bytes()).decode())
    old_default = set(re.findall(r'^(.+): test$', gzip.decompress((PREP / 'parent-default-list.stdout.gz').read_bytes()).decode(), re.MULTILINE))
    assert len(old_all) == 879 and set(prep['parent_all_test_names']) == old_all
    assert set(prep['parent_default_inventory']['test_names']) == old_default
    tests = json.loads((out / 'scalar-tests.json').read_text())
    layouts = json.loads((out / 'layouts.json').read_text())
    binaries = json.loads((out / 'binaries.json').read_text())
    library = json.loads((out / 'library.json').read_text())
    assert set(binaries) == set(layouts) == set(tests) == {'all', 'default'}
    assert all(sha(r['path']) == r['sha256'] for r in [*binaries.values(), library])
    for features, previous in [('all', old_all), ('default', old_default)]:
        text = raw['candidate-scalar-' + features, 'stdout']
        actual = names(text)
        assert actual == previous | NEW_TESTS
        assert tests[features]['passed'] == len(actual) == prep['expected_' + features + '_tests']
        assert f'test result: ok. {len(actual)} passed; 0 failed; 0 ignored;' in text
        assert set(tests[features]['new_tests']) == NEW_TESTS
        assert all(sum(fragment in name for name in actual) == count
                   for fragment, count in tests[features]['groups'].items())
        line = re.findall(r'^PI_MEMO_LAYOUT\t([^\n]+)$', text, re.MULTILINE)
        assert len(line) == 1
        assert layouts[features] == dict(zip(LAYOUT_FIELDS, map(int, line[0].split('\t')), strict=True))
        artifacts = [json.loads(line) for line in raw['candidate-build-tests-' + features, 'stdout'].splitlines() if line.startswith('{')]
        artifact = [r for r in artifacts if r.get('reason') == 'compiler-artifact'
                    and r.get('executable') == binaries[features]['path']]
        assert len(artifact) == 1 and artifact[0]['features'] == binaries[features]['features']
        assert artifact[0]['manifest_path'] == binaries[features]['manifest_path'] == str(Path(prep['variants']['candidate']['source']) / 'crates/hyperreal/Cargo.toml')
        assert artifact[0]['features'] == (prep['expected_all_features'] if features == 'all' else ['default'])
        command = next(r['command'] for r in commands if r['name'] == 'candidate-scalar-' + features)
        assert command == [binaries[features]['path'], '--test-threads=2', '--show-output']
    artifacts = [json.loads(line) for line in raw['candidate-build-library-all', 'stdout'].splitlines() if line.startswith('{')]
    artifacts = [r for r in artifacts if r.get('reason') == 'compiler-artifact' and library['path'] in r.get('filenames', [])]
    assert len(artifacts) == 1 and artifacts[0]['features'] == library['features'] == prep['expected_all_features']
    assert artifacts[0]['manifest_path'] == library['manifest_path'] == binaries['all']['manifest_path']
    assert result['layouts'] == layouts
    dump(destination, {'status': 'pi-local-memo-qualified-costs-pending',
        'assessor_sha256': sha(__file__), 'preparation_sha256': sha(PREP / 'protocol.json'),
        'result_sha256': sha(out / 'result.json'), 'commands_passed': 8,
        'all_features_scalar_tests': tests['all']['passed'], 'default_features_scalar_tests': tests['default']['passed'],
        'all_parent_test_names_preserved_in_both_suites': True, 'new_controls': sorted(NEW_TESTS),
        'retained_test_groups': {mode: row['groups'] for mode, row in tests.items()},
        'strict_clippy_configurations': result['strict_clippy'], 'layouts': layouts,
        'memo_value_layout_delta_bytes': layouts['all']['candidate_memo_value'] - layouts['all']['old_memo_value'],
        'keyed_entry_layout_delta_bytes': layouts['all']['candidate_keyed_entry'] - layouts['all']['old_keyed_entry'],
        'all_artifacts_and_original_source_guards_passed': True, 'library': library,
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in out.iterdir() if p.is_file()},
        'cost_samples': 0, 'native_gates_run': False,
        'limits': ['Layout sizes describe this compiler/target and do not establish heap or timing savings.',
                   'Correctness qualification does not admit this combined runtime or close prior cost failures.',
                   'The entire properties, targets and crate future-work scope remains open.'],
        'runtime_candidate': True, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'qualified': True, 'all_features_tests': tests['all']['passed'],
        'default_features_tests': tests['default']['passed'], 'layouts': layouts,
        'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
