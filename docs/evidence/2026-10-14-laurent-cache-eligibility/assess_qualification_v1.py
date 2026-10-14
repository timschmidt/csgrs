"""Independently verify the complete closed qualification, without cost claims."""
import gzip
import json
from pathlib import Path
import re

from eligibility_common_v1 import E, PARENT, PREP, sha, dump, verify


def names(raw):
    return set(re.findall(r'^test ([^ ]+) \.\.\. ok$', raw, re.MULTILINE))


def main():
    destination = E / 'qualification-assessment-v1.json'
    assert not destination.exists()
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    out = E / 'qualification-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 10
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    assert {p.name for p in out.iterdir() if p.is_file()} == set(result['artifact_sha256']) | {'result.json'}
    protocol = json.loads((out / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(E / 'qualify_v1.py')
    assert protocol['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert protocol['review_sha256'] == sha(E / 'source-review-v1.json')
    commands = json.loads((out / 'commands.json').read_text())
    expected = ['baseline-clippy-all', 'baseline-build-tests-all', 'baseline-controls-all',
        'candidate-clippy-all', 'candidate-build-tests-all', 'candidate-scalar-all',
        'candidate-clippy-default', 'candidate-build-tests-default', 'candidate-controls-default',
        'candidate-build-library-all']
    assert [c['name'] for c in commands] == expected and all(c['exit_code'] == 0 for c in commands)
    raw = {}
    for command in commands:
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[command['name'], stream] = gzip.decompress(path.read_bytes()).decode()
        if 'clippy' in command['name']:
            assert command['command'][-3:] == ['--', '-D', 'warnings']
            assert '--all-targets' in command['command']
    parent = names(gzip.decompress((PARENT / 'qualification-v1/scalar-library.stdout.gz').read_bytes()).decode())
    controls = names(raw['baseline-controls-all', 'stdout'])
    candidate = names(raw['candidate-scalar-all', 'stdout'])
    default = names(raw['candidate-controls-default', 'stdout'])
    assert len(parent) == 876 and len(controls) == 3 and len(candidate) == 879 and len(default) == 2
    assert candidate == parent | controls and default == {n for n in controls if '::serde_' not in n}
    for command, count in [('baseline-controls-all', 3), ('candidate-scalar-all', 879), ('candidate-controls-default', 2)]:
        assert f'test result: ok. {count} passed; 0 failed; 0 ignored;' in raw[command, 'stdout']
    binaries = json.loads((out / 'binaries.json').read_text())
    library = json.loads((out / 'library.json').read_text())
    assert set(binaries) == {'baseline-all', 'candidate-all', 'candidate-default'}
    assert all(sha(row['path']) == row['sha256'] for row in [*binaries.values(), library])
    assert binaries['candidate-default']['features'] == ['default']
    assert binaries['baseline-all']['features'] == binaries['candidate-all']['features'] == library['features']
    for label, command in [('baseline-all', 'baseline-build-tests-all'),
                           ('candidate-all', 'candidate-build-tests-all'),
                           ('candidate-default', 'candidate-build-tests-default')]:
        events = [json.loads(line) for line in raw[command, 'stdout'].splitlines() if line.startswith('{')]
        artifact = [r for r in events if r.get('reason') == 'compiler-artifact'
                    and r.get('executable') == binaries[label]['path']]
        assert len(artifact) == 1 and artifact[0]['features'] == binaries[label]['features']
        assert artifact[0]['manifest_path'] == binaries[label]['manifest_path']
    counts = {key: sum(fragment in name for name in candidate) for key, fragment in [
        ('P4', 'quadratic_tower_sign_retention_tests::'),
        ('persistent_cache', 'persistent_laurent_cache_tests::'),
        ('borrowed_integer', 'borrowed_integer_tests::')]}
    assert counts == {'P4': 11, 'persistent_cache': 6, 'borrowed_integer': 2}
    dump(destination, {'status': 'lookup-eligibility-qualified-costs-pending',
        'assessor_sha256': sha(__file__), 'preparation_sha256': sha(PREP / 'protocol.json'),
        'result_sha256': sha(out / 'result.json'), 'commands_passed': 10,
        'all_features_scalar_tests': 879, 'original_876_tests_preserved': True,
        'baseline_all_features_controls': sorted(controls), 'candidate_default_controls': sorted(default),
        'retained_test_groups': counts, 'strict_clippy_configurations': result['strict_clippy'],
        'all_artifacts_and_original_source_guards_passed': True,
        'source_variants': {group: {'source': variant['source'], 'files': variant['files'],
                                  'source_hashes_sha256': variant['source_hashes_sha256']}
                            for group, variant in prep['variants'].items()},
        'library': library, 'cost_samples': 0, 'native_gates_run': False,
        'limits': ['Default features ran only the two new controls; no full default suite claim.',
                   'Correctness qualification does not admit the candidate or resolve earlier library/native cost failures.',
                   'No runtime worktree source was changed; both isolated snapshots remain frozen.'],
        'runtime_candidate': True, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'qualified': True, 'all_features_scalar_tests': 879,
                      'baseline_controls': 3, 'default_controls': 2, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
