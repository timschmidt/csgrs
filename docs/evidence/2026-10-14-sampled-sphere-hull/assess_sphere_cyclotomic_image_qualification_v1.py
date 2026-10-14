"""Independently check the completed, unchanged broad Rust qualification."""
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-cyclotomic-image-qualification-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    destination = E / 'sphere-cyclotomic-image-qualification-assessment-v1.json'
    assert not destination.exists()
    result = json.loads((OUT / 'result.json').read_text())
    assert result['complete'] and result['commands_all_zero'] == 11
    assert result['all_frozen_sources_unchanged']
    assert all(sha(OUT / p) == h for p, h in result['artifacts_sha256'].items())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    assert all(sha(E / p) == h for p, h in protocol['inputs_sha256'].items())
    prior_mapping = E / 'sphere-cyclotomic-image-v1/source-hashes.json.gz'
    assert sha(prior_mapping) == protocol['source_mapping_sha256']
    source_mapping = json.loads(gzip.decompress(prior_mapping.read_bytes()))
    root_mapping = json.loads(gzip.decompress((OUT / 'root-source-hashes.json.gz').read_bytes()))
    assert all(sha(Path(protocol['source']) / p) == h for p, h in source_mapping.items())
    assert all(sha(ROOT / p) == h for p, h in root_mapping.items())
    commands = json.loads((OUT / 'commands.json').read_text())
    names = ['format', 'default-clippy', 'default-tests', 'all-features-clippy',
             'all-features-tests', 'doctests', 'docs', 'grammar-native-consumers',
             'hyperreal-fuzz-check', 'hyperlimit-fuzz-check', 'hypermesh-fuzz-check']
    assert [row['name'] for row in commands] == names
    assert all(row['exit_code'] == 0 for row in commands)
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    tests = {}
    pattern = re.compile(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; '
                         r'(\d+) measured; (\d+) filtered out;', re.MULTILINE)
    for name in ['default-tests', 'all-features-tests', 'doctests', 'grammar-native-consumers']:
        stdout = (OUT / (name + '.stdout')).read_text()
        summaries = [tuple(map(int, row)) for row in pattern.findall(stdout)]
        assert summaries and 'test result: FAILED' not in stdout
        totals = [sum(row[index] for row in summaries) for index in range(5)]
        assert totals[1] == totals[3] == totals[4] == 0
        tests[name] = dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered_out'], totals))
        tests[name]['completed_test_summaries'] = len(summaries)
    assert tests['default-tests']['passed'] == 3926
    assert tests['grammar-native-consumers']['passed'] == 18
    value = {
        'complete': True, 'assessor_sha256': sha(__file__),
        'result_sha256': sha(OUT / 'result.json'), 'commands_all_zero': 11,
        'source_files_rechecked': len(source_mapping), 'root_files_rechecked': len(root_mapping),
        'tests': tests,
        'packages': protocol['packages'], 'features': ['default', 'all-features'],
        'checks': 'Format, strict all-target Clippy, library/integration tests, all-feature doctests and warning-denied docs, selected native grammar consumers, three fuzz manifests checked with all binary targets.',
        'limits': [
            'Ignored tests remain ignored. Counts describe these runs, not complete input or branch coverage.',
            'This unchanged finite-field candidate remains isolated from main; its earlier timing screen is not a complete cost admission.',
            'No executed fuzz campaign, benchmark, production formal proof, allocation/retention/size admission, all-workspace CI or push qualification is claimed.',
            'No process timing in this qualification is a matched performance comparison.'
        ],
        'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(value, indent=2) + '\n')
    print(json.dumps({'complete': True, 'commands': 11, 'tests': tests,
                      'assessment_sha256': sha(destination)}))


if __name__ == '__main__':
    main()
