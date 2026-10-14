"""Independently close the fixed normal-form filter qualification after it exits."""
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
RUN = E / 'sphere-normal-image-qualification-v1'
PRIOR = E / 'sphere-normal-image-v1'
DESTINATION = E / 'sphere-normal-image-qualification-closure-v1.json'
COMMANDS = [
    'format', 'default-clippy', 'default-tests', 'all-features-clippy',
    'all-features-tests', 'doctests', 'docs', 'grammar-native-clippy',
    'grammar-native-consumers', 'hyperreal-fuzz-check', 'hyperlimit-fuzz-check',
    'hypermesh-fuzz-check',
]
EXPECTED_COUNTS = {
    'default-tests': (3928, 35, 95),
    'all-features-tests': (4080, 40, 96),
    'doctests': (29, 0, 4),
    'grammar-native-consumers': (19, 0, 4),
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def mapping(path):
    return json.loads(gzip.decompress(Path(path).read_bytes()))


def main():
    assert not DESTINATION.exists()
    assert not (RUN / 'failure.json').exists()
    result = read(RUN / 'result.json')
    assert result['complete'] and result['commands_all_zero'] == len(COMMANDS) == 12
    assert all(sha(RUN / name) == digest for name, digest in result['artifacts_sha256'].items())
    protocol = read(RUN / 'protocol.json')
    assert protocol['command_wall_limit_seconds'] == 1800
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    sources = mapping(PRIOR / 'source-hashes.json.gz')
    roots = mapping(RUN / 'root-source-hashes.json.gz')
    assert len(sources) == protocol['source_files'] == 10136
    assert len(roots) == 10129
    assert sha(PRIOR / 'source-hashes.json.gz') == protocol['source_mapping_sha256']
    assert all(sha(Path(protocol['source']) / name) == digest for name, digest in sources.items())
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    prepared = read(PRIOR / 'protocol.json')
    assert all(sha(E / name) == digest for name, digest in prepared['inputs_sha256'].items())
    for name in ['binaries.json', 'libraries.json']:
        assert all(sha(row['path']) == row['sha256'] for row in read(PRIOR / name).values())

    commands = read(RUN / 'commands.json')
    assert [row['name'] for row in commands] == COMMANDS
    for row in commands:
        assert row['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(RUN / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    driver = E / 'sphere-normal-image-qualification-driver-v1.stdout'
    events = [json.loads(line) for line in driver.read_text().splitlines()]
    assert [row['start'] for row in events if 'start' in row] == COMMANDS
    assert [row for row in events if 'exit_code' in row] == commands
    assert events[-1] == {'complete': True, 'commands': 12, 'result_sha256': sha(RUN / 'result.json')}
    stderr = E / 'sphere-normal-image-qualification-driver-v1.stderr'
    assert stderr.read_bytes() == b''

    counts = {}
    pattern = re.compile(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; '
                         r'(\d+) measured; (\d+) filtered out;', re.MULTILINE)
    fields = ['passed', 'failed', 'ignored', 'measured', 'filtered_out']
    for name, expected in EXPECTED_COUNTS.items():
        output = (RUN / (name + '.stdout')).read_text()
        rows = [tuple(map(int, row)) for row in pattern.findall(output)]
        assert rows
        count = dict(zip(fields, map(sum, zip(*rows))))
        count['completed_test_summaries'] = len(rows)
        assert (count['passed'], count['ignored'], len(rows)) == expected
        assert count['failed'] == count['measured'] == count['filtered_out'] == 0
        if name in ['default-tests', 'all-features-tests']:
            for test in ['cyclotomic_root_images_reject_only_certified_nonzero_residues',
                         'cyclotomic_root_collisions_and_bad_units_keep_exact_zero_search']:
                assert re.search(r'^test .*::' + test + r' \.\.\. ok$', output, re.MULTILINE)
        counts[name] = count

    closure = {
        'complete': True,
        'commands_all_zero': 12,
        'result_sha256': sha(RUN / 'result.json'),
        'assessor_sha256': sha(__file__),
        'driver_stdout_sha256': sha(driver),
        'driver_stderr_sha256': sha(stderr),
        'test_counts': counts,
        'source_files_rechecked': len(sources),
        'root_source_files_rechecked': len(roots),
        'all_frozen_sources_inputs_artifacts_libraries_and_binaries_rechecked': True,
        'strict_lint_docs_and_fuzz_binary_compilation_passed': True,
        'both_new_normal_form_tests_passed_at_both_feature_settings': True,
        'fuzz_campaign_executed': False,
        'new_formal_proof_executed': False,
        'native_cost_protocol': 'measure_normal_image_native_cost_v1.py',
        'native_resource_protocol': 'measure_normal_image_resources_v1.py',
        'native_cost_or_resource_trial_executed': False,
        'limits': 'Selected four-package Rust qualification, native consumer tests, and fuzz-target compilation. Ignored tests remain ignored. No executed fuzz campaign, ASan, new formal proof, complete workspace CI, push readiness or runtime admission is claimed.',
        'runtime_admitted': False,
        'plan_complete': False,
    }
    DESTINATION.write_text(json.dumps(closure, indent=2) + '\n')
    print(json.dumps({'complete': True, 'test_counts': counts, 'closure_sha256': sha(DESTINATION)}))


if __name__ == '__main__':
    main()
