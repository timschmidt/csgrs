"""Feature and consumer qualification of the frozen deterministic hull candidate."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-stable-selection-qualification-v1'
SOURCE = ROOT / 'target/sphere-stable-selection-v1/source'
TARGET = ROOT / 'target/sphere-stable-selection-qualification-v1'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}
CONSUMERS = ['native_queries', 'geometry_context', 'solid_cache', 'native_hull_determinism']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def counts(raw):
    rows = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', raw)
    assert rows and 'test result: FAILED' not in raw
    keys = ['passed', 'failed', 'ignored', 'measured', 'filtered_out']
    totals = {key: sum(int(row[i]) for row in rows) for i, key in enumerate(keys)}
    assert totals['failed'] == totals['measured'] == totals['filtered_out'] == 0
    return totals | {'completed_test_summaries': len(rows)}


def main():
    assert not OUT.exists() and not TARGET.exists()
    scope = json.loads((E / 'sphere-stable-selection-qualification-scope-v1.json').read_text())
    assert sha(E / 'sphere-stable-selection-v1/result.json') == scope['feasibility_result_sha256']
    assert sha(E / 'sphere-stable-support-v1/result.json') == scope['independent_support_result_sha256']
    assert sha(E / 'sphere-stable-selection-closure-v1.json') == scope['closure_sha256']
    inputs = {name: sha(E / name) for name in [
        'qualify_sphere_stable_selection_v1.py', 'sphere-stable-selection-qualification-scope-v1.json',
        'sphere-stable-selection-closure-v1.json', 'sphere-stable-selection-v1/source-hashes.json.gz',
        'sphere-cyclotomic-image-v1/source-hashes.json.gz', 'mesh-identity-preparation-v1/source-hashes.json.gz',
        'sphere-cyclotomic-image-qualification-assessment-v1.json']}
    for directory in ['sphere-stable-selection-v1', 'sphere-stable-support-v1']:
        result = json.loads((E / directory / 'result.json').read_text())
        assert result['complete']
        inputs[directory + '/result.json'] = sha(E / directory / 'result.json')
        for name, digest in result['artifacts_sha256'].items():
            assert sha(E / directory / name) == digest
            inputs[directory + '/' + name] = digest
    mapping = json.loads(gzip.decompress((E / 'sphere-stable-selection-v1/source-hashes.json.gz').read_bytes()))
    parent_mapping = json.loads(gzip.decompress((E / 'sphere-cyclotomic-image-v1/source-hashes.json.gz').read_bytes()))
    baseline = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    assert len(mapping) == 10136
    delta = {name: digest for name, digest in mapping.items() if parent_mapping.get(name) != digest}
    assert set(delta) == {'crates/hypermesh/src/convex_hull.rs', 'tests/native_hull_determinism.rs'}
    assert set(mapping) == set(parent_mapping) | {'tests/native_hull_determinism.rs'}
    root_mapping = {name: sha(ROOT / name) for name in baseline}
    assert all(root_mapping[name] == digest for name, digest in baseline.items() if name != 'docs/properties-status.json')
    feasibility = json.loads((E / 'sphere-stable-selection-v1/protocol.json').read_text())
    libraries = json.loads((E / 'sphere-stable-selection-v1/libraries.json').read_text())
    binaries = json.loads((E / 'sphere-stable-selection-v1/binaries.json').read_text())

    def guard(full=False):
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        assert all(sha(SOURCE / name) == digest for name, digest in (mapping if full else delta).items())
        for records in [libraries, binaries, feasibility['parent_libraries']]:
            assert all(sha(row['path']) == row['sha256'] for row in records.values())
        if full:
            assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
            assert all(sha(Path(feasibility['parent_source']) / name) == digest for name, digest in parent_mapping.items())
            assert {str(path.relative_to(SOURCE)) for path in SOURCE.rglob('*') if path.is_file()} == set(mapping)

    guard(full=True)
    OUT.mkdir()
    (OUT / 'root-source-hashes.json.gz').write_bytes(gzip.compress(
        (json.dumps(root_mapping, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'source': str(SOURCE),
        'target': str(TARGET), 'environment': ENV, 'source_files': len(mapping),
        'root_source_files': len(root_mapping), 'delta_from_qualified_image_sha256': delta,
        'scope': scope, 'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
        'runtime_admitted': False, 'plan_complete': False})
    commands, test_counts = [], {}

    def run(name, command, extra=None, test=False):
        guard()
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
                cwd=SOURCE, env=os.environ | ENV | (extra or {}), stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
            'process_wall_seconds': time.monotonic() - start,
            'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': code}), flush=True)
        guard()
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                'runtime_admitted': False, 'plan_complete': False})
            raise SystemExit(code)
        if test:
            test_counts[name] = counts((OUT / (name + '.stdout')).read_text())
            dump(OUT / 'test-counts.json', test_counts)

    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    for selection, features in [('default', []), ('all-features', ['--all-features'])]:
        run(selection + '-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypermesh',
            '--all-targets', *features, '--', '-D', 'warnings'])
        run(selection + '-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypermesh',
            '--lib', '--tests', *features, '--', '--test-threads=2', '--show-output'], test=True)
    run('doctests', ['cargo', 'test', '--locked', '--release', '-p', 'hypermesh', '--all-features', '--doc'], test=True)
    run('docs', ['cargo', 'doc', '--locked', '--release', '-p', 'hypermesh', '--all-features', '--no-deps'],
        extra={'RUSTDOCFLAGS': '-D warnings'})
    selected_tests = [argument for name in CONSUMERS for argument in ['--test', name]]
    run('native-consumer-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'csgrs',
        '--no-default-features', *selected_tests, '--', '-D', 'warnings'])
    run('native-consumer-tests', ['cargo', 'test', '--locked', '--release', '-p', 'csgrs',
        '--no-default-features', *selected_tests, '--', '--test-threads=1', '--show-output'], test=True)
    run('mesh-fuzz-check', ['cargo', 'check', '--locked', '--release', '--manifest-path',
        str(SOURCE / 'crates/hypermesh/fuzz/Cargo.toml'), '--bins'])
    assert len(commands) == 10
    assert test_counts['default-tests']['passed'] == 234
    assert test_counts['all-features-tests']['passed'] == 235
    assert test_counts['native-consumer-tests']['passed'] == 19
    guard(full=True)
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': 10,
        'test_counts': test_counts, 'all_frozen_sources_inputs_and_native_artifacts_unchanged': True,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'coverage': scope['coverage'], 'limits': scope['limitations'],
        'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
