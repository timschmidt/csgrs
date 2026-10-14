"""Broader Rust qualification of the unchanged native feasibility candidate."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PRIOR = E / 'sphere-normal-image-v1'
OUT = E / 'sphere-normal-image-qualification-v1'
TARGET = ROOT / 'target/sphere-normal-image-qualification-v1'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}
PACKAGES = ['hyperreal', 'hyperlimit', 'hypermesh', 'hypercurve']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not OUT.exists() and not TARGET.exists()
    result = json.loads((PRIOR / 'result.json').read_text())
    assert result['complete'] and result['commands_all_zero'] == 10
    assert result['targeted_scalar_tests_passed'] == 14 and result['all_twelve_full_original_native_outputs_preserved']
    assert all(sha(PRIOR / p) == h for p, h in result['artifacts_sha256'].items())
    support = E / 'sphere-stable-support-v1'
    proved = json.loads((support / 'result.json').read_text())
    assert proved['complete'] and sum(c['face_point_pairs'] for c in proved['cases']) == 612288
    assert all(sha(support / p) == h for p, h in proved['artifacts_sha256'].items())
    prep = json.loads((PRIOR / 'protocol.json').read_text())
    source = Path(prep['source'])
    mapping = json.loads(gzip.decompress((PRIOR / 'source-hashes.json.gz').read_bytes()))
    baseline_mapping = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    root_mapping = {p: sha(ROOT / p) for p in baseline_mapping}
    assert all(root_mapping[p] == h for p, h in baseline_mapping.items() if p != 'docs/properties-status.json')
    inputs = {p: sha(E / p) for p in ['qualify_sphere_normal_image_v1.py', 'sphere-normal-image-qualification-scope-v1.json',
        'sphere-normal-image-feasibility-closure-v1.json', 'sphere-stable-support-v1/result.json',
        'normal-image-scalar-cost-assessment-v1.json', 'normal-image-scalar-cost-closure-v1.json']}
    scope = json.loads((E / 'sphere-normal-image-qualification-scope-v1.json').read_text())
    assert sha(PRIOR / 'result.json') == scope['feasibility_result_sha256']
    assert sha(support / 'result.json') == scope['independent_support_result_sha256']
    cost = json.loads((E / 'normal-image-scalar-cost-assessment-v1.json').read_text())
    assert cost['complete'] and cost['commands_all_zero'] == 35
    assert sha(E / 'normal-image-scalar-cost-assessment-v1.json') == scope['scalar_cost_assessment_sha256']
    frozen_binaries = json.loads((PRIOR / 'binaries.json').read_text())
    frozen_libraries = json.loads((PRIOR / 'libraries.json').read_text())
    OUT.mkdir()
    (OUT / 'root-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(root_mapping, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'source': str(source), 'source_mapping_sha256': sha(PRIOR / 'source-hashes.json.gz'),
        'source_files': len(mapping), 'environment': ENV, 'packages': PACKAGES, 'inputs_sha256': inputs,
        'command_wall_limit_seconds': 1800, 'runtime_admitted': False, 'plan_complete': False})

    def guard():
        assert all(sha(source / p) == h for p, h in mapping.items())
        assert all(sha(ROOT / p) == h for p, h in root_mapping.items())
        assert all(sha(E / p) == h for p, h in inputs.items())
        assert all(sha(r['path']) == r['sha256'] for r in frozen_binaries.values())
        assert all(sha(r['path']) == r['sha256'] for r in frozen_libraries.values())

    commands = []

    def run(name, command, extra=None):
        guard()
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800s', *command],
                cwd=source, env=os.environ | ENV | (extra or {}), stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
               'process_wall_seconds': time.monotonic() - started,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        guard()
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                'all_frozen_sources_unchanged': True, 'runtime_admitted': False, 'plan_complete': False})
            raise SystemExit(code)

    package_args = [arg for package in PACKAGES for arg in ['-p', package]]
    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    for name, features in [('default', []), ('all-features', ['--all-features'])]:
        run(name + '-clippy', ['cargo', 'clippy', '--locked', '--release', *package_args,
            '--all-targets', *features, '--', '-D', 'warnings'])
        run(name + '-tests', ['cargo', 'test', '--locked', '--release', *package_args,
            '--lib', '--tests', *features, '--', '--test-threads=2', '--show-output'])
    run('doctests', ['cargo', 'test', '--locked', '--release', *package_args, '--all-features', '--doc'])
    run('docs', ['cargo', 'doc', '--locked', '--release', *package_args, '--all-features', '--no-deps'],
        {'RUSTDOCFLAGS': '-D warnings'})
    native_tests = ['--test', 'native_queries', '--test', 'geometry_context', '--test', 'solid_cache',
                    '--test', 'native_hull_determinism']
    run('grammar-native-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'csgrs',
        '--no-default-features', *native_tests, '--', '-D', 'warnings'])
    run('grammar-native-consumers', ['cargo', 'test', '--locked', '--release', '-p', 'csgrs',
        '--no-default-features', *native_tests, '--', '--test-threads=2', '--show-output'])
    for package in ['hyperreal', 'hyperlimit', 'hypermesh']:
        run(package + '-fuzz-check', ['cargo', 'check', '--locked', '--release',
            '--manifest-path', f'crates/{package}/fuzz/Cargo.toml', '--bins'])
    guard()
    assert len(commands) == scope['commands'] == 12
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': len(commands),
        'source_files_rechecked': len(mapping), 'all_frozen_sources_unchanged': True,
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'limits': 'Broad selected Rust tests, lint, docs and fuzz-target compilation. No executed fuzz campaign, formal proof, native cost admission, complete workspace CI or push qualification is claimed.',
        'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'commands': len(commands), 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
