"""Qualify the frozen enclosure adapters before the complete native comparison."""
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
PRIOR = E.parent / '2026-10-14-sampled-sphere-hull'
WORK = ROOT / 'target/directed-scalar-bench-v2'
SOURCE = WORK / 'source'
OUT = E / 'competitive-qualification-v2'
CAMPAIGN = E / 'qualification-v4/campaign'
TRIPLE = 'x86_64-unknown-linux-gnu'
BINARY = WORK / 'native-target/release/examples/directed_scalar_bench'
BASE_ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
            'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'RUSTUP_TOOLCHAIN': 'stable',
            'CARGO_TARGET_DIR': str(WORK / 'check-target'), 'RUSTFLAGS': '',
            'PYTHONDONTWRITEBYTECODE': '1', 'RUSTC_BOOTSTRAP': '1', 'ASAN_OPTIONS': '', 'LSAN_OPTIONS': ''}
ASAN_ENV = BASE_ENV | {'CARGO_TARGET_DIR': str(WORK / 'sanitizer-target'),
                       'RUSTFLAGS': '-Zsanitizer=address -Cforce-frame-pointers=yes',
                       'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1:abort_on_error=0:exitcode=66'}
NATIVE_ENV = BASE_ENV | {'CARGO_TARGET_DIR': str(WORK / 'native-target')}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def mapping(path):
    return json.loads(gzip.decompress(path.read_bytes()))


def inventory(base):
    return {str(p.relative_to(base)): sha(p) for p in base.rglob('*') if p.is_file()}


def main():
    assert not OUT.exists() and all(not (WORK / name).exists() for name in ['check-target', 'sanitizer-target', 'native-target'])
    scope = read(E / 'competitive-scope-v2.json')
    roots = mapping(E / 'main-source-before.json.gz')
    sources = mapping(E / 'competitive-source-v2.json.gz')
    assert inventory(SOURCE) == sources == roots | scope['expected_delta_sha256']
    assert read(E / 'qualification-v4/result.json')['complete']
    assert sha(CAMPAIGN / 'selected-inputs.json') == sha(E / 'competitive-inputs-v2.json') == scope['selected_inputs_sha256']
    assert sha(E / 'qualification-v4/result.json') == scope['qualified_search_result_sha256']
    inputs = {name: sha(E / name) for name in [
        'qualify_competitive_v2.py', 'competitive-contract-v2.json', 'competitive-scope-v2.json',
        'competitive-prototype-v2.patch', 'competitive-source-v2.json.gz', 'competitive-inputs-v2.json',
        'main-source-before.json.gz', 'closure-v1.json', 'closure-v2.json', 'closure-v3.json', 'closure-v4.json']}
    old_sources = {ROOT / f'target/directed-scalar-search-v{v}/source': mapping(E / f'source-hashes-v{v}.json.gz') for v in range(1, 5)}
    old_sources[ROOT / 'target/directed-scalar-bench-v1/source'] = mapping(E / 'competitive-unexecuted-source-v1.json.gz')
    old_artifacts = {E / f'qualification-v{v}': read(E / f'closure-v{v}.json')['trial_artifacts_sha256'] for v in range(1, 5)}
    preserved = list(read(PRIOR / 'structural-fuzz-inventory-v5/binaries.json').values())
    for version in range(1, 5):
        preserved.extend(read(E / f'qualification-v{version}/binaries.json').values())
    stdlib = Path(read(PRIOR / 'structural-fuzz-std-dependencies-protocol-v1.json')['installed_source'])
    stdlib_table = mapping(PRIOR / 'structural-fuzz-std-dependencies-source-v1.json.gz')
    commands, binaries = [], {}

    def guard():
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        for base, table in [(ROOT, roots), (SOURCE, sources), (stdlib, stdlib_table), *old_sources.items(), *old_artifacts.items()]:
            assert all(sha(base / name) == digest for name, digest in table.items())
        assert all(sha(row['path']) == row['sha256'] for row in [*preserved, *binaries.values()])

    guard()
    OUT.mkdir()
    dump(OUT / 'protocol.json', {'source': str(SOURCE), 'source_files': len(sources), 'inputs_sha256': inputs,
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True), 'base_environment': BASE_ENV,
        'sanitizer_environment': ASAN_ENV, 'native_environment': NATIVE_ENV, 'stdlib_source': str(stdlib),
        'stdlib_source_mapping_sha256': sha(PRIOR / 'structural-fuzz-std-dependencies-source-v1.json.gz'),
        'preserved_binaries': preserved, 'commands': 13, 'selected_inputs': 59, 'expected_native_samples': 1298,
        'admitted': False, 'P38_complete': False, 'plan_complete': False})

    def run(name, command, environment=BASE_ENV):
        guard()
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800s', *command],
                                  cwd=SOURCE, env=os.environ | environment, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code, 'process_wall_seconds': time.monotonic() - start,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        if name.endswith('-tests'):
            for path in re.findall(r'Running .* \(([^()\n]+)\)', (OUT / (name + '.stderr')).read_text()):
                binaries[name + '/' + Path(path).name] = {'path': path, 'sha256': sha(path), 'bytes': Path(path).stat().st_size}
        if name == 'native-build' and BINARY.exists():
            binaries[name] = {'path': str(BINARY), 'sha256': sha(BINARY), 'bytes': BINARY.stat().st_size}
        dump(OUT / 'binaries.json', binaries)
        print(json.dumps(row), flush=True)
        guard()
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row, 'artifacts_sha256': inventory(OUT),
                                      'admitted': False, 'P38_complete': False, 'plan_complete': False})
            raise SystemExit(1)

    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    run('fuzz-format', ['cargo', 'fmt', '--manifest-path', 'crates/hyperreal/fuzz/Cargo.toml', '--', '--check'])
    run('python-tests', ['python3', '-B', '-m', 'unittest', 'discover', '-s', 'crates/hyperreal/fuzz/directed', '-p', 'test_*.py', '-v'])
    for name, features in [('serde-clippy', ['--features', 'serde']),
                           ('trace-clippy', ['--features', 'serde,dispatch-trace']),
                           ('all-features-clippy', ['--all-features'])]:
        run(name, ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-targets', *features, '--', '-D', 'warnings'])
    for name, features in [('serde-tests', ['--features', 'serde']), ('all-features-tests', ['--all-features'])]:
        run(name, ['cargo', 'test', '--locked', '--release', '-Zbuild-std', '--target', TRIPLE,
                   '-p', 'hyperreal', '--example', 'directed_scalar_bench', *features,
                   '--', '--test-threads=2', '--show-output'], ASAN_ENV)
    run('native-build', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal',
                         '--example', 'directed_scalar_bench', '--features', 'serde', '--verbose'], NATIVE_ENV)
    run('native-symbols', ['nm', '--defined-only', str(BINARY)], NATIVE_ENV)
    assert not re.search(r'\b__(?:a|l)san_', (OUT / 'native-symbols.stdout').read_text())
    comparison = OUT / 'comparison'
    run('comparison', ['python3', '-B', 'crates/hyperreal/fuzz/directed/competitive.py', 'run',
         '--binary', str(BINARY), '--campaign', str(CAMPAIGN), '--output', str(comparison),
         '--seed', '20261014', '--samples', '11', '--timeout', '30'], NATIVE_ENV)
    run('comparison-replay', ['python3', '-B', 'crates/hyperreal/fuzz/directed/competitive.py', 'verify', str(comparison)], NATIVE_ENV)
    run('native-size', ['size', str(BINARY)], NATIVE_ENV)
    result = read(comparison / 'result.json')
    assert result['complete'] and result['input_count'] == 59 and result['completed_samples'] == 1298
    assert inventory(SOURCE) == sources
    guard()
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': 13, 'native_inputs': 59,
        'native_samples': 1298, 'samples_per_backend_per_input': 11, 'representation_buckets': 22,
        'all_numeric_and_timing_evidence_replayed': True, 'native_sanitizer_symbols_absent': True,
        'artifacts_sha256': inventory(OUT), 'limits': scope['limits'], 'admitted': False,
        'interval_comparison_complete': True, 'P38_complete': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
