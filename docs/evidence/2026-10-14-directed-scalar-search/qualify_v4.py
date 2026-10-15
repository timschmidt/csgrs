"""Run the frozen scalar-search prototype qualification without resuming failures."""
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
WORK = ROOT / 'target/directed-scalar-search-v4'
SOURCE = WORK / 'source'
OUT = E / 'qualification-v4'
TRIPLE = 'x86_64-unknown-linux-gnu'
BINARY = WORK / 'sanitizer-target' / TRIPLE / 'release/examples/directed_scalar_probe'
BASE_ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
            'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'RUSTUP_TOOLCHAIN': 'stable',
            'CARGO_TARGET_DIR': str(WORK / 'check-target'), 'RUSTFLAGS': '',
            'PYTHONDONTWRITEBYTECODE': '1', 'RUSTC_BOOTSTRAP': '1'}
ASAN_ENV = BASE_ENV | {'CARGO_TARGET_DIR': str(WORK / 'sanitizer-target'),
                       'RUSTFLAGS': '-Zsanitizer=address -Cforce-frame-pointers=yes',
                       'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1:abort_on_error=0:exitcode=66',
                       'LSAN_OPTIONS': ''}


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
    assert not OUT.exists() and not (WORK / 'check-target').exists() and not (WORK / 'sanitizer-target').exists()
    scope = read(E / 'qualification-scope-v4.json')
    roots = mapping(E / 'main-source-before.json.gz')
    sources = mapping(E / 'source-hashes-v4.json.gz')
    assert inventory(SOURCE) == sources == roots | scope['expected_delta_sha256']
    inputs = {name: sha(E / name) for name in [
        'qualify_v4.py', 'contract-v4.json', 'qualification-scope-v4.json', 'prototype-v4.patch',
        'main-source-before.json.gz', 'source-hashes-v4.json.gz', 'closure-v1.json', 'closure-v2.json', 'closure-v3.json']}
    stdlib = read(PRIOR / 'structural-fuzz-std-dependencies-protocol-v1.json')['installed_source']
    stdlib_table = mapping(PRIOR / 'structural-fuzz-std-dependencies-source-v1.json.gz')
    preserved = [*read(PRIOR / 'structural-fuzz-inventory-v5/binaries.json').values(),
                 *read(E / 'qualification-v1/binaries.json').values(),
                 *read(E / 'qualification-v2/binaries.json').values(),
                 *read(E / 'qualification-v3/binaries.json').values()]
    prior_sources = {v: mapping(E / f'source-hashes-v{v}.json.gz') for v in [1, 2, 3]}
    prior_artifacts = {v: read(E / f'closure-v{v}.json')['trial_artifacts_sha256'] for v in [1, 2, 3]}
    commands, binaries = [], {}

    def guard():
        for version in [1, 2, 3]:
            assert all(sha(ROOT / f'target/directed-scalar-search-v{version}/source' / name) == digest for name, digest in prior_sources[version].items())
            assert all(sha(E / f'qualification-v{version}' / name) == digest for name, digest in prior_artifacts[version].items())
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        for base, table in [(ROOT, roots), (SOURCE, sources), (Path(stdlib), stdlib_table)]:
            assert all(sha(base / name) == digest for name, digest in table.items())
        assert all(sha(row['path']) == row['sha256'] for row in [*preserved, *binaries.values()])

    guard()
    OUT.mkdir()
    dump(OUT / 'protocol.json', {'source': str(SOURCE), 'source_files': len(sources),
                               'inputs_sha256': inputs, 'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
                               'base_environment': BASE_ENV, 'sanitizer_environment': ASAN_ENV,
                               'stdlib_source': stdlib, 'stdlib_source_mapping_sha256': sha(PRIOR / 'structural-fuzz-std-dependencies-source-v1.json.gz'),
                               'preserved_binaries': preserved, 'commands': 13, 'campaign': scope['campaign'],
                               'admitted': False, 'P38_complete': False, 'plan_complete': False})

    def run(name, command, environment=BASE_ENV):
        guard()
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800s', *command],
                                  cwd=SOURCE, env=os.environ | environment, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
               'process_wall_seconds': time.monotonic() - start,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        if name.endswith('-tests'):
            for path in re.findall(r'Running .* \(([^()\n]+)\)', (OUT / (name + '.stderr')).read_text()):
                binaries[name + '/' + Path(path).name] = {'path': path, 'sha256': sha(path), 'bytes': Path(path).stat().st_size}
        if name == 'probe-build' and BINARY.exists():
            binaries[name] = {'path': str(BINARY), 'sha256': sha(BINARY), 'bytes': BINARY.stat().st_size}
        dump(OUT / 'binaries.json', binaries)
        print(json.dumps(row), flush=True)
        guard()
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                                      'artifacts_sha256': inventory(OUT), 'admitted': False, 'P38_complete': False})
            raise SystemExit(1)

    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    run('fuzz-format', ['cargo', 'fmt', '--manifest-path', 'crates/hyperreal/fuzz/Cargo.toml', '--', '--check'])
    run('python-tests', ['python3', '-B', '-m', 'unittest', 'discover', '-s', 'crates/hyperreal/fuzz/directed', '-p', 'test_*.py', '-v'])
    for name, features in [('probe-features-clippy', ['--features', 'serde,dispatch-trace']),
                           ('all-features-clippy', ['--all-features'])]:
        run(name, ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '--all-targets', *features, '--', '-D', 'warnings'])
    for name, features in [('probe-features-tests', ['--features', 'serde,dispatch-trace']),
                           ('all-features-tests', ['--all-features'])]:
        run(name, ['cargo', 'test', '--locked', '--release', '-Zbuild-std', '--target', TRIPLE,
                   '-p', 'hyperreal', '--example', 'directed_scalar_probe', *features,
                   '--', '--test-threads=2', '--show-output'], ASAN_ENV)
    run('probe-build', ['cargo', 'build', '--locked', '--release', '-Zbuild-std', '--target', TRIPLE,
                         '-p', 'hyperreal', '--example', 'directed_scalar_probe', '--features', 'serde,dispatch-trace', '--verbose'], ASAN_ENV)
    run('inventory', [str(BINARY), '--list'], ASAN_ENV)
    listing = read(OUT / 'inventory.stdout')
    assert len(listing['representations']) == 22 and all(row['name'] == row['actual_class'] for row in listing['representations'])
    regression = SOURCE / 'crates/hyperreal/examples/directed_scalar_probe/regressions/mixed_number_v2.input.json'
    run('mixed-number-regression', [str(BINARY), str(regression)], ASAN_ENV)
    run('mixed-number-wire-validation', ['python3', '-B', 'crates/hyperreal/fuzz/directed/evidence.py',
         '--input', str(regression), '--response', str(OUT / 'mixed-number-regression.stdout'),
         '--inventory', str(OUT / 'inventory.stdout')], ASAN_ENV)
    campaign = OUT / 'campaign'
    run('campaign', ['python3', '-B', 'crates/hyperreal/fuzz/directed/campaign.py', 'run',
                     '--binary', str(BINARY), '--output', str(campaign), '--seed', '20261014',
                     '--rounds', '3', '--population', '8', '--parents', '4', '--timeout', '30'], ASAN_ENV)
    run('campaign-replay', ['python3', '-B', 'crates/hyperreal/fuzz/directed/campaign.py', 'verify', str(campaign)], ASAN_ENV)
    result = read(campaign / 'result.json')
    assert result['complete'] and result['completed_probes'] == 528 and result['selection_events'] == 264
    assert inventory(SOURCE) == sources
    guard()
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': 13, 'campaign_completed_probes': 528,
                             'selection_events': 264, 'all_selection_and_mutation_traces_replayed': True,
                             'artifacts_sha256': inventory(OUT), 'limits': scope['limits'],
                             'competitive_benchmark_complete': False, 'admitted': False,
                             'P38_complete': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
