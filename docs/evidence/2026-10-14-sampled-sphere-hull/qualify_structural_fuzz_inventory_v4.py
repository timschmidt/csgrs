"""Freeze and execute one bounded qualification of the shared scalar fuzz inventory."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'structural-fuzz-inventory-v4'
WORK = ROOT / 'target/structural-fuzz-inventory-v4'
SOURCE = WORK / 'source'
TARGET = 'x86_64-unknown-linux-gnu'
FUZZ_BINARY = WORK / 'fuzz-target' / TARGET / 'release/structural_representations'
SEED_DIR = Path('crates/hyperreal/fuzz/corpus/structural_representations')
ENV = {
    'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
    'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
    'CARGO_TARGET_DIR': str(WORK / 'qualification-target'),
    'RUSTUP_TOOLCHAIN': 'stable', 'RUSTC_BOOTSTRAP': '1',
    'RUSTFLAGS': '-Cforce-frame-pointers=yes',
    'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1:abort_on_error=0:exitcode=66',
    'LSAN_OPTIONS': '',
}


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


def prepare():
    assert not OUT.exists()
    for name in ['qualification-target', 'fuzz-target', 'mutation-corpus', 'artifacts']:
        assert not (WORK / name).exists()
    scope = read(E / 'structural-fuzz-inventory-scope-v4.json')
    admission = read(E / 'mpfr-test-cleanup-main-admission-v1/admission.json')
    assert admission['complete'] and admission['test_cleanup_admitted']
    parent = mapping(E / 'mpfr-test-cleanup-main-v1/source-hashes.json.gz')
    actual = inventory(SOURCE)
    assert actual == parent | scope['expected_delta_sha256']
    assert len(actual) == scope['source_files'] == 10153
    original = (ROOT / scope['parent_source'] / 'crates/hyperreal/tests/real_representations.rs').read_text()
    recipe = original[original.index('struct RepresentationCase {'):original.index('fn structural_kind_index(')].rstrip()
    shared = (SOURCE / 'crates/hyperreal/tests/support/representations.rs').read_text()
    shared = shared[shared.index('pub(crate) struct RepresentationCase {'):].rstrip()
    assert shared.replace('pub(crate) ', '') == recipe
    pairs = {(left, (left + stride) % 22) for stride in range(22) for left in range(22)}
    assert pairs == {(left, right) for left in range(22) for right in range(22)}
    root_authority = mapping(E / 'mpfr-test-cleanup-main-admission-v1/source-hashes-after.json.gz')
    roots = {name: sha(ROOT / name) for name in root_authority}
    assert all(roots[name] == digest for name, digest in root_authority.items()
               if name != 'docs/properties-status.json')
    assert len(roots) == 10130
    seeds = inventory(SOURCE / SEED_DIR)
    assert set(seeds) == set(scope['original_seed_sha256']) | {f'stride-{i:02}' for i in range(22)}
    assert len(seeds) == 43
    assert all(seeds[name] == digest for name, digest in scope['original_seed_sha256'].items())
    assert all((SOURCE / name).read_bytes() == bytes.fromhex(value)
               for name, value in scope['new_seed_hex'].items())
    assert all((SOURCE / SEED_DIR / name).stat().st_size <= 34 for name in seeds)
    prior = read(E / 'mpfr-test-cleanup-main-v1/protocol.json')
    inputs = {name: sha(E / name) for name in [
        'qualify_structural_fuzz_inventory_v4.py', 'structural-fuzz-inventory-scope-v4.json',
        'structural-fuzz-inventory-v4.patch', 'structural-fuzz-inventory-preparation-failure-v1.json',
        'structural-fuzz-inventory-preparation-failure-v2.json', 'structural-fuzz-inventory-closure-v3.json',
        'mpfr-test-cleanup-main-admission-v1/admission.json',
        'mpfr-test-cleanup-main-admission-v1/source-hashes-after.json.gz',
        'mpfr-test-cleanup-main-v1/source-hashes.json.gz', 'mpfr-test-cleanup-main-v1/result.json',
    ]}
    env = os.environ | ENV
    compiler = subprocess.check_output(['rustc', '-Vv'], env=env, text=True)
    assert compiler == prior['compiler']
    cargo_fuzz = shutil.which('cargo-fuzz')
    assert cargo_fuzz
    OUT.mkdir()
    for name, table in [('source-hashes.json.gz', actual), ('root-source-hashes.json.gz', roots)]:
        (OUT / name).write_bytes(gzip.compress((json.dumps(table, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {
        'source': str(SOURCE), 'source_files': len(actual), 'root_source_files': len(roots),
        'inputs_sha256': inputs, 'seed_sha256': seeds, 'compiler': compiler, 'environment': ENV,
        'cargo_fuzz': {'path': cargo_fuzz, 'sha256': sha(cargo_fuzz),
                      'version': subprocess.check_output(['cargo', 'fuzz', '--version'], env=env, text=True)},
        'preserved_binaries': list(read(E / 'mpfr-test-cleanup-main-v1/binaries.json').values()),
        'command_wall_limit_seconds': 1800, 'commands': 10,
        'test_only': True, 'admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'prepared': True, 'source_files': len(actual), 'retained_seeds': len(seeds)}))


def qualify():
    assert OUT.exists() and not (OUT / 'commands.json').exists()
    assert not (OUT / 'failure.json').exists() and not (OUT / 'result.json').exists()
    protocol = read(OUT / 'protocol.json')
    scope = read(E / 'structural-fuzz-inventory-scope-v4.json')
    source_table = mapping(OUT / 'source-hashes.json.gz')
    root_table = mapping(OUT / 'root-source-hashes.json.gz')
    seeds = sorted(protocol['seed_sha256'])
    commands, binaries = [], {}
    env = os.environ | ENV
    assert 'CARGO_ENCODED_RUSTFLAGS' not in env
    assert subprocess.check_output(['rustc', '-Vv'], env=env, text=True) == protocol['compiler']

    def guard():
        assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
        for base, table in [(ROOT, root_table), (SOURCE, source_table)]:
            assert all(sha(base / name) == digest for name, digest in table.items())
        assert sha(protocol['cargo_fuzz']['path']) == protocol['cargo_fuzz']['sha256']
        assert all(sha(row['path']) == row['sha256']
                   for row in [*protocol['preserved_binaries'], *binaries.values()])

    def run(name, command):
        guard()
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800s', *command],
                                  cwd=SOURCE, env=env, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
               'process_wall_seconds': time.monotonic() - start,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        paths = re.findall(r'Running .* \(([^()\n]+)\)', (OUT / (name + '.stderr')).read_text()) if name.endswith('-tests') else []
        if name == 'fuzz-build' and FUZZ_BINARY.exists():
            paths.append(str(FUZZ_BINARY))
        for path in paths:
            binaries[name + '/' + Path(path).name] = {'path': path, 'sha256': sha(path), 'bytes': Path(path).stat().st_size}
        dump(OUT / 'binaries.json', binaries)
        print(json.dumps(row), flush=True)
        guard()
        if code:
            for directory in ['mutation-corpus', 'artifacts']:
                if (WORK / directory).exists():
                    shutil.copytree(WORK / directory, OUT / ('retained-failure-' + directory))
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                                      'retained_artifacts_sha256': inventory(OUT),
                                      'admitted': False, 'plan_complete': False})
            raise SystemExit(1)

    guard()
    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    run('fuzz-format', ['cargo', 'fmt', '--manifest-path', 'crates/hyperreal/fuzz/Cargo.toml', '--', '--check'])
    for name, features in [('default-clippy', []), ('all-features-clippy', ['--all-features'])]:
        run(name, ['cargo', 'clippy', '--locked', '--release', '--target', TARGET,
                   '-p', 'hyperreal', '--all-targets', *features, '--', '-D', 'warnings'])
    run('fuzz-clippy', ['cargo', 'clippy', '--locked', '--release', '--target', TARGET,
                       '--manifest-path', 'crates/hyperreal/fuzz/Cargo.toml', '--bins', '--all-features', '--', '-D', 'warnings'])
    counts = {}
    for name, features in [('default-tests', []), ('all-features-tests', ['--all-features'])]:
        run(name, ['cargo', 'test', '--locked', '--release', '--target', TARGET,
                   '-p', 'hyperreal', '--test', 'real_representations', *features,
                   '--', '--test-threads=2', '--show-output'])
        rows = [tuple(map(int, row)) for row in re.findall(
            r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;',
            (OUT / (name + '.stdout')).read_text(), re.MULTILINE)]
        assert len(rows) == 1 and rows[0][1:] == (0, 0, 0, 0)
        counts[name] = rows[0][0]
    run('fuzz-build', ['cargo', 'fuzz', 'build', 'structural_representations',
                       '--fuzz-dir', 'crates/hyperreal/fuzz', '--target-dir', str(WORK / 'fuzz-target'),
                       '--target', TARGET, '--sanitizer', 'address', '--build-std', '--debug-assertions', '--verbose'])
    build_log = (OUT / 'fuzz-build.stderr').read_text()
    for crate in ['std', 'hyperreal', 'structural_representations']:
        assert any('--crate-name ' + crate + ' ' in line and 'sanitizer=address' in line
                   and 'sanitizer-coverage-inline-8bit-counters' in line for line in build_log.splitlines())
    fuzz_options = ['-timeout=120', '-rss_limit_mb=8192', '-max_len=34', '-print_final_stats=1']
    run('seed-replay', [str(FUZZ_BINARY), '-runs=1', *fuzz_options,
                        *[str(SOURCE / SEED_DIR / name) for name in seeds]])
    replay = (OUT / 'seed-replay.stderr').read_text()
    assert 'Running 43 inputs 1 time(s) each.' in replay
    assert re.findall(r'^Executed (.+) in \d+ ms$', replay, re.MULTILINE) == [str(SOURCE / SEED_DIR / name) for name in seeds]
    corpus = WORK / 'mutation-corpus'
    artifacts = WORK / 'artifacts'
    assert not corpus.exists() and not artifacts.exists()
    shutil.copytree(SOURCE / SEED_DIR, corpus)
    artifacts.mkdir()
    run('bounded-mutations', [str(FUZZ_BINARY), '-runs=256', '-seed=20261014', *fuzz_options,
                              '-artifact_prefix=' + str(artifacts) + '/', str(corpus)])
    mutation = (OUT / 'bounded-mutations.stderr').read_text()
    assert re.search(r'^Done 256 runs in \d+ second\(s\)$', mutation, re.MULTILINE)
    assert 'inline 8-bit counters' in mutation and 'cov:' in mutation
    for name in ['seed-replay', 'bounded-mutations']:
        text = (OUT / (name + '.stderr')).read_text()
        assert not any(marker in text for marker in ['ERROR: AddressSanitizer:', 'ERROR: LeakSanitizer:', 'ERROR: libFuzzer:'])
    assert not list(artifacts.iterdir())
    assert all(sha(corpus / name) == digest for name, digest in protocol['seed_sha256'].items())
    dump(OUT / 'mutation-corpus.json', {'files_sha256': inventory(corpus),
                                     'new_files': sorted(set(inventory(corpus)) - set(seeds))})
    shutil.copytree(corpus, OUT / 'retained-mutation-corpus')
    assert inventory(SOURCE) == source_table
    guard()
    assert len(commands) == 10 and all(row['exit_code'] == 0 for row in commands)
    dump(OUT / 'result.json', {
        'complete': True, 'commands_all_zero': 10, 'test_counts': counts,
        'representatives': 22, 'fixed_replayed_seeds': 43,
        'retained_stride_schedule_ordered_pairs': 484, 'bounded_mutation_runs': 256,
        'instrumented_standard_library_build_confirmed': True,
        'all_frozen_sources_inputs_and_binaries_rechecked': True,
        'artifacts_sha256': inventory(OUT), 'limits': scope['limits'],
        'admitted': False, 'directed_lexicase_campaign_complete': False, 'plan_complete': False,
    })
    print(json.dumps({'complete': True, 'test_counts': counts, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('phase', choices=['prepare', 'qualify'])
    args = parser.parse_args()
    {'prepare': prepare, 'qualify': qualify}[args.phase]()
