"""Run fresh downstream checks for the empty-composition repair, serially."""
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tarfile
import time

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('local_validation', E / 'validate_v1.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)
ROOT = v.ROOT


def sources():
    files = v.sources()
    names = subprocess.check_output([
        'git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard',
        '--', 'tests/ffi', 'tests/wasm', 'bindings', 'scripts/test-wasm-bindings.sh',
        'crates/csgrs-ffi/include', 'readme.md', 'docs/csg.png',
    ], cwd=ROOT).decode().split('\0')
    files.update({name: v.sha(ROOT / name) for name in names if name})
    files.update({str(path.relative_to(ROOT)): v.sha(path) for path in [
        Path(__file__), E / 'validate_v1.py',
    ]})
    return dict(sorted(files.items()))


def main():
    phase = sys.argv[1]
    out = E / phase
    target = ROOT / 'target/empty-composition' / phase
    lib = target / 'release'
    c_binary = target / 'empty-c-consumer'
    cpp_binary = target / 'empty-cpp-consumer'
    native = [
        ('mesh-output-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypermesh', '--lib', 'output::tests::', '--', '--test-threads=1']),
        ('mesh-boolean-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hypermesh', '--test', 'boolean', '--', '--test-threads=1']),
        ('csgrs-all-feature-tests', ['cargo', 'test', '--locked', '--release', '-p', 'csgrs', '--all-features', '--lib', '--tests', '--', '--test-threads=2']),
        ('ffi-tests', ['cargo', 'test', '--locked', '--release', '-p', 'csgrs-ffi']),
        ('all-feature-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'csgrs', '-p', 'hypermesh', '-p', 'csgrs-ffi', '--all-targets', '--all-features', '--', '-D', 'warnings']),
        ('strict-docs', ['cargo', 'doc', '--locked', '-p', 'csgrs', '-p', 'hypermesh', '-p', 'csgrs-ffi', '--all-features', '--no-deps']),
    ]
    bindings = [
        ('ffi-build', ['cargo', 'build', '--locked', '--release', '-p', 'csgrs-ffi']),
        ('c-build', ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', 'crates/csgrs-ffi/include', 'tests/ffi/empty_composition.c', '-L', str(lib), '-Wl,-rpath,' + str(lib), '-lcsgrs_ffi', '-o', str(c_binary)]),
        ('c-consumer', [str(c_binary)]),
        ('cpp-build', ['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', 'bindings/cpp', 'tests/ffi/empty_composition.cpp', '-L', str(lib), '-Wl,-rpath,' + str(lib), '-lcsgrs_ffi', '-o', str(cpp_binary)]),
        ('cpp-consumer', [str(cpp_binary)]),
        ('python-consumer', ['python3', 'tests/ffi/empty_composition.py']),
        ('wasm-consumers', ['bash', 'scripts/test-wasm-bindings.sh']),
    ]
    commands = {'native-v1': native, 'bindings-v1': bindings}[phase]
    assert json.loads((E / 'candidate-v1/result.json').read_text())['passed']
    out.mkdir(exist_ok=False)
    assert not target.exists()
    initial = sources()
    preserved = {}
    for prior in ['baseline-v1', 'candidate-v1']:
        preserved.update(json.loads((E / prior / 'terminal-observation.json').read_text())['executables_sha256'])
    with gzip.open(out / 'source-hashes.json.gz', 'wt') as f:
        json.dump(initial, f, sort_keys=True)
    (out / 'tracked.patch').write_bytes(subprocess.check_output([
        'git', 'diff', '--binary', 'HEAD', '--', 'src', 'tests', 'crates', 'bindings', 'scripts',
        'Cargo.toml', 'Cargo.lock', '.cargo/config.toml', '.rustfmt.toml', 'AGENTS.md', 'readme.md',
    ], cwd=ROOT))
    untracked = subprocess.check_output(['git', 'ls-files', '-z', '--others', '--exclude-standard'], cwd=ROOT).decode().split('\0')
    with tarfile.open(out / 'untracked-sources.tar.gz', 'w:gz') as archive:
        for name in sorted(set(untracked) & initial.keys()):
            archive.add(ROOT / name, arcname=name)
    env = {
        'CARGO_HOME': '/tmp/csgrs-properties-cargo',
        'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
        'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
        'CARGO_BUILD_JOBS': '2', 'CARGO_TARGET_DIR': str(target),
        'RUSTDOCFLAGS': '-D warnings',
        'CSGRS_LIBRARY': str(lib / 'libcsgrs_ffi.so'),
        'WASM_BINDGEN': '/tmp/csgrs-properties-tools/bin/wasm-bindgen',
        'TSC': str(ROOT / 'tests/wasm/node_modules/.bin/tsc'),
    }
    v.dump(out / 'protocol.json', {
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'commands': commands, 'environment': env, 'preserved_executables_sha256': preserved,
        'runner_sha256': v.sha(Path(__file__)), 'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'scope': 'Affected local and downstream consumers; not the complete repository pre-push matrix, registry installation, or proof of plan completion.',
        'wall_limit_seconds_per_command': 1800,
        'stop_on_first_failure': True, 'no_retry_resume_filter_or_pool': True,
    })
    rows = []
    binaries = {}

    def guard():
        assert sources() == initial
        assert all(v.sha(Path(path)) == digest for path, digest in (preserved | binaries).items())

    for name, command in commands:
        guard()
        print(json.dumps({'started': name, 'phase': phase}), flush=True)
        start = time.monotonic()
        with (out / (name + '.stdout')).open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            child = subprocess.Popen(command, cwd=ROOT, env=os.environ | env,
                                     stdout=stdout, stderr=stderr, start_new_session=True)
            try:
                code = child.wait(timeout=1800)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
                code = 124
        errors = (out / (name + '.stderr')).read_text()
        paths = [ROOT / p for p in re.findall(r'^\s*Running .* \(([^\n]+)\)$', errors, re.M)]
        if name in {'c-consumer', 'cpp-consumer'}:
            paths.append(Path(command[0]))
        if name == 'ffi-build' and code == 0:
            paths.append(lib / 'libcsgrs_ffi.so')
        if name == 'wasm-consumers' and code == 0:
            paths += [target / 'wasm32-unknown-unknown/wasm/csgrs.wasm',
                      target / 'wasm-bindings-node/csgrs_bg.wasm',
                      target / 'wasm-bindings-node/csgrs.js',
                      target / 'wasm-bindings-node/csgrs.d.ts']
        binaries.update({str(path.resolve()): v.sha(path) for path in paths})
        guard()
        row = {'name': name, 'exit_code': code, 'seconds': time.monotonic() - start,
               'sources_and_preserved_binaries_unchanged': True,
               'stdout_sha256': v.sha(out / (name + '.stdout')),
               'stderr_sha256': v.sha(out / (name + '.stderr')),
               'warnings': re.findall(r'^warning[^\n]*', errors, re.M)}
        rows.append(row)
        v.dump(out / 'binaries.json', binaries)
        v.dump(out / 'result.json', {'commands': rows, 'passed': len(rows) == len(commands) and all(r['exit_code'] == 0 for r in rows)})
        print(json.dumps(row), flush=True)
        if code:
            raise SystemExit(code)


if __name__ == '__main__':
    main()
