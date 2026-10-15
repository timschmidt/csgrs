"""Validate frozen scalar integration with standalone builds, ASan and Verus.

Run only after the preceding native gate has terminated successfully. Generated
proof output is allowed solely inside target directories, and is archived.
"""
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('qualify_v4', E / 'qualify_v4.py')
q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(q)
p = q.p
OUT = E / 'extended-v4'
TRIPLE = 'x86_64-unknown-linux-gnu'


def source_inventory():
    result = {}
    for folder, directories, files in os.walk(q.SOURCE):
        directories[:] = sorted(name for name in directories if name != 'target')
        for name in sorted(files):
            path = Path(folder) / name
            assert not path.is_symlink()
            result[str(path.relative_to(q.SOURCE))] = p.sha(path)
    return result


def main():
    assert not OUT.exists()
    native = json.loads((E / 'qualification-v4/result.json').read_text())
    assert native['complete'] and native['commands_executed'] == native['commands_expected'] == 8
    crate = q.SOURCE / 'crates/hyperreal'
    pin = json.loads((crate / 'verification/toolchain.json').read_text())
    verus = p.ROOT / 'crates/hyperreal/target/verus' / pin['version'] / 'verus-x86-linux'
    tooling = p.inventory(verus)
    assert 'verus' in tooling
    std_record = E.parent / '2026-10-14-sampled-sphere-hull/structural-fuzz-std-dependencies-source-v1.json.gz'
    std = p.read_map(std_record)
    std_root = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/lib/rustlib/src/rust/library')
    assert len(std) == 3539
    sources = p.read_map(E / 'freeze-v4/source-hashes.json.gz')
    assert not any('target' in Path(name).parts for name in sources)
    main_sources = p.read_map(p.MAIN_RECORD)
    env = q.ENV | {'CARGO_TARGET_DIR': str(q.WORK / 'extended-check-target')}
    asan = env | {'CARGO_TARGET_DIR': str(q.WORK / 'sanitizer-target'), 'RUSTC_BOOTSTRAP': '1',
                  'RUSTFLAGS': '-Zsanitizer=address -Cforce-frame-pointers=yes',
                  'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1:abort_on_error=0:exitcode=66', 'LSAN_OPTIONS': ''}
    assert not Path(env['CARGO_TARGET_DIR']).exists() and not Path(asan['CARGO_TARGET_DIR']).exists()
    assert not (crate / 'target').exists()
    proof_env = env | {'VERUS': str(verus / 'verus')}
    commands = [
        ('docs', ['cargo', 'doc', '--locked', '-p', 'hyperreal', '--all-features', '--no-deps'],
         q.SOURCE, env | {'RUSTDOCFLAGS': '-D warnings'}),
        ('standalone-fuzz-check', ['cargo', 'check', '--locked', '--manifest-path',
                                  'crates/hyperreal/fuzz/Cargo.toml', '--bins'], q.SOURCE, env),
        ('asan-default-tests', ['cargo', 'test', '--locked', '--release', '-Zbuild-std', '--target', TRIPLE,
                                '-p', 'hyperreal', '--lib', '--tests'], q.SOURCE, asan),
        ('asan-all-feature-tests', ['cargo', 'test', '--locked', '--release', '-Zbuild-std', '--target', TRIPLE,
                                    '-p', 'hyperreal', '--all-features', '--lib', '--tests'], q.SOURCE, asan),
        ('verus', ['python3', 'scripts/verify_verus.py', 'verify'], crate, proof_env),
    ]
    inputs = {name: p.sha(E / name) for name in ['prepare_v1.py', 'qualify_v4.py', 'validate_extended_v4.py',
               'freeze-v4/source-hashes.json.gz', 'freeze-v4/result.json', 'qualification-v4/result.json']}
    binaries = {}

    def guard():
        assert source_inventory() == sources
        assert all(p.sha(p.ROOT / name) == digest for name, digest in main_sources.items())
        assert all(p.sha(std_root / name) == digest for name, digest in std.items())
        assert p.inventory(verus) == tooling
        assert all(p.sha(E / name) == digest for name, digest in inputs.items())
        assert all(p.sha(Path(name)) == digest for name, digest in binaries.items())

    guard()
    OUT.mkdir()
    p.dump(OUT / 'protocol.json', {
        'commands': [{'name': name, 'argv': command, 'cwd': str(cwd), 'environment': environment}
                     for name, command, cwd, environment in commands],
        'inputs_sha256': inputs, 'verus_directory': str(verus), 'verus_sha256': tooling,
        'pinned_verus': pin, 'standard_library_source_record': str(std_record.relative_to(p.ROOT)),
        'standard_library_source_record_sha256': p.sha(std_record),
        'sanitizer_scope': 'Rust targets and rebuilt standard library. Native GMP/MPFR is not claimed instrumented.',
        'proof_scope': 'Current conditional target, dependency binding and listed contracts. All 13 whole-crate groups and empirical baseline acceptance remain open.',
        'stop_on_failure': True, 'closed_trials_are_not_resumed_retried_filtered_or_pooled': True,
        'runtime_admitted': False,
    })
    rows = []
    for name, command, cwd, environment in commands:
        guard()
        print(json.dumps({'started': name}), flush=True)
        start = time.monotonic()
        with (OUT / f'{name}.stdout').open('wb') as stdout, (OUT / f'{name}.stderr').open('wb') as stderr:
            result = subprocess.run(command, cwd=cwd, env=os.environ | environment, stdout=stdout, stderr=stderr)
        output = (OUT / f'{name}.stdout').read_text()
        errors = (OUT / f'{name}.stderr').read_text()
        for binary in re.findall(r'^\s*Running .* \((/[^\n]+)\)$', errors, re.M):
            if Path(binary).is_file():
                binaries[binary] = p.sha(Path(binary))
        reports = crate / 'target/verification'
        if name == 'verus' and reports.exists():
            shutil.copytree(reports, OUT / 'proof-reports')
        guard()
        row = {'name': name, 'exit_code': result.returncode, 'seconds': time.monotonic() - start,
               'stdout_sha256': p.sha(OUT / f'{name}.stdout'), 'stderr_sha256': p.sha(OUT / f'{name}.stderr'),
               'test_batches': [dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'], map(int, match)))
                               for match in re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out', output)],
               'sanitizer_error_markers': re.findall(r'(?:ERROR|SUMMARY): (?:AddressSanitizer|LeakSanitizer)[^\n]*', output + errors),
               'source_and_tooling_unchanged': True}
        rows.append(row)
        p.dump(OUT / 'commands.json', rows)
        p.dump(OUT / 'binaries.json', binaries)
        p.dump(OUT / 'result.json', {'complete': len(rows) == len(commands) and all(r['exit_code'] == 0 and not r['sanitizer_error_markers'] for r in rows),
                                   'commands_executed': len(rows), 'commands_expected': len(commands),
                                   'last_exit_code': result.returncode, 'runtime_admitted': False})
        print(json.dumps({key: row[key] for key in ['name', 'exit_code', 'seconds', 'sanitizer_error_markers']}), flush=True)
        if result.returncode:
            raise SystemExit(result.returncode)
        assert not row['sanitizer_error_markers']


if __name__ == '__main__':
    main()
