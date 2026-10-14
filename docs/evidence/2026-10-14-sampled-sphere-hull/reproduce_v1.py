"""Snapshot current source, then record four unchanged strict sphere hull queries."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/sampled-sphere-hull-v1/baseline'
SOURCE = WORK / 'source'
TARGET = WORK / 'target'
OUT = E / 'baseline-v1'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'CARGO_TARGET_DIR': str(TARGET)}
CASES = ['fifteen_degrees', 'original_medium', 'fifth_turn', 'seventh_turn']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')


def main():
    assert not WORK.exists() and not OUT.exists()
    paths = set(filter(None, subprocess.check_output(
        ['git', 'ls-files', '-c', '-o', '--exclude-standard', '-z'], cwd=ROOT).decode().split('\0')))
    paths = sorted(p for p in paths if not p.startswith(('docs/evidence/', 'crates/comparisons/', 'target/')))
    mapping = {p: sha(ROOT / p) for p in paths}
    native_path = ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json'
    native = json.loads(native_path.read_text())['pending_native_sha256']
    assert len(native) == 35 and all(mapping[p] == h for p, h in native.items())
    WORK.mkdir(parents=True)
    OUT.mkdir()
    SOURCE.mkdir()
    for p in paths:
        destination = SOURCE / p
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / p, destination)
    hashes = OUT / 'source-hashes.json.gz'
    hashes.write_bytes(gzip.compress((json.dumps(mapping, sort_keys=True) + '\n').encode(), mtime=0))
    inputs = {p: sha(E / p) for p in ['reproduce_v1.py', 'reproduce_v1.rs', 'scope-v1.json']}
    commands = []

    def guard():
        assert all(sha(SOURCE / p) == h for p, h in mapping.items())
        assert all(sha(ROOT / p) == h for p, h in mapping.items())
        assert all(sha(E / p) == h for p, h in inputs.items())

    def run(name, command):
        guard()
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | ENV,
                                  stdout=stdout, stderr=stderr).returncode
        elapsed = time.monotonic() - started
        guard()
        row = {'name': name, 'command': command, 'exit_code': code,
               'process_wall_seconds': elapsed,
               'stdout_sha256': sha(OUT / (name + '.stdout')),
               'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        if code:
            raise SystemExit(code)

    dump(OUT / 'protocol.json', {'status': 'frozen-before-execution',
         'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
         'source': str(SOURCE), 'target': str(TARGET), 'source_files': len(mapping),
         'source_hashes_sha256': sha(hashes), 'inputs_sha256': inputs,
         'environment': ENV, 'cases': CASES, 'per_case_wall_stop_seconds': 180,
         'existing_native_sha256': native, 'case_order_is_fixed': True,
         'scope': 'Correctness and failure reproduction, not a matched benchmark or native DRC gate.'})
    run('probe-format', ['rustfmt', '--edition', '2024', '--check', str(E / 'reproduce_v1.rs')])
    run('build-libraries', ['cargo', 'build', '--locked', '--release', '-p', 'csgrs',
                          '--no-default-features', '--lib', '--message-format=json'])
    cargo_rows = [json.loads(line) for line in (OUT / 'build-libraries.stdout').read_text().splitlines()
                  if line.startswith('{')]
    libraries = [r for r in cargo_rows if r.get('reason') == 'compiler-artifact'
                 and r['target']['name'] == 'csgrs' and 'lib' in r['target']['kind']]
    if not libraries:
        libraries = [r for r in cargo_rows if r.get('reason') == 'compiler-artifact'
                     and r['target']['name'] == 'csgrs' and 'rlib' in r['target']['crate_types']]
    assert len(libraries) == 1
    library = next(p for p in libraries[0]['filenames'] if p.endswith('.rlib'))
    binary = WORK / 'reproduce-v1'
    run('link-probe', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'reproduce_v1.rs'),
                      '--extern', 'csgrs=' + library, '-L', 'dependency=' + str(TARGET / 'release/deps'),
                      '-o', str(binary)])
    dump(OUT / 'binary.json', {'path': str(binary), 'sha256': sha(binary),
         'bytes': binary.stat().st_size, 'library': library, 'library_sha256': sha(library),
         'features': libraries[0]['features']})
    for case in CASES:
        run(case, ['timeout', '--signal=TERM', '--kill-after=10s', '180s',
                   '/usr/bin/time', '-v', str(binary), case])
    guard()
    dump(OUT / 'result.json', {'complete': True, 'commands_passed': len(commands),
         'source_files': len(mapping), 'all_sources_unchanged': True,
         'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
         'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'commands_passed': len(commands), 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
