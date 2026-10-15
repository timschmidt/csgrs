"""Check a benchmark description and README correction in a frozen source copy."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'scalar-selection-wording-v1'
WORK = ROOT / 'target/scalar-selection-wording-v1'
SOURCE = WORK / 'source'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not OUT.exists() and not (WORK / 'target').exists()
    scope = read(E / 'scalar-selection-wording-scope-v1.json')
    base = json.loads(gzip.decompress((E / 'structural-fuzz-inventory-admission-v5/source-hashes-after.json.gz').read_bytes()))
    sources = {str(p.relative_to(SOURCE)): sha(p) for p in SOURCE.rglob('*') if p.is_file()}
    assert len(base) == len(sources) == 10153 and sources == base | scope['expected_delta_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in base.items())
    inputs = {name: sha(E / name) for name in [
        'qualify_scalar_selection_wording_v1.py', 'scalar-selection-wording-scope-v1.json',
        'scalar-selection-wording-v1.patch', 'scalar-selection-review-v1.json',
        'structural-fuzz-inventory-admission-v5/source-hashes-after.json.gz',
    ]}
    preserved = list(read(E / 'structural-fuzz-inventory-v5/binaries.json').values())
    environment = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
                   'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
                   'CARGO_TARGET_DIR': str(WORK / 'target'), 'RUSTUP_TOOLCHAIN': 'stable', 'RUSTFLAGS': ''}

    def guard():
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        for directory, table in [(ROOT, base), (SOURCE, sources)]:
            assert all(sha(directory / name) == digest for name, digest in table.items())
        assert all(sha(row['path']) == row['sha256'] for row in preserved)

    guard()
    OUT.mkdir()
    (OUT / 'source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(sources, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'source': str(SOURCE), 'source_files': len(sources),
                               'inputs_sha256': inputs, 'environment': environment,
                               'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
                               'preserved_binaries': preserved, 'commands': 2,
                               'not_a_performance_measurement': True})
    commands = []
    for name, command in [
        ('format', ['cargo', 'fmt', '--all', '--', '--check']),
        ('benchmark-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal',
                             '--bench', 'adversarial_library', '--features', 'simple', '--', '-D', 'warnings']),
    ]:
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
        print(json.dumps(row), flush=True)
        guard()
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row, 'admitted': False})
            raise SystemExit(1)
    dump(OUT / 'result.json', {'complete': True, 'commands_all_zero': 2, 'all_frozen_sources_inputs_and_binaries_rechecked': True,
                             'artifacts_sha256': {p.name: sha(p) for p in OUT.iterdir() if p.is_file()},
                             'limits': scope['limits'], 'admitted': False, 'P38_complete': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
