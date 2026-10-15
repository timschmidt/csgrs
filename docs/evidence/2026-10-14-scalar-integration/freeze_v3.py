"""Format the integrated source and freeze the complete inputs before execution."""
import gzip
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('prepare_v1', E / 'prepare_v1.py')
p = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(p)
SOURCE = p.ROOT / 'target/scalar-main-integration-v3/source'
OUT = E / 'freeze-v3'


def main():
    assert not OUT.exists()
    OUT.mkdir()
    main_sources = p.read_map(p.MAIN_RECORD)
    before = p.inventory(SOURCE)
    changed = {name for name, digest in before.items() if main_sources.get(name) != digest}
    included = sorted(name for name in changed if name.endswith('.rs') and '/vendor/' not in name)
    commands = [
        ['cargo', 'fmt', '--all'],
        ['rustfmt', '--edition', '2024', '--config-path', 'crates/.rustfmt.toml',
         '--config', 'skip_children=true', *included],
        ['cargo', 'fmt', '--manifest-path', 'crates/hyperreal/fuzz/Cargo.toml'],
    ]
    p.dump(OUT / 'protocol.json', {'commands': commands, 'runner_sha256': p.sha(Path(__file__)),
                                 'scope': 'Source formatting only, before the qualification freeze.'})
    rows = []
    for index, command in enumerate(commands):
        start = time.monotonic()
        with (OUT / f'{index}.stdout').open('wb') as stdout, (OUT / f'{index}.stderr').open('wb') as stderr:
            result = subprocess.run(command, cwd=SOURCE, env=os.environ | {'RUSTUP_TOOLCHAIN': 'stable'},
                                    stdout=stdout, stderr=stderr)
        row = {'command': command, 'exit_code': result.returncode, 'seconds': time.monotonic() - start,
               'stdout_sha256': p.sha(OUT / f'{index}.stdout'), 'stderr_sha256': p.sha(OUT / f'{index}.stderr')}
        rows.append(row)
        p.dump(OUT / 'commands.json', rows)
        print(json.dumps({'format_command': index, 'exit_code': result.returncode}), flush=True)
        if result.returncode:
            raise SystemExit(result.returncode)
    after = p.inventory(SOURCE)
    delta = {name: digest for name, digest in after.items() if main_sources.get(name) != digest}
    assert not main_sources.keys() - after.keys()
    assert all(p.sha(p.ROOT / name) == digest for name, digest in main_sources.items())
    assert p.inventory(p.ROOT / 'target/scalar-main-integration-v2/source') == p.read_map(E / 'preparation-v2/source-hashes.json.gz')
    record = OUT / 'source-hashes.json.gz'
    record.write_bytes(gzip.compress((json.dumps(after, sort_keys=True) + '\n').encode(), mtime=0))
    p.archive(OUT / 'overlay.tar.gz', SOURCE, delta)
    p.dump(OUT / 'result.json', {
        'complete': True, 'source_directory': str(SOURCE), 'source_files': len(after),
        'source_record_sha256': p.sha(record), 'overlay_sha256': p.sha(OUT / 'overlay.tar.gz'),
        'changes_from_main_sha256': delta,
        'formatting_changed_sha256': {name: digest for name, digest in after.items() if before.get(name) != digest},
        'included_format_inputs': included, 'main_and_conflicted_source_unchanged': True,
        'runtime_admitted': False,
    })
    print(json.dumps({'frozen_source_files': len(after), 'changed_from_main': len(delta)}), flush=True)


if __name__ == '__main__':
    main()
