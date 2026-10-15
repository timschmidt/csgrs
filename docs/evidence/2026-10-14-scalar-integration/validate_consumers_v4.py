"""Exercise the integrated scalar runtime with the in-scope native geometry.

The curve and DRC commands retain the prior worktree validation's complete
inventories and test concurrency, including its original full-board guards.
"""
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('extended_v4', E / 'validate_extended_v4.py')
x = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(x)
p, q = x.p, x.q
OUT = E / 'consumers-v4'


def main():
    assert not OUT.exists()
    caps = json.loads((E / 'unit-caps-v4/result.json').read_text())
    assert caps['complete'] and caps['commands_executed'] == 39
    source = p.read_map(E / 'freeze-v4/source-hashes.json.gz')
    main_sources = p.read_map(p.MAIN_RECORD)
    native = json.loads((p.ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert len(native) == 35 and all(source[name] == digest for name, digest in native.items())
    preserved = json.loads((E / 'native-closure-v4.json').read_text())['native_binaries_sha256']
    preserved |= json.loads((E / 'extended-v4/binaries.json').read_text())
    env = q.ENV | {'CARGO_TARGET_DIR': str(q.WORK / 'consumer-target')}
    assert not Path(env['CARGO_TARGET_DIR']).exists()
    common = ['cargo', 'test', '--locked', '--release']
    suffix = ['--all-features', '--lib', '--tests', '--', '--test-threads=2']
    commands = [
        ('foundations', common + ['-p', 'hyperlattice', '-p', 'hyperlimit', '-p', 'hypersolve'] + suffix),
        ('curve', common + ['-p', 'hypercurve'] + suffix),
        ('drc', common + ['-p', 'hyperdrc'] + suffix),
    ]
    inputs = {name: p.sha(E / name) for name in ['validate_consumers_v4.py', 'validate_extended_v4.py',
              'qualify_v4.py', 'prepare_v1.py', 'unit-caps-v4/result.json', 'freeze-v4/source-hashes.json.gz']}
    binaries = {}

    def guard():
        assert x.source_inventory() == source
        assert all(p.sha(p.ROOT / name) == digest for name, digest in main_sources.items())
        assert all(p.sha(E / name) == digest for name, digest in inputs.items())
        assert all(p.sha(Path(name)) == digest for name, digest in (preserved | binaries).items())

    guard()
    OUT.mkdir()
    p.dump(OUT / 'protocol.json', {
        'commands': commands, 'environment': env, 'cwd': str(q.SOURCE), 'inputs_sha256': inputs,
        'pending_native_sha256': native, 'preserved_binaries_sha256': preserved,
        'original_runtime_guards': 'Full inventories and original assertions retained; no limit, fixture, filter or ignore changes.',
        'scope': 'Fresh native consumer qualification on the integrated source, not a matched speed comparison or whole-repository validation.',
        'stop_on_first_failing_command': True, 'no_retry_resume_filter_or_pool': True,
        'runtime_admitted': False,
    })
    rows = []
    for name, command in commands:
        guard()
        print(json.dumps({'started': name}), flush=True)
        start = time.monotonic()
        stdout_path, stderr_path = OUT / f'{name}.stdout', OUT / f'{name}.stderr'
        with stdout_path.open('wb') as stdout, stderr_path.open('wb') as stderr:
            result = subprocess.run(command, cwd=q.SOURCE, env=os.environ | env, stdout=stdout, stderr=stderr)
        output, errors = stdout_path.read_text(), stderr_path.read_text()
        for binary in re.findall(r'^\s*Running .* \((/[^\n]+)\)$', errors, re.M):
            assert Path(binary).is_file()
            binaries[binary] = p.sha(Path(binary))
        guard()
        row = {'name': name, 'exit_code': result.returncode, 'seconds': time.monotonic() - start,
               'stdout_sha256': p.sha(stdout_path), 'stderr_sha256': p.sha(stderr_path),
               'test_batches': [dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'], map(int, match)))
                               for match in re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out', output)],
               'warnings': re.findall(r'^warning[^\n]*', errors, re.M),
               'sources_and_previous_binaries_unchanged': True}
        rows.append(row)
        p.dump(OUT / 'commands.json', rows)
        p.dump(OUT / 'binaries.json', binaries)
        p.dump(OUT / 'result.json', {'complete': len(rows) == len(commands) and all(row['exit_code'] == 0 for row in rows),
                                   'commands_executed': len(rows), 'commands_expected': len(commands),
                                   'last_exit_code': result.returncode, 'runtime_admitted': False})
        print(json.dumps({key: row[key] for key in ['name', 'exit_code', 'seconds', 'warnings']}), flush=True)
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
