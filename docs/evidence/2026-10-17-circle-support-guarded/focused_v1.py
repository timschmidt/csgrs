"""Check focused semantics and strict lints without reopening a Cargo target."""
import re
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import RUNTIME, W, dump, read, run, sha

build = RUNTIME / 'build-v1'
result = read(build / 'result.json')
assert result['complete'] and result['output_closed'] and not result['command_failures']
protocol = read(build / 'protocol.json')
output = RUNTIME / 'focused-v1'
output.mkdir(exist_ok=False)
pins = {str(path): sha(path) for path in [Path(__file__), W / 'runner_v1.py', W / 'contract-v1.md',
                                       build / 'protocol.json', build / 'result.json']}
pins |= {record['path']: record['sha256'] for record in result['binaries'].values()}
dump(output / 'protocol.json', {'input_sha256': pins, 'build': str(build),
     'read_only_dependencies_sha256': protocol['read_only_dependencies_sha256'],
     'direct_clippy_driver_scope': 'No-feature library test harness with the same source and dependencies; not Cargo all-feature/all-target Clippy.',
     'cargo_targets_reopened': False})
commands = []
for label, record in result['binaries'].items():
    assert sha(record['path']) == record['sha256']
    row = run(output, label + '-contracts', [record['path'],
              'curve_intersection::circle_support_cache_tests::', '--nocapture', '--test-threads=1'], output)
    commands.append(row)
    dump(output / 'commands.json', commands)
    if row['exit_code'] == 0:
        assert 'test result: ok. 8 passed; 0 failed;' in (output / (row['name'] + '.stdout')).read_text()
    source = Path(protocol['sources'][label]['path'])
    command = ['clippy-driver', '--crate-name', 'hypercurve', '--edition=2024', '--test',
               str(source / 'crates/hypercurve/src/lib.rs'), '-C', 'opt-level=3', '-C', 'codegen-units=16',
               '-C', 'embed-bitcode=no', '-D', 'warnings', '--emit=metadata',
               '--remap-path-prefix', str(source) + '=/csgrs-circle-support-guarded-v1',
               '-L', 'dependency=' + str(Path(protocol['externs']['hyperreal']).parent),
               '-o', str(output / (label + '.rmeta'))]
    for name, path in protocol['externs'].items():
        command += ['--extern', name + '=' + path]
    row = run(output, label + '-clippy', command, source)
    commands.append(row)
    dump(output / 'commands.json', commands)
    if row['external_stop_reason']:
        break
assert all(sha(path) == digest for path, digest in (pins | protocol['read_only_dependencies_sha256']).items())
assert all(sha(Path(value['path']) / path) == digest
           for value in protocol['sources'].values() for path, digest in value['sha256'].items())
dump(output / 'result.json', {'complete': True, 'output_closed': True,
     'command_failures': [row['name'] for row in commands if row['exit_code'] != 0],
     'all_inputs_unchanged': True, 'cache_admitted': False})
raise SystemExit(101 if any(row['exit_code'] != 0 for row in commands) else 0)
