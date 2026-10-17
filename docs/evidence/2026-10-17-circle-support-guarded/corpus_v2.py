"""Run the entire no-feature library corpus from a fresh source mirror."""
import os
import re
import shutil
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import RUNTIME, W, dump, read, run, sha

build = RUNTIME / sys.argv[1]
qualification = RUNTIME / sys.argv[2]
compiled = read(build / 'result.json')
focused = read(qualification / 'result.json')
assert compiled['complete'] and compiled['output_closed'] and not compiled['command_failures']
assert focused['complete'] and focused['output_closed'] and not focused['command_failures']
protocol = read(build / 'protocol.json')
binary = compiled['binaries']['candidate']
assert sha(binary['path']) == binary['sha256']
source = protocol['sources']['candidate']
assert all(sha(Path(source['path']) / path) == digest for path, digest in source['sha256'].items())
assert str(source['path']).encode() not in Path(binary['path']).read_bytes()
output = RUNTIME / 'corpus-v2'
output.mkdir(exist_ok=False)
mirror = output / 'runtime-source-mirror'
mirror.mkdir()
for path, digest in source['sha256'].items():
    dest = mirror / path
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(Path(source['path']) / path, dest)
    assert sha(dest) == digest
pins = {str(path): sha(path) for path in [Path(__file__), W / 'runner_v1.py', W / 'contract-v1.md',
                                       build / 'result.json', build / 'protocol.json', qualification / 'result.json']}
pins[binary['path']] = binary['sha256']
dump(output / 'protocol.json', {
    'input_sha256': pins, 'binary': binary, 'source_mirror_sha256': source['sha256'],
    'features': [], 'selection': 'Entire library test binary, with no added filtering or policy changes.',
    'declared_default_features': [], 'separate_cargo_default_feature_command_executed': False,
    'existing_and_diagnostic_ignored_cases_preserved': True,
    'existing_generated_test_environment': {key: value for key, value in os.environ.items() if key.startswith('PROPTEST_')},
    'RUST_BACKTRACE': '0',
    'full_crate_all_features_or_downstream_qualification_claimed': False,
    'cargo_targets_reopened': False,
})
row = run(output, 'tests', [binary['path'], '--nocapture', '--test-threads=1'], mirror, {'RUST_BACKTRACE': '0'})
dump(output / 'commands.json', [row])
unchanged = all(sha(mirror / path) == digest for path, digest in source['sha256'].items())
generated = {str(path.relative_to(mirror)): sha(path) for path in mirror.rglob('*')
             if path.is_file() and str(path.relative_to(mirror)) not in source['sha256']}
assert all(sha(path) == digest for path, digest in pins.items())
summaries = re.findall(r'test result: (\w+)\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;',
                       (output / 'tests.stdout').read_text())
dump(output / 'result.json', {'complete': True, 'output_closed': True,
     'command_failures': [] if row['exit_code'] == 0 else ['tests'], 'exit_code': row['exit_code'],
     'external_stop_reason': row['external_stop_reason'], 'source_mirror_unchanged': unchanged,
     'generated_outputs': generated, 'test_summaries': summaries, 'all_inputs_unchanged': True,
     'property_persistence_warnings': 'FileFailurePersistence' in (output / 'tests.stderr').read_text(),
     'no_feature_library_only': True, 'cache_admitted': False, 'full_plan_complete': False})
raise SystemExit(0 if row['exit_code'] == 0 and unchanged and 'FileFailurePersistence' not in (output / 'tests.stderr').read_text() else 101)
