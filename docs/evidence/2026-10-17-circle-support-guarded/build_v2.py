"""Correct the lint in a fresh candidate; reuse the unchanged qualified baseline."""
import shutil
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import RUNTIME, W, dump, read, run, sha

old = RUNTIME / 'build-v1'
original = read(old / 'protocol.json')
built = read(old / 'result.json')
focused = RUNTIME / 'focused-v1'
assert built['complete'] and built['output_closed'] and not built['command_failures']
assert read(focused / 'result.json')['command_failures'] == ['candidate-clippy']
qualified = {row['name']: row for row in read(focused / 'commands.json')}
assert qualified['baseline-contracts']['exit_code'] == qualified['baseline-clippy']['exit_code'] == 0
assert 'clippy::collapsible-if' in (focused / 'candidate-clippy.stderr').read_text()
source = RUNTIME / 'build-v2/candidate-source'
output = source.parent
output.mkdir(exist_ok=False)
source.mkdir()
previous = original['sources']['candidate']
for path, digest in previous['sha256'].items():
    assert sha(Path(previous['path']) / path) == digest
    dest = source / path
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(Path(previous['path']) / path, dest)
path = source / 'crates/hypercurve/src/curve_intersection.rs'
text = path.read_text()
before = '''        if small_batch {
            if let Some(index) =
                (0..self.circular_support_relations.len()).find(|&index| matches(index))
            {
                return Ok(Some(
                    self.circular_support_relations[index].relation.clone(),
                ));
            }
        }'''
after = '''        if small_batch
            && let Some(index) =
                (0..self.circular_support_relations.len()).find(|&index| matches(index))
        {
            return Ok(Some(
                self.circular_support_relations[index].relation.clone(),
            ));
        }'''
assert text.count(before) == 1
path.write_text(text.replace(before, after))
commands = [run(output, 'format', ['rustfmt', '--edition', '2024', '--config', 'skip_children=true', str(path)], source)]
assert commands[0]['exit_code'] == 0
pins = {str(path): sha(path) for path in [Path(__file__), W / 'runner_v1.py', W / 'contract-v1.md',
                old / 'result.json', old / 'protocol.json', focused / 'commands.json', focused / 'result.json']}
sources = {'baseline': original['sources']['baseline'],
           'candidate': {'path': str(source), 'sha256': {
               str(path.relative_to(source)): sha(path) for path in source.rglob('*') if path.is_file()}}}
changed = {path for path, digest in sources['candidate']['sha256'].items() if previous['sha256'][path] != digest}
assert changed == {'crates/hypercurve/src/curve_intersection.rs'}
shutil.copy2(old / 'main-input-hashes.json.gz', output / 'main-input-hashes.json.gz')
comparison = subprocess.run(['git', 'diff', '--no-index', '--binary', '--',
    str(Path(previous['path']) / 'crates/hypercurve/src/curve_intersection.rs'),
    str(source / 'crates/hypercurve/src/curve_intersection.rs')], stdout=subprocess.PIPE)
assert comparison.returncode == 1
(output / 'lint-repair.patch').write_bytes(comparison.stdout)
dump(output / 'protocol.json', original | {
    'input_sha256': original['input_sha256'] | pins, 'sources': sources,
    'only_change_from_v1': 'Collapse the equivalent nested if to satisfy clippy::collapsible_if.',
    'baseline_reused_read_only': built['binaries']['baseline'],
    'baseline_qualified_commands': {name: qualified[name] for name in ['baseline-contracts', 'baseline-clippy']},
    'clippy_version': subprocess.check_output(['clippy-driver', '--version'], text=True),
    'direct_clippy_driver_scope': 'No-feature library test harness with identical dependency libraries, not Cargo all-feature/all-target Clippy.',
})
binary = output / 'candidate-tests'
command = next(row['command'] for row in read(old / 'commands.json') if row['name'] == 'candidate')
command = [arg.replace(previous['path'], str(source)).replace(str(old / 'candidate-tests'), str(binary)) for arg in command]
row = run(output, 'candidate', command, source)
commands.append(row)
binaries = {'baseline': built['binaries']['baseline']}
if row['exit_code'] == 0:
    binaries['candidate'] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    commands.append(run(output, 'candidate-contracts', [str(binary),
        'curve_intersection::circle_support_cache_tests::', '--nocapture', '--test-threads=1'], output))
    if commands[-1]['exit_code'] == 0:
        assert 'test result: ok. 8 passed; 0 failed;' in (output / 'candidate-contracts.stdout').read_text()
    lint = command.copy()
    lint[0] = 'clippy-driver'
    lint[lint.index('-o') + 1] = str(output / 'candidate.rmeta')
    lint.append('--emit=metadata')
    commands.append(run(output, 'candidate-clippy', lint, source))
dump(output / 'commands.json', commands)
assert all(sha(path) == digest for path, digest in (original['input_sha256'] | pins | original['read_only_dependencies_sha256']).items())
assert all(sha(Path(value['path']) / path) == digest
           for value in sources.values() for path, digest in value['sha256'].items())
assert all(sha(value['path']) == value['sha256'] for value in binaries.values())
dump(output / 'result.json', {'complete': True, 'output_closed': True, 'binaries': binaries,
    'command_failures': [row['name'] for row in commands if row['exit_code'] != 0],
    'sources_and_dependencies_unchanged': True, 'main_sources_unchanged': True,
    'baseline_reused_with_qualified_source_and_binary_identity': True,
    'scope': 'Both no-feature test harnesses passed the eight exact cache contracts and strict direct Clippy.',
    'cache_admitted': False})
raise SystemExit(0 if len(binaries) == 2 and not any(row['exit_code'] for row in commands) else 101)
