"""Continue the active physical build with adequate scratch space, preserving completed commands."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import shutil
import signal
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = Path('/dev/shm/csgrs-physical-build-requalification')
SOURCE, TARGET = WORK / 'source-v1', WORK / 'target-v1'
OLD, OUT = E / 'validation-v1', E / 'validation-v2'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def process(pid):
    path = Path('/proc') / str(pid) / 'stat'
    raw = path.read_text()
    fields = raw[raw.rfind(')') + 2:].split()
    return fields[0], int(fields[1]), int(fields[49])


transition = json.loads((E / 'scratch-transition-v1.json').read_text())
supervisor = transition['supervisor_pid']
child = int(transition['current_direct_child'].split()[0])
previous = json.loads((OLD / 'commands.json').read_text())
old_protocol = json.loads((OLD / 'protocol.json').read_text())
commands = old_protocol['commands']
current_name, current_command = commands[len(previous)]
assert current_name == 'curve-all'
assert [r['name'] for r in previous] == transition['completed_commands']
with gzip.open(OLD / 'source-hashes.json.gz', 'rt') as stream:
    inputs = json.load(stream)
OUT.mkdir(exist_ok=False)
scratch = WORK / 'scratch-v2'
scratch.mkdir(exist_ok=False)
protocol = {'runner_sha256': sha(Path(__file__)), 'parent_protocol_sha256': sha(OLD / 'protocol.json'),
            'source_hashes_path': str(OLD / 'source-hashes.json.gz'),
            'source_hashes_sha256': sha(OLD / 'source-hashes.json.gz'),
            'source': str(SOURCE), 'target': str(TARGET),
            'target_strategy': old_protocol['target_strategy'], 'active_target_continued': True,
            'completed_target_reset_or_reopened': False, 'used_runner_edited': False,
            'transition': str(E / 'scratch-transition-v1.json'), 'scratch': str(scratch),
            'completed_command_names': [r['name'] for r in previous],
            'adopted_running_command': [current_name, current_command],
            'remaining_commands': commands[len(previous) + 1:],
            'elapsed_limit_seconds_per_command': 1800, 'address_space_limit_bytes_per_process': 8 * 1024**3,
            'runtime_gates_wait_for_format_reproduction': True,
            'runtime_admitted': False, 'full_plan_complete': False}
dump(OUT / 'protocol.json', protocol)
print('Waiting for the already-running curve-all command; its process is uninterrupted.', flush=True)
while True:
    state, parent, raw_status = process(child)
    assert parent == supervisor
    if state == 'Z':
        break
    time.sleep(1)
code = os.waitstatus_to_exitcode(raw_status)
assert process(supervisor)[0] in ['T', 't']
# Terminate only the stopped supervisor after its command has exited.
os.kill(supervisor, signal.SIGTERM)
os.kill(supervisor, signal.SIGCONT)
dump(E / 'scratch-transition-completion-v1.json', {
    'supervisor_pid': supervisor, 'terminated_after_current_command_exit': True,
    'command': current_name, 'command_exit_code': code, 'child_wait_status': raw_status,
    'target_reset': False, 'used_runner_edited': False})
assert all(sha(SOURCE / p) == h and sha(ROOT / p) == h for p, h in inputs.items())
retained = WORK / 'executables-v1'


def capture(name, command, directory, exit_code, elapsed, environment_kind):
    assert all(sha(SOURCE / p) == h and sha(ROOT / p) == h for p, h in inputs.items())
    stdout, stderr = directory / (name + '.stdout'), directory / (name + '.stderr')
    paths = {Path(p) for p in re.findall(r'Running[^\n]*\(([^)]+)\)', stderr.read_text(errors='replace'))}
    if name == 'native-cli' and exit_code == 0:
        for line in stdout.read_text().splitlines():
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            if item.get('reason') == 'compiler-artifact' and item.get('executable'):
                paths.add(Path(item['executable']))
    binaries = {}
    for path in paths:
        path = path if path.is_absolute() else SOURCE / path
        assert path.is_relative_to(TARGET) and path.is_file(), path
        digest = sha(path)
        copy = retained / digest / path.name
        copy.parent.mkdir(exist_ok=True)
        if not copy.exists():
            shutil.copy2(path, copy)
        assert sha(copy) == digest
        binaries[str(path)] = {'sha256': digest, 'retained': str(copy), 'bytes': path.stat().st_size}
    return {'name': name, 'command': command, 'exit_code': exit_code, 'elapsed_seconds': elapsed,
            'environment_kind': environment_kind, 'log_directory': str(directory), 'sources_unchanged': True,
            'binaries': binaries, **{suffix + '_sha256': sha(directory / (name + '.' + suffix))
                                   for suffix in ['stdout', 'stderr', 'resources']}}


resource_text = (OLD / (current_name + '.resources')).read_text()
elapsed_text = re.search(r'Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): (\S+)', resource_text).group(1)
elapsed = 0.0
for value in elapsed_text.split(':'):
    elapsed = elapsed * 60 + float(value)
results = [row | {'log_directory': str(OLD), 'environment_kind': 'initial runner'} for row in previous]
row = capture(current_name, current_command, OLD, code, elapsed, 'initial runner; completion recorded by continuation')
row['elapsed_source'] = 'Original /usr/bin/time measurement; excludes supervisor pause after child completion.'
results.append(row)
dump(OUT / 'commands.json', results)
print(json.dumps({'adopted': current_name, 'exit_code': code, 'elapsed_seconds': elapsed}), flush=True)
environment = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    environment.pop(key, None)
environment.update(old_protocol['environment'])
environment.update({'TMPDIR': str(scratch), 'RUST_BACKTRACE': '0', 'CARGO_TERM_COLOR': 'never'})


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


for name, command in commands[len(results):]:
    assert sha(Path(__file__)) == protocol['runner_sha256']
    if name == 'full-board':
        print('Waiting for isolated format compilation before original timing gates.', flush=True)
        while not (E / 'format-reproduction-v1/result.json').exists():
            time.sleep(1)
        assert json.loads((E / 'format-reproduction-v1/result.json').read_text())['complete']
    print('START ' + name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('wb') as stdout, (OUT / (name + '.stderr')).open('wb') as stderr:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(OUT / (name + '.resources')),
                                 'timeout', '--signal=TERM', '--kill-after=20s', '1800s', *command],
                                cwd=SOURCE, env=environment, stdout=stdout, stderr=stderr,
                                preexec_fn=limits, start_new_session=True)
    row = capture(name, command, OUT, result.returncode, time.monotonic() - start, 'continuation with dedicated scratch')
    results.append(row)
    dump(OUT / 'commands.json', results)
    print(json.dumps({'name': name, 'exit_code': row['exit_code'], 'seconds': row['elapsed_seconds']}), flush=True)
assert [r['name'] for r in results] == [r[0] for r in commands]
record = {'complete': True, 'commands': len(results), 'passed': sum(r['exit_code'] == 0 for r in results),
          'failed': [r['name'] for r in results if r['exit_code'] != 0], 'sources_unchanged': True,
          'target_strategy': protocol['target_strategy'], 'active_target_continued': True,
          'runtime_admitted': False, 'full_plan_complete': False}
dump(OUT / 'result.json', record)
print(json.dumps(record), flush=True)
if record['failed']:
    raise SystemExit(101)
