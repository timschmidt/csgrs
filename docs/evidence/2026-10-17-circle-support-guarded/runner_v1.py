"""Contain fresh diagnostic commands without writing to completed targets."""
import hashlib
import json
import os
import resource
import signal
import subprocess
import time
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
RUNTIME = Path('/run/user/1000/csgrs-circle-support-guarded-v1')


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def resources():
    def free(path):
        fs = os.statvfs(path)
        return fs.f_bavail * fs.f_frsize
    available = next(line for line in Path('/proc/meminfo').read_text().splitlines()
                     if line.startswith('MemAvailable:'))
    return {'repository_free_bytes': free(ROOT), 'runtime_free_bytes': free(RUNTIME),
            'host_available_memory_bytes': int(available.split()[1]) * 1024}


def violation(value):
    for key, minimum in [('repository_free_bytes', 768 * 1024**2),
                         ('runtime_free_bytes', 256 * 1024**2),
                         ('host_available_memory_bytes', 2 * 1024**3)]:
        if value[key] < minimum:
            return key + '-reserve'
    return None


def run(output, name, command, cwd, extra=None):
    initial = resources()
    reason = violation(initial)
    if reason:
        row = {'name': name, 'command': command, 'exit_code': None,
               'external_stop_reason': reason, 'not_run': True, 'initial_resources': initial}
        dump(output / (name + '.result.json'), row)
        return row
    env = os.environ.copy()
    for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER']:
        env.pop(key, None)
    scratch = output / (name + '-scratch')
    scratch.mkdir()
    env.update({'TMPDIR': str(scratch), 'PYTHONDONTWRITEBYTECODE': '1', **(extra or {})})

    def limits():
        resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

    observations = [{'elapsed_seconds': 0, **initial}]
    start = time.monotonic()
    print('START ' + name, flush=True)
    with (output / (name + '.stdout')).open('wb') as out, (output / (name + '.stderr')).open('wb') as err:
        process = subprocess.Popen(['/usr/bin/time', '-v', '-o', str(output / (name + '.resources')), *command],
                                   cwd=cwd, env=env, stdout=out, stderr=err, preexec_fn=limits,
                                   start_new_session=True)
        dump(output / (name + '.process.json'), {'pid': process.pid, 'started_unix': time.time()})
        while process.poll() is None:
            try:
                process.wait(timeout=5)
                continue
            except subprocess.TimeoutExpired:
                pass
            elapsed = time.monotonic() - start
            try:
                observed = resources()
                observations.append({'elapsed_seconds': elapsed, **observed})
                reason = violation(observed)
            except Exception as error:
                reason = 'resource-probe-unavailable'
                observations.append({'elapsed_seconds': elapsed, 'error': repr(error)})
            if elapsed > 1800:
                reason = 'external-timeout'
            if reason:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                break
        code = process.wait()
    row = {'name': name, 'command': command, 'exit_code': code, 'elapsed_seconds': time.monotonic() - start,
           'external_stop_reason': reason, 'resource_observations': observations,
           'environment_overrides': extra or {}, 'scratch_remaining': [str(p) for p in scratch.rglob('*')],
           **{suffix + '_sha256': sha(output / (name + '.' + suffix))
              for suffix in ['stdout', 'stderr', 'resources']}}
    dump(output / (name + '.result.json'), row)
    print(json.dumps({key: row[key] for key in ['name', 'exit_code', 'elapsed_seconds', 'external_stop_reason']}), flush=True)
    return row
