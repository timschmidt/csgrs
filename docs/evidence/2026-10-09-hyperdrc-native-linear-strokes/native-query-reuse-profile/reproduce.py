import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

candidate = Path('docs/evidence/2026-10-09-hyperdrc-native-linear-strokes/native-query-reuse-compact-candidate')
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=False)
record = json.loads((candidate / 'sources.json').read_text())

def verify():
    assert all(hashlib.sha256(Path(name).read_bytes()).hexdigest() == digest
               for name, digest in record['source_hashes'].items())

verify()
executables = []
for line in (candidate / 'build-board-smoke.log').read_text().splitlines():
    try:
        artifact = json.loads(line)
    except ValueError:
        continue
    if artifact.get('reason') == 'compiler-artifact' and artifact.get('target', {}).get('name') == 'hyperdrc' and artifact.get('executable'):
        executables.append(Path(artifact['executable']))
assert len(executables) == 1
binary = executables[0]
binary_hash = hashlib.sha256(binary.read_bytes()).hexdigest()
command = [str(binary), 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture']
(out / 'reproduce.py').write_text(Path(__file__).read_text())
observations = []
ticks = os.sysconf('SC_CLK_TCK')
thresholds = iter([40, 90, 140, 190])
threshold = next(thresholds, None)
started = time.monotonic()
with (out / 'full-board-smoke.log').open('w') as log:
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    while process.poll() is None:
        try:
            fields = Path(f'/proc/{process.pid}/stat').read_text().rsplit(')', 1)[1].split()
            cpu_seconds = (int(fields[11]) + int(fields[12])) / ticks
        except FileNotFoundError:
            continue
        if threshold is not None and cpu_seconds >= threshold:
            gdb = ['gdb', '-q', '-nx', '-batch', '-iex', 'set sysroot /', '-iex',
                   'set debuginfod enabled off', '-iex', 'set auto-load off',
                   str(binary), '-p', str(process.pid), '-ex', 'set pagination off',
                   '-ex', 'set print frame-arguments none', '-ex',
                   'thread apply all bt 60', '-ex', 'detach']
            trace = out / f'backtrace-at-{threshold}-cpu-seconds.log'
            with trace.open('w') as output:
                code = subprocess.run(gdb, stdout=output, stderr=subprocess.STDOUT,
                                      env={**os.environ, 'DEBUGINFOD_URLS': ''}).returncode
            observations.append(dict(pid=process.pid, cpu_seconds_before_attachment=cpu_seconds,
                                     command=gdb, exit_code=code, log=trace.name))
            (out / 'observations.json').write_text(json.dumps(observations, indent=2) + '\n')
            print('sample', threshold, 'CPU seconds; gdb exit', code, flush=True)
            threshold = next(thresholds, None)
        time.sleep(0.75)
    code = process.wait()
verify()
assert hashlib.sha256(binary.read_bytes()).hexdigest() == binary_hash
(out / 'manifest.json').write_text(json.dumps(dict(
    scope='Four debugger stack samples from a separate full-board run. Timing is perturbed and is not an admission runtime or matched benchmark.',
    source_reference=str(candidate / 'sources.json'), source_unchanged=True,
    binary=str(binary), binary_sha256=binary_hash, binary_unchanged=True,
    command=command, exit_code=code, elapsed_wall_seconds=time.monotonic()-started,
    observations=observations,
), indent=2) + '\n')
print('diagnostic board exit', code, flush=True)
