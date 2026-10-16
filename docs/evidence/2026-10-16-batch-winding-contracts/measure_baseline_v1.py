"""Record every pre-edit timing/allocation child, including any failures."""
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

E = Path(__file__).resolve().parent
TARGET = Path(sys.argv[1])
OUT = E / 'baseline-probes-v1/measurements'
OUT.mkdir()
rows = []
active = None


def terminate(signum, _frame):
    if active is not None and active.poll() is None:
        os.killpg(active.pid, signal.SIGKILL)
    raise SystemExit(128 + signum)


signal.signal(signal.SIGTERM, terminate)
signal.signal(signal.SIGINT, terminate)
for kind in ['timing', 'allocation']:
    for temperature in ['cold', 'warm']:
        for sample in range(3 if kind == 'timing' else 1):
            for case in ['overlap', 'coincident', 'touching']:
                name = f'{kind}-{temperature}-{sample}-{case}'
                artifact = OUT / (name + '.artifact.json')
                command = ['/usr/bin/time', '-v', '-o', str(OUT / (name + '.resources')),
                           str(TARGET / kind), case, temperature, str(artifact)]
                active = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                          start_new_session=True)
                try:
                    stdout, stderr = active.communicate(timeout=120)
                    code = active.returncode
                except subprocess.TimeoutExpired:
                    os.killpg(active.pid, signal.SIGKILL)
                    stdout, stderr = active.communicate()
                    code = 124
                finally:
                    active = None
                (OUT / (name + '.stdout')).write_bytes(stdout)
                (OUT / (name + '.stderr')).write_bytes(stderr)
                row = {'name': name, 'kind': kind, 'case': case, 'temperature': temperature,
                       'sample': sample, 'command': command, 'exit_code': code,
                       'stdout_sha256': hashlib.sha256(stdout).hexdigest(),
                       'stderr_sha256': hashlib.sha256(stderr).hexdigest(),
                       'resources_sha256': hashlib.sha256((OUT / (name + '.resources')).read_bytes()).hexdigest()}
                if code == 0:
                    data = list(csv.DictReader(io.StringIO(stdout.decode())))
                    assert len(data) == 1
                    row['measurement'] = data[0]
                    row['artifact_sha256'] = hashlib.sha256(artifact.read_bytes()).hexdigest()
                rows.append(row)
                (OUT / 'result.json').write_text(json.dumps({'rows': rows}, indent=2) + '\n')
                print(name, code, flush=True)
assert len(rows) == 24 and all(row['exit_code'] == 0 for row in rows)
for case in ['overlap', 'coincident', 'touching']:
    relevant = [row for row in rows if row['case'] == case]
    assert len({row['artifact_sha256'] for row in relevant}) == 1
    reference = (OUT / (relevant[0]['name'] + '.artifact.json')).read_bytes()
    assert all((OUT / (row['name'] + '.artifact.json')).read_bytes() == reference for row in relevant)
