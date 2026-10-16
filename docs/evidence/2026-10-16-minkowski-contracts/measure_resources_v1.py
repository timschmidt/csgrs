"""Retain all fixed cost trials, including failures and unfavorable results."""
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
OUT = E / 'resources-v1/measurements'
OUT.mkdir()
rows = []
active = None

def terminate(signum, _frame):
    if active is not None and active.poll() is None:
        os.killpg(active.pid, signal.SIGKILL)
    raise SystemExit(128 + signum)

signal.signal(signal.SIGTERM, terminate)
signal.signal(signal.SIGINT, terminate)
groups = {'cube': ['cube-minkowski'],
          'scalar': ['rational', 'unresolved', 'root-square', 'eighth-angle'],
          'expanded': ['concave', 'disconnected', 'hollow']}
for kind in ['timing', 'allocation']:
    for temperature in ['cold', 'warm']:
        for pair in range(3 if kind == 'timing' else 1):
            for group, cases in groups.items():
                for case in cases:
                    variants = ['baseline', 'candidate'] if pair % 2 == 0 else ['candidate', 'baseline']
                    if group == 'expanded': variants = ['candidate']
                    for variant in variants:
                        name = f'{kind}-{temperature}-{pair}-{group}-{case}-{variant}'
                        artifact = OUT / (name + '.artifact.json')
                        command = [str(TARGET / f'{variant}-{group}-{kind}')]
                        if group == 'scalar': command.append(variant)
                        command += [case, temperature, str(artifact)]
                        timed = ['/usr/bin/time', '-v', '-o', str(OUT / (name + '.resources'))] + command
                        active = subprocess.Popen(timed, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
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
                        row = {'name': name, 'group': group, 'variant': variant, 'kind': kind,
                               'temperature': temperature, 'case': case, 'pair': pair,
                               'command': timed, 'exit_code': code,
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
assert len(rows) == 104 and all(row['exit_code'] == 0 for row in rows)
for group, cases in groups.items():
    for case in cases:
        relevant = [row for row in rows if row['group'] == group and row['case'] == case]
        assert len({row['artifact_sha256'] for row in relevant}) == 1, (group, case)
        reference = (OUT / (relevant[0]['name'] + '.artifact.json')).read_bytes()
        assert all((OUT / (row['name'] + '.artifact.json')).read_bytes() == reference for row in relevant)
        if group == 'cube':
            assert reference == Path('/dev/shm/csgrs-minkowski-contracts/baseline-v1/timing-cold-0.json').read_bytes()
sizes = {str(path): path.stat().st_size for path in TARGET.iterdir() if path.is_file()}
(OUT / 'sizes.json').write_text(json.dumps(sizes, indent=2) + '\n')
