"""Retain all old/new children and compare exact artifacts across both epochs."""
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
TARGET = Path(sys.argv[1])
OUT = E / 'resources-v1/measurements'
OUT.mkdir()
rows = []
active = None


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def terminate(signum, _frame):
    if active is not None and active.poll() is None:
        os.killpg(active.pid, signal.SIGKILL)
    raise SystemExit(128 + signum)


signal.signal(signal.SIGTERM, terminate)
signal.signal(signal.SIGINT, terminate)
sizes = {f'{variant}-{kind}': {'bytes': (TARGET / f'{variant}-{kind}').stat().st_size,
                              'sha256': sha(TARGET / f'{variant}-{kind}')}
         for variant in ['baseline', 'candidate'] for kind in ['timing', 'allocation']}
(OUT / 'sizes.json').write_text(json.dumps(sizes, indent=2) + '\n')
for kind in ['timing', 'allocation']:
    for temperature in ['cold', 'warm']:
        for sample in range(3 if kind == 'timing' else 1):
            for case in ['overlap', 'coincident', 'touching', 'full26', 'merged26']:
                variants = ['baseline', 'candidate'] if case in ['overlap', 'coincident', 'touching'] else ['candidate']
                for variant in variants:
                    name = f'{variant}-{kind}-{temperature}-{sample}-{case}'
                    artifact = OUT / (name + '.artifact.json')
                    binary = TARGET / f'{variant}-{kind}'
                    expected = sizes[binary.name]['sha256']
                    assert sha(binary) == expected
                    command = ['/usr/bin/time', '-v', '-o', str(OUT / (name + '.resources')),
                               str(binary), case, temperature, str(artifact)]
                    started = time.time_ns()
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
                    finished = time.time_ns()
                    assert sha(binary) == expected
                    (OUT / (name + '.stdout')).write_bytes(stdout)
                    (OUT / (name + '.stderr')).write_bytes(stderr)
                    row = {'name': name, 'variant': variant, 'kind': kind, 'case': case,
                           'temperature': temperature, 'sample': sample, 'command': command,
                           'exit_code': code, 'started_unix_nanos': started, 'finished_unix_nanos': finished,
                           'binary_sha256': expected, 'binary_unchanged': True,
                           'stdout_sha256': hashlib.sha256(stdout).hexdigest(),
                           'stderr_sha256': hashlib.sha256(stderr).hexdigest(),
                           'resources_sha256': sha(OUT / (name + '.resources'))}
                    if code == 0:
                        try:
                            data = list(csv.DictReader(io.StringIO(stdout.decode())))
                            assert len(data) == 1
                            row['measurement'] = data[0]
                            assert data[0]['case'] == case and data[0]['temperature'] == temperature
                            assert data[0]['certainty'] == 'certified_local'
                            row['artifact_sha256'] = sha(artifact)
                        except (AssertionError, KeyError, ValueError, OSError) as error:
                            row['measurement_error'] = repr(error)
                    rows.append(row)
                    (OUT / 'result.json').write_text(json.dumps({'rows': rows}, indent=2) + '\n')
                    print(name, code, flush=True)
assert len(rows) == 64 and all(row['exit_code'] == 0 and 'measurement_error' not in row for row in rows)
original = E / 'baseline-probes-v1/measurements'
initial = json.loads((original / 'result.json').read_text())['rows']
assert len(initial) == 24 and all(row['exit_code'] == 0 for row in initial)
artifacts = {}
for case in ['overlap', 'coincident', 'touching', 'full26', 'merged26']:
    relevant = [(OUT, row) for row in rows if row['case'] == case]
    relevant += [(original, row) for row in initial if row['case'] == case]
    reference = (relevant[0][0] / (relevant[0][1]['name'] + '.artifact.json')).read_bytes()
    expected = hashlib.sha256(reference).hexdigest()
    for folder, row in relevant:
        assert row['artifact_sha256'] == expected
        assert (folder / (row['name'] + '.artifact.json')).read_bytes() == reference
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    artifacts[case] = {'sha256': expected, 'bytes': len(reference), 'rows': len(relevant),
                       'original_baseline_rows': sum(folder == original for folder, _ in relevant)}
(OUT / 'artifact-comparison.json').write_text(json.dumps(artifacts, indent=2) + '\n')
assert sum(row['kind'] == 'timing' for row in rows) == 48
assert sum(row['kind'] == 'allocation' for row in rows) == 16
print(json.dumps({'children': len(rows), 'original_baseline_children': len(initial),
                  'all_artifacts_identical_within_case': True}))
