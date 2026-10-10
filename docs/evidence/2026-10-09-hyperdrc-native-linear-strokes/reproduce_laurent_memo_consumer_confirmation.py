"""One fixed follow-up: ten fresh timing pairs, unchanged binaries and workload."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
COST = EVIDENCE / 'laurent-memo-consumer-cost'
OUT = EVIDENCE / 'laurent-memo-consumer-confirmation'
FOCUSED = EVIDENCE / 'laurent-memo-consumer'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
original = read(COST / 'manifest.json')
assert read(COST / 'wrapper-status.json')['exit_code'] == 0
records = {name: read(FOCUSED / (name + '-sources.json')) for name in ['baseline', 'candidate']}
hashes = {name: json.loads(gzip.decompress(Path(record['source_hashes_file']).read_bytes())) for name, record in records.items()}
directories = {'baseline': ROOT / 'target/laurent-memo-index/baseline/source', 'candidate': ROOT / 'target/laurent-memo-consumer/candidate/source'}
native = read(Path(records['baseline']['main_source_record']))
binaries = {name: COST / (name + '-time') for name in directories}
expected_witnesses = [row for row in original['trials'][0]['rows'] if 'witness' in row or 'fixture' in row]
OUT.mkdir()
shutil.copyfile(__file__, OUT / 'reproduce.py')
commands = []
trials = []


def verify():
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
    for variant, files in hashes.items():
        assert all(digest(directories[variant] / name) == value for name, value in files.items())
    assert all(digest(path) == original['binaries'][name + '-time']['sha256'] for name, path in binaries.items())


for index in range(1, 11):
    # Reverse the first pair relative to the initial experiment.
    for variant in (('candidate', 'baseline') if index % 2 else ('baseline', 'candidate')):
        verify()
        label = f'time-{index}-{variant}'
        command = ['taskset', '-c', '0', '/usr/bin/time', '-v', str(binaries[variant]), '4096']
        start = time.monotonic()
        with (OUT / (label + '.jsonl')).open('w') as output, (OUT / (label + '.log')).open('w') as log:
            result = subprocess.run(command, cwd=ROOT, stdout=output, stderr=log)
        commands.append({'name': label, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
        (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        assert result.returncode == 0, label
        verify()
        rows = [json.loads(line) for line in (OUT / (label + '.jsonl')).read_text().splitlines()]
        assert len(rows) == 48
        assert [row for row in rows if 'witness' in row or 'fixture' in row] == expected_witnesses
        assert all(row['iterations'] == 4096 for row in rows if 'name' in row)
        trials.append({'variant': variant, 'index': index, 'rows': rows})
        print(label, 'passed', flush=True)
names = [row['name'] for row in trials[0]['rows'] if 'name' in row]
summaries = []
for name in names:
    samples = {variant: [next(row['nanos'] for row in trial['rows'] if row.get('name') == name) for trial in trials if trial['variant'] == variant] for variant in directories}
    medians = {variant: statistics.median(values) for variant, values in samples.items()}
    summaries.append({'name': name, 'samples_batch_ns': samples, 'medians_batch_ns': medians, 'ratio': medians['candidate'] / medians['baseline'], 'nonoverlapping_ranges': max(samples['baseline']) < min(samples['candidate']) or max(samples['candidate']) < min(samples['baseline'])})
assert len(summaries) == 24
manifest = {
    'scope': 'Exactly ten additional fresh process pairs, same two timing executables, same 4,096 fixtures per row, same CPU 0. All 24 rows retained. This follow-up investigates the initial 3.3% construction regression; it does not replace the initial samples or rerun allocation/build/tests.',
    'initial_record': str(COST / 'manifest.json'), 'initial_record_sha256': digest(COST / 'manifest.json'),
    'commands': commands, 'trials': trials, 'summaries': summaries,
    'witness_rows_preserved': 24, 'source_and_binary_hashes_unchanged': True,
    'binaries': {name: {'path': str(path), 'sha256': digest(path)} for name, path in binaries.items()},
}
(OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Fixed follow-up complete; construction concern:', next(row for row in summaries if row['name'] == 'radical_laurent_transport_construction'), flush=True)
