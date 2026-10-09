"""Resolve the short rational-only trial's construction/query timing variation.

No source edits or builds occur here. Reuse the exact validated timing binaries,
four times as many newly constructed point pairs, five fresh processes each,
CPU 0, alternating order. Keep all samples and independent exact witnesses.
"""
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import time

root = Path.cwd() / 'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes'
prior = root / 'point-equality-rational-cost'
out = root / 'point-equality-rational-long-cost'
out.mkdir(exist_ok=False)
record = json.loads((prior / 'manifest.json').read_text())
source = json.loads((root / 'point-equality-rational-candidate/sources.json').read_text())
base = json.loads((prior / 'baseline-libraries.json').read_text())
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert all(row['exit_code'] == 0 for row in record['commands'])
def verify():
    assert all(digest(Path(p)) == h for p, h in source['source_hashes'].items())
    assert all(digest(Path(row['path'])) == row['sha256'] for row in base['libraries'])
    assert all(digest(prior / (variant + '-time')) == record['binaries'][variant + '-time']['sha256']
               for variant in ['baseline', 'candidate'])
verify()
shutil.copy2(__file__, out / 'reproduce.py')
commands, trials = [], []
for index in range(1, 6):
    for variant in (('baseline', 'candidate') if index % 2 else ('candidate', 'baseline')):
        verify()
        name = f'{index}-{variant}'
        command = ['taskset', '-c', '0', '/usr/bin/time', '-v', str(prior / (variant + '-time')), '16384']
        start = time.monotonic()
        with (out / (name + '.jsonl')).open('w') as stdout, (out / (name + '.log')).open('w') as stderr:
            code = subprocess.run(command, stdout=stdout, stderr=stderr).returncode
        commands.append(dict(name=name, command=command, exit_code=code, wall_seconds=time.monotonic() - start))
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        verify()
        assert code == 0, name
        rows = [json.loads(line) for line in (out / (name + '.jsonl')).read_text().splitlines()]
        assert len(rows) == 112
        trials.append(dict(index=index, variant=variant, rows=rows))
        print(name, 'passed', flush=True)
witness = lambda t:[r for r in t['rows'] if 'witness' in r or 'fixture' in r]
assert len(witness(trials[0])) == 56 and all(witness(t) == witness(trials[0]) for t in trials)
summaries = []
for name in [r['name'] for r in trials[0]['rows'] if 'name' in r]:
    variants = {}
    for variant in ['baseline', 'candidate']:
        values = [next(r['nanos'] for r in t['rows'] if r.get('name') == name) for t in trials if t['variant'] == variant]
        variants[variant] = dict(samples_ns=values, median_ns=statistics.median(values))
    summaries.append(dict(name=name, variants=variants))
manifest = dict(scope=__doc__, prior_measurements=str(prior / 'manifest.json'),
                unchanged_source=True, source_record=str(root / 'point-equality-rational-candidate/sources.json'),
                binary_records={v:record['binaries'][v + '-time'] for v in ['baseline', 'candidate']},
                commands=commands, trials=trials, summaries=summaries, exact_witnesses_equal=True)
(out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('larger matched point timings complete', flush=True)
