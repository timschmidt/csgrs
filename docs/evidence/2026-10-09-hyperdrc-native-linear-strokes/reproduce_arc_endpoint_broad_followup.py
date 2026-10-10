"""One fixed ten-pair follow-up, preserving the original five pairs and binaries."""
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
INITIAL = EVIDENCE / 'arc-endpoint-production-replay'
OUT = EVIDENCE / 'arc-endpoint-broad-followup'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
initial = read(INITIAL / 'manifest.json')
diagnostic = read(EVIDENCE / 'diagnostics-81.json')
assert initial['historical_row_deltas'] == 0
assert len(initial['cost_trials']) == 10
assert len(diagnostic['timing_overview']['nonoverlapping_slower_rows']) == 4
sources = {variant: read(EVIDENCE / 'arc-endpoint-certificate' / (variant + '-sources.json')) for variant in ['baseline', 'candidate']}
maps = {}
for variant, record in sources.items():
    path = Path(record['source_hashes_file'])
    assert digest(path) == record['source_hashes_file_sha256']
    maps[variant] = json.loads(gzip.decompress(path.read_bytes()))
native = read(EVIDENCE / 'native-exactness-gate/sources.json')['source_hashes']
canonical = read(INITIAL / 'rows.json')
assert len(canonical) == 231
OUT.mkdir()
shutil.copyfile(__file__, OUT / 'reproduce.py')
plan = {
    'status': 'Fixed before executing the additional trials; no adaptive sample extension.',
    'motivation': 'Four nonoverlapping slower rows in the first five paired production capability processes, including controls outside arc membership. Preserve the initial adverse evidence and compare all 231 workloads using the exact same binaries.',
    'initial_record': str(INITIAL / 'manifest.json'), 'initial_record_sha256': digest(INITIAL / 'manifest.json'),
    'initial_nonoverlapping_slower_rows': diagnostic['timing_overview']['nonoverlapping_slower_rows'],
    'additional_pairs': 10, 'pair_indices': list(range(6, 16)), 'total_pairs_after_followup': 15,
    'order': 'candidate then baseline on even indices; baseline then candidate on odd indices',
    'cpu': 0, 'all_231_workloads_each_process': True,
    'binaries': {variant: initial['variants'][variant]['binaries']['operation'] for variant in sources},
    'decision_rule': 'Review all fifteen samples per variant, preserving every original and new sample. Do not extend sampling again to obtain a favorable result. Persistent substantial regressions require rejection or a separately frozen revised candidate; report-byte identity alone does not admit performance.',
}
(OUT / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
commands, additional = [], []


def verify():
    for variant, record in sources.items():
        source = Path(record['source_directory'])
        assert all(digest(source / name) == value for name, value in maps[variant].items())
        binary = plan['binaries'][variant]
        assert digest(Path(binary['path'])) == binary['sha256']
    assert all(digest(ROOT / name) == value for name, value in native.items())


for index in plan['pair_indices']:
    for variant in (['baseline', 'candidate'] if index % 2 else ['candidate', 'baseline']):
        verify()
        name = f'{variant}-operation-{index}'
        print('start', name, flush=True)
        command = ['taskset', '-c', '0', '/usr/bin/time', '-v', plan['binaries'][variant]['path']]
        start = time.monotonic()
        with (OUT / (name + '.log')).open('w') as log:
            result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=False)
        commands.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
        (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        verify()
        assert result.returncode == 0, name
        rows = []
        for line in (OUT / (name + '.log')).read_text().splitlines():
            if line.startswith('operation_'):
                label, count, elapsed, witness = line.split('\t')
                rows.append({'name': label, 'iterations': int(count), 'elapsed_ns': int(elapsed), 'witness': witness})
        assert [{key: value for key, value in row.items() if key != 'elapsed_ns'} for row in rows] == canonical
        additional.append({'variant': variant, 'index': index, 'rows': rows})
        (OUT / 'additional-trials.json').write_text(json.dumps(additional, indent=2) + '\n')
        print('finish', name, flush=True)
trials = initial['cost_trials'] + additional
assert len(trials) == 30
comparison = []
for witness in canonical:
    values = {variant: [next(row['elapsed_ns'] for row in trial['rows'] if row['name'] == witness['name']) for trial in trials if trial['variant'] == variant] for variant in sources}
    assert all(len(samples) == 15 for samples in values.values())
    baseline, candidate = values['baseline'], values['candidate']
    comparison.append({'name': witness['name'], 'iterations': witness['iterations'], 'baseline_samples_ns': baseline, 'candidate_samples_ns': candidate, 'candidate_over_baseline_median': statistics.median(candidate) / statistics.median(baseline), 'sample_ranges_overlap': max(min(baseline), min(candidate)) <= min(max(baseline), max(candidate)), 'candidate_all_samples_slower': min(candidate) > max(baseline), 'candidate_all_samples_faster': max(candidate) < min(baseline)})
verify()
record = {
    'scope': 'One fixed ten-pair extension to the complete 231-row replay; combined results contain all fifteen samples per version and no discarded trials. Same frozen production binaries, CPU 0, alternating order, no build/test/source edit during measurement.',
    'plan': plan, 'commands': commands, 'additional_trials': additional,
    'comparison': comparison, 'all_sources_and_binaries_unchanged': True,
    'all_231_witnesses_preserved_each_process': True,
    'limitations': 'Min/max ranges are descriptive, not confidence intervals. Timing variation is not automatically attributable to endpoint reuse. This follow-up is bounded and final for these binaries; a retention decision still requires review and the separate native check if the tradeoff passes.',
}
(OUT / 'manifest.json').write_text(json.dumps(record, indent=2) + '\n')
print('Fixed broad-workload follow-up complete; all original samples retained.', flush=True)
