from pathlib import Path
import hashlib
import json
import os
import re
import statistics
import subprocess
import time

out = Path('docs/evidence/2026-10-08-hyperdrc-native-geometry/measurement-probe')
iterations = 1000
expected = None
runs = []
samples = {}

for trial in range(1, 6):
    modes = ['baseline', 'candidate'] if trial % 2 else ['candidate', 'baseline']
    for mode in modes:
        name = f'trial-{trial}-{mode}'
        binary = Path(f'/tmp/properties-drc-measurement-{mode}-bin')
        provenance = json.loads((out / f'{mode}-binary.json').read_text())
        actual_hash = hashlib.sha256(binary.read_bytes()).hexdigest()
        assert actual_hash == provenance['sha256'], (mode, provenance)
        command = ['/usr/bin/time', '-v', '-o', str(out / f'{name}-resources.log'),
                   'taskset', '-c', '0', str(binary)]
        started = time.time()
        result = subprocess.run(command, env=dict(os.environ, DRC_MEASUREMENT_ITERATIONS=str(iterations)),
                                capture_output=True, text=True)
        (out / f'{name}.jsonl').write_text(result.stdout)
        (out / f'{name}.stderr').write_text(result.stderr)
        assert result.returncode == 0, (command, result.stderr)
        rows = [json.loads(row) for row in result.stdout.splitlines()]
        assert len(rows) == 6
        reports = [{key: value for key, value in row.items() if key != 'elapsed_ns'} for row in rows]
        if expected is None:
            expected = reports
        assert reports == expected, (mode, trial, reports, expected)
        for row in rows:
            samples.setdefault((row['fixture'], row['mode'], mode), []).append(row['elapsed_ns'] / iterations)
        resources = (out / f'{name}-resources.log').read_text()
        peak_rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', resources).group(1))
        runs.append({'trial': trial, 'mode': mode, 'command': command, 'exit_code': result.returncode,
                     'stdout': f'{name}.jsonl', 'resources': f'{name}-resources.log',
                     'process_peak_rss_kib': peak_rss, 'started_unix': started, 'finished_unix': time.time()})
        (out / 'runs.json').write_text(json.dumps(runs, indent=2) + '\n')
        print(f'{name}: 6 rows, preserved reports, exit 0', flush=True)

summary = []
for fixture, mode in sorted({(key[0], key[1]) for key in samples}):
    baseline = samples[fixture, mode, 'baseline']
    candidate = samples[fixture, mode, 'candidate']
    baseline_median = statistics.median(baseline)
    candidate_median = statistics.median(candidate)
    summary.append({'fixture': fixture, 'mode': mode,
                    'baseline_ns_per_query': baseline, 'candidate_ns_per_query': candidate,
                    'baseline_median_ns': baseline_median, 'candidate_median_ns': candidate_median,
                    'candidate_delta_percent': 100 * (candidate_median / baseline_median - 1)})
(out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2), flush=True)
