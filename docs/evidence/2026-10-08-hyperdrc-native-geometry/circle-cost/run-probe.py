from pathlib import Path
import hashlib
import json
import os
import re
import statistics
import subprocess
import time

out = Path('docs/evidence/2026-10-08-hyperdrc-native-geometry/circle-cost')
binary = Path('/tmp/properties-drc-native-circle-cost-bin')
provenance = json.loads((out / 'binary.json').read_text())
assert hashlib.sha256(binary.read_bytes()).hexdigest() == provenance['sha256']
expected = None
runs = []
samples = {}
for trial in range(1, 6):
    name = f'trial-{trial}'
    command = ['/usr/bin/time', '-v', '-o', str(out / f'{name}-resources.log'),
               'taskset', '-c', '0', str(binary)]
    started = time.time()
    result = subprocess.run(command, env=dict(os.environ, HYPERDRC_CIRCLE_ITERATIONS='100'),
                            capture_output=True, text=True)
    (out / f'{name}.jsonl').write_text(result.stdout)
    (out / f'{name}.stderr').write_text(result.stderr)
    assert result.returncode == 0, result.stderr
    rows = [json.loads(row) for row in result.stdout.splitlines()]
    assert len(rows) == 9
    outputs = [{key: value for key, value in row.items() if key != 'elapsed_ns'} for row in rows]
    if expected is None:
        expected = outputs
    assert outputs == expected
    for row in rows:
        samples.setdefault((row['fixture'], row['mode']), []).append(row['elapsed_ns'] / row['iterations'])
    resource = (out / f'{name}-resources.log').read_text()
    rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', resource).group(1))
    runs.append({'trial': trial, 'command': command, 'exit_code': 0,
                 'started_unix': started, 'finished_unix': time.time(),
                 'stdout': f'{name}.jsonl', 'resources': f'{name}-resources.log',
                 'process_peak_rss_kib': rss})
    (out / 'runs.json').write_text(json.dumps(runs, indent=2) + '\n')
    print(name, '9 oracle-checked rows; stable report checksums and sizes', flush=True)
summary = [{'fixture': fixture, 'mode': mode, 'ns_per_observation': values,
            'median_ns': statistics.median(values)}
           for (fixture, mode), values in sorted(samples.items())]
(out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2), flush=True)
