"""Keep every predetermined paired trial, diagnostic and serialized output."""
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
TARGET = Path(sys.argv[1])
OUT = E / 'resources-v1/measurements'
OUT.mkdir()
CASES = ['linear-8', 'grid-4x4', 'arc-12', 'feature-merge-22', 'merge-duplicate']
rows = []
for kind in ['timing', 'allocation']:
    for temperature in ['cold', 'warm']:
        for pair in range(3 if kind == 'timing' else 1):
            for case in CASES:
                variants = ['baseline', 'candidate'] if pair % 2 == 0 else ['candidate', 'baseline']
                for variant in variants:
                    name = f'{kind}-{temperature}-{pair}-{case}-{variant}'
                    command = [str(TARGET / f'{variant}-{kind}'), case, temperature, str(OUT / (name + '.geometry.json'))]
                    result = subprocess.run(command, capture_output=True, text=True, timeout=120)
                    (OUT / (name + '.stdout')).write_text(result.stdout)
                    (OUT / (name + '.stderr')).write_text(result.stderr)
                    row = {'name': name, 'variant': variant, 'kind': kind, 'temperature': temperature,
                           'case': case, 'pair': pair, 'command': command, 'exit_code': result.returncode,
                           'stdout_sha256': hashlib.sha256(result.stdout.encode()).hexdigest(),
                           'stderr_sha256': hashlib.sha256(result.stderr.encode()).hexdigest()}
                    if result.returncode == 0:
                        data = list(csv.DictReader(io.StringIO(result.stdout)))
                        assert len(data) == 1
                        row['measurement'] = data[0]
                        artifact = OUT / (name + '.geometry.json')
                        row['geometry_sha256'] = hashlib.sha256(artifact.read_bytes()).hexdigest()
                    rows.append(row)
                    (OUT / 'result.json').write_text(json.dumps({'rows': rows}, indent=2) + '\n')
                    print(name, result.returncode, flush=True)
assert len(rows) == 80 and all(row['exit_code'] == 0 for row in rows)
for case in CASES:
    for variant in ['baseline', 'candidate']:
        hashes = {row['geometry_sha256'] for row in rows if row['case'] == case and row['variant'] == variant}
        assert len(hashes) == 1, (case, variant, hashes)
    expected = Path('/dev/shm/csgrs-distribution-contracts/reproduction-v2/fixtures') / (case + '.json')
    digest = hashlib.sha256(expected.read_bytes()).hexdigest()
    assert all(row['geometry_sha256'] == digest for row in rows if row['case'] == case)
    assert all((OUT / (row['name'] + '.geometry.json')).read_bytes() == expected.read_bytes() for row in rows if row['case'] == case)
sizes = {str(path): path.stat().st_size for path in TARGET.iterdir() if path.is_file()}
(OUT / 'sizes.json').write_text(json.dumps(sizes, indent=2) + '\n')
