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
CASES = ['readme-truetype', 'bench-truetype', 'readme-hershey', 'bench-hershey']
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
assert len(rows) == 64 and all(row['exit_code'] == 0 for row in rows)
for case in CASES:
    for variant in ['baseline', 'candidate']:
        hashes = {row['geometry_sha256'] for row in rows if row['case'] == case and row['variant'] == variant}
        assert len(hashes) == 1, (case, variant, hashes)
    if 'hershey' in case:
        expected = Path('/dev/shm/csgrs-text-contracts/reproduction-v1/fixtures') / (case + '.json')
        digest = hashlib.sha256(expected.read_bytes()).hexdigest()
        assert all(row['geometry_sha256'] == digest for row in rows if row['case'] == case)
sizes = {str(path): path.stat().st_size for path in TARGET.iterdir() if path.is_file()}
(OUT / 'sizes.json').write_text(json.dumps(sizes, indent=2) + '\n')
preservation = []
for case in ['custom-hershey', 'negative-hershey']:
    expected = Path('/dev/shm/csgrs-text-contracts/reproduction-v1/fixtures') / (case + '.json')
    for variant in ['baseline', 'candidate']:
        name = f'preservation-{case}-{variant}'
        artifact = OUT / (name + '.geometry.json')
        command = [str(TARGET / f'{variant}-timing'), case, 'cold', str(artifact)]
        result = subprocess.run(command, capture_output=True, text=True, timeout=120)
        (OUT / (name + '.stdout')).write_text(result.stdout)
        (OUT / (name + '.stderr')).write_text(result.stderr)
        row = {'command': command, 'exit_code': result.returncode, 'case': case,
               'variant': variant, 'byte_equal': result.returncode == 0 and artifact.read_bytes() == expected.read_bytes()}
        preservation.append(row)
        (OUT / 'preservation.json').write_text(json.dumps(preservation, indent=2) + '\n')
assert all(row['byte_equal'] for row in preservation)
