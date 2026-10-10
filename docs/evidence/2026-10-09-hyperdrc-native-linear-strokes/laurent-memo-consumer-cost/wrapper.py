"""Measure static consumers against the verified production baseline."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
VALIDATION = EVIDENCE / 'laurent-memo-consumer'
OUT = EVIDENCE / 'laurent-memo-consumer-cost'
WORK = ROOT / 'target/laurent-memo-consumer'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
status = read(VALIDATION / 'status.json')
assert len(status) == 4 and all(row['exit_code'] == 0 for row in status)
assert read(VALIDATION / 'validation-complete.json')['frontier_rows_identical'] == 19140
assert read(VALIDATION / 'frontier-comparison.json')['different_rows'] == 0
baseline_status = read(VALIDATION / 'baseline-status.json')
assert len(baseline_status) == 4 and all(row['exit_code'] == 0 for row in baseline_status)
records = {name: read(VALIDATION / (name + '-sources.json')) for name in ['baseline', 'candidate']}
hashes = {name: json.loads(gzip.decompress(Path(record['source_hashes_file']).read_bytes())) for name, record in records.items()}
native = read(Path(records['baseline']['main_source_record']))
libraries = read(VALIDATION / 'libraries.json')
assert all(digest(Path(row['path'])) == row['sha256'] for row in libraries.values())
oracle = EVIDENCE / 'laurent-memo-sharing/independent-identities.json'
assert read(oracle)['identities'] == 126
source_directories = {'baseline': ROOT / 'target/laurent-memo-index/baseline/source', 'candidate': WORK / 'candidate/source'}
adapters = WORK / 'cost-adapters'
adapters.mkdir()
adapter_records = {}
for name in ['baseline', 'candidate']:
    adapter = adapters / name
    adapter.mkdir()
    (adapter / 'sources.json').write_text(json.dumps({'source_hashes': hashes[name], 'compact_source_record': str(VALIDATION / (name + '-sources.json'))}, indent=2) + '\n')
    (adapter / 'status.json').write_text(json.dumps(baseline_status if name == 'baseline' else status, indent=2) + '\n')
    adapter_records[name] = {filename: {'path': str(adapter / filename), 'sha256': digest(adapter / filename)} for filename in ['sources.json', 'status.json']}
metadata = VALIDATION / 'baseline-libraries.json'
metadata.write_text(json.dumps({'libraries': [{'crate': 'hyperreal', **libraries['baseline']}]}, indent=2) + '\n')


def verify():
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
    for variant, entries in hashes.items():
        assert all(digest(source_directories[variant] / name) == value for name, value in entries.items())


command = [
    sys.executable, str(EVIDENCE / 'reproduce_normal_form_ownership_cost.py'), str(OUT),
    '--candidate-evidence', str(adapters / 'candidate'),
    '--candidate-source', str(WORK / 'candidate/source'),
    '--baseline-evidence', str(adapters / 'baseline'),
    '--baseline-metadata', str(metadata),
    '--candidate-build-log', str(VALIDATION / 'candidate-library.log'),
    '--candidate-validation-last', 'candidate-library',
]
verify()
result = subprocess.run(command, cwd=ROOT)
verify()
if OUT.exists():
    (OUT / '.gitignore').write_text('/baseline-time\n/baseline-memory\n/candidate-time\n/candidate-memory\n')
    shutil.copyfile(__file__, OUT / 'wrapper.py')
    (OUT / 'wrapper-status.json').write_text(json.dumps({
        'command': command, 'exit_code': result.returncode,
        'production_baseline': 'Unchanged source/library from diagnostics-68; new measurement processes',
        'baseline_validation_reused': str(EVIDENCE / 'diagnostics-68.json'),
        'frontier_record': str(VALIDATION / 'frontier-comparison.json'),
        'frontier_record_sha256': digest(VALIDATION / 'frontier-comparison.json'),
        'reused_independent_oracle': str(oracle), 'oracle_sha256': digest(oracle),
        'source_adapters': adapter_records,
        'all_baseline_candidate_and_main_sources_unchanged': True,
    }, indent=2) + '\n')
if result.returncode == 0:
    retained = WORK / 'measured-libraries'
    retained.mkdir()
    for name, row in libraries.items():
        assert digest(Path(row['path'])) == row['sha256']
        path = retained / (name + '.rlib')
        shutil.copyfile(row['path'], path)
        assert digest(path) == row['sha256']
        row['retained_path'] = str(path)
    (OUT / 'retained-libraries.json').write_text(json.dumps({'libraries': libraries}, indent=2) + '\n')
raise SystemExit(result.returncode)
