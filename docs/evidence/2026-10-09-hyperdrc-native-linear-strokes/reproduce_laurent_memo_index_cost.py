"""Measure the fresh production memo-index pair after its frontier gate."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
VALIDATION = EVIDENCE / 'laurent-memo-index'
OUT = EVIDENCE / 'laurent-memo-index-cost'
WORK = ROOT / 'target/laurent-memo-index'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
status = read(VALIDATION / 'status.json')
assert len(status) == 8 and all(row['exit_code'] == 0 for row in status)
assert read(VALIDATION / 'validation-complete.json')['frontier_rows_identical'] == 19140
assert read(VALIDATION / 'frontier-comparison.json')['different_rows'] == 0
assert read(VALIDATION / 'historical-baseline-comparison.json')['different_rows'] == 0
records = {name: read(VALIDATION / (name + '-sources.json')) for name in ['baseline', 'candidate']}
hashes = {name: json.loads(gzip.decompress(Path(record['source_hashes_file']).read_bytes())) for name, record in records.items()}
native = read(Path(records['baseline']['main_source_record']))
libraries = read(VALIDATION / 'libraries.json')
assert all(digest(Path(row['path'])) == row['sha256'] for row in libraries.values())
oracle = EVIDENCE / 'laurent-memo-sharing/independent-identities.json'
assert read(oracle)['identities'] == 126
adapters = WORK / 'cost-adapters'
adapters.mkdir()
adapter_records = {}
for name in ['baseline', 'candidate']:
    adapter = adapters / name
    adapter.mkdir()
    (adapter / 'sources.json').write_text(json.dumps({'source_hashes': hashes[name], 'compact_source_record': str(VALIDATION / (name + '-sources.json'))}, indent=2) + '\n')
    (adapter / 'status.json').write_text(json.dumps([row for row in status if row['name'].startswith(name + '-')], indent=2) + '\n')
    adapter_records[name] = {filename: {'path': str(adapter / filename), 'sha256': digest(adapter / filename)} for filename in ['sources.json', 'status.json']}
metadata = VALIDATION / 'baseline-libraries.json'
metadata.write_text(json.dumps({'libraries': [{'crate': 'hyperreal', **libraries['baseline']}]}, indent=2) + '\n')


def verify():
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
    for variant, entries in hashes.items():
        assert all(digest(WORK / variant / 'source' / name) == value for name, value in entries.items())


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
        'fresh_production_baseline': True,
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
