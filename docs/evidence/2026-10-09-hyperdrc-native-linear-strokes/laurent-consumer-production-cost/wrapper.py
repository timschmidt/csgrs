"""Measure the production-only static consumers with the existing identity probe."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PRODUCTION = EVIDENCE / 'laurent-consumer-production'
VALIDATION = EVIDENCE / 'laurent-consumer-production-validation'
INPUTS = PRODUCTION / 'scalar-cost-inputs'
OUT = EVIDENCE / 'laurent-consumer-production-cost'
WORK = ROOT / 'target/laurent-consumer-production'
BASE = ROOT / 'target/contact-index-directed-width'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(PRODUCTION / 'sources.json')
base = read(Path(source['base_source_record']))
native = read(Path(source['main_source_record']))
hashes = base['source_hashes'] | source['changed_source_hashes'] | source['additional_source_hashes']
status = read(VALIDATION / 'status.json')
assert len(status) == 14 and all(row['exit_code'] == 0 for row in status)
assert read(VALIDATION / 'source-verification.json')['all_candidate_and_main_source_hashes_unchanged']
assert read(EVIDENCE / 'diagnostics-72.json')['tests']['drc-integration-tests']['passed'] == 35
oracle = EVIDENCE / 'laurent-memo-sharing/independent-identities.json'
assert read(oracle)['identities'] == 126
INPUTS.mkdir()
ADAPTERS = WORK / 'cost-adapters'
ADAPTERS.mkdir()
commands = []
libraries = {}


def verify():
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
    assert all(digest(BASE / 'source' / name) == value for name, value in base['source_hashes'].items())
    assert all(digest(WORK / 'source' / name) == value for name, value in hashes.items())


for name, work in [('baseline', BASE), ('candidate', WORK)]:
    verify()
    command = ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '--features', 'serde', '--lib', '--message-format=json']
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(work / 'target')}
    start = time.monotonic()
    with (INPUTS / (name + '-library.log')).open('w') as log:
        result = subprocess.run(command, cwd=work / 'source', env=env, stdout=log, stderr=subprocess.STDOUT)
    row = {'name': name + '-library', 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start}
    commands.append(row)
    (INPUTS / 'status.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    assert result.returncode == 0, name
    events = [json.loads(line) for line in (INPUTS / (name + '-library.log')).read_text().splitlines() if line.startswith('{')]
    library = next(Path(filename) for event in events if event.get('reason') == 'compiler-artifact' and event['target']['name'] == 'hyperreal' for filename in event['filenames'] if filename.endswith('.rlib'))
    libraries[name] = {'crate': 'hyperreal', 'path': str(library), 'bytes': library.stat().st_size, 'sha256': digest(library)}
    adapter = ADAPTERS / name
    adapter.mkdir()
    # The generic runner compares the shared inherited input set. The wrapper
    # separately freezes all 261 restored non-Rust inputs before/after each build
    # and the complete measurement run; none is an input to this scalar probe.
    selected = base['source_hashes'] if name == 'baseline' else {path: hashes[path] for path in base['source_hashes']}
    record = {'source_hashes': selected, 'production_source_record': str(PRODUCTION / 'sources.json'), 'scope': 'Shared inherited input set for the generic scalar probe; restored inputs are verified by the wrapper.'}
    (adapter / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
    (adapter / 'status.json').write_text(json.dumps([row], indent=2) + '\n')
    (INPUTS / (name + '-sources.json.gz')).write_bytes(gzip.compress((adapter / 'sources.json').read_bytes(), mtime=0))
    print(name, 'scalar library built', flush=True)
(INPUTS / 'libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
(INPUTS / 'baseline-libraries.json').write_text(json.dumps({'libraries': [libraries['baseline']]}, indent=2) + '\n')
command = [
    sys.executable, str(EVIDENCE / 'reproduce_normal_form_ownership_cost.py'), str(OUT),
    '--candidate-evidence', str(ADAPTERS / 'candidate'),
    '--candidate-source', str(WORK / 'source'),
    '--baseline-evidence', str(ADAPTERS / 'baseline'),
    '--baseline-metadata', str(INPUTS / 'baseline-libraries.json'),
    '--candidate-build-log', str(INPUTS / 'candidate-library.log'),
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
        'production_validation': str(VALIDATION / 'status.json'),
        'production_validation_sha256': digest(VALIDATION / 'status.json'),
        'all_production_and_baseline_inputs_unchanged': True,
        'reused_independent_oracle': str(oracle), 'oracle_sha256': digest(oracle),
        'source_adapters_archived_at': str(INPUTS),
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
