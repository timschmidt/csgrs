"""Measure unused root memo omission against the same frozen native baseline."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PREVIOUS = EVIDENCE / 'laurent-memo-sharing'
VALIDATION = EVIDENCE / 'laurent-root-memo-omission'
OUT = EVIDENCE / 'laurent-root-memo-omission-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
status = read(VALIDATION / 'status.json')
assert len(status) == 4 and all(row['exit_code'] == 0 for row in status)
assert read(VALIDATION / 'frontier-comparison.json')['unexpected_different_rows'] == 0
libraries = read(VALIDATION / 'libraries.json')
assert all(digest(Path(row['path'])) == row['sha256'] for row in libraries.values())
assert read(PREVIOUS / 'independent-identities.json')['identities'] == 126
directory = VALIDATION / 'candidate'
directory.mkdir()
shutil.copyfile(VALIDATION / 'candidate-sources.json', directory / 'sources.json')
shutil.copyfile(VALIDATION / 'status.json', directory / 'status.json')
metadata = {'libraries': [{'crate': 'hyperreal', **libraries['baseline']}]}
(VALIDATION / 'baseline-libraries.json').write_text(json.dumps(metadata, indent=2) + '\n')
command = [
    sys.executable, str(EVIDENCE / 'reproduce_normal_form_ownership_cost.py'), str(OUT),
    '--candidate-evidence', str(directory),
    '--candidate-source', str(ROOT / 'target/laurent-root-memo-omission/candidate/source'),
    '--baseline-evidence', str(PREVIOUS / 'baseline'),
    '--baseline-metadata', str(VALIDATION / 'baseline-libraries.json'),
    '--candidate-build-log', str(VALIDATION / 'candidate-library.log'),
    '--candidate-validation-last', 'candidate-library',
    '--expected-source-change', 'crates/hyperreal/src/computable/node/algebra.rs',
    '--expected-source-change', 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs',
]
result = subprocess.run(command, cwd=ROOT)
if OUT.exists():
    (OUT / '.gitignore').write_text('/baseline-time\n/baseline-memory\n/candidate-time\n/candidate-memory\n')
    shutil.copyfile(__file__, OUT / 'wrapper.py')
    (OUT / 'wrapper-status.json').write_text(json.dumps({'command': command, 'exit_code': result.returncode, 'reused_independent_oracle': str(PREVIOUS / 'independent-identities.json'), 'oracle_sha256': digest(PREVIOUS / 'independent-identities.json')}, indent=2) + '\n')
raise SystemExit(result.returncode)
