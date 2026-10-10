"""Reuse the independent public normal-form corpus for an early resource gate."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
VALIDATION = EVIDENCE / 'laurent-memo-sharing'
OUT = EVIDENCE / 'laurent-memo-sharing-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
status = read(VALIDATION / 'status.json')
assert len(status) == 8 and all(row['exit_code'] == 0 for row in status)
assert read(VALIDATION / 'frontier-comparison.json')['different_rows'] == 0
libraries = read(VALIDATION / 'libraries.json')
assert all(digest(Path(row['path'])) == row['sha256'] for row in libraries.values())
for variant in ['baseline', 'candidate']:
    directory = VALIDATION / variant
    directory.mkdir()
    shutil.copyfile(VALIDATION / (variant + '-sources.json'), directory / 'sources.json')
    shutil.copyfile(VALIDATION / 'status.json', directory / 'status.json')
metadata = {'libraries': [{'crate': 'hyperreal', **libraries['baseline']}]}
(VALIDATION / 'baseline-libraries.json').write_text(json.dumps(metadata, indent=2) + '\n')
subprocess.run([sys.executable, str(EVIDENCE / 'normal_form_ownership_oracle.py'), str(VALIDATION / 'independent-identities.json')], check=True)
command = [
    sys.executable, str(EVIDENCE / 'reproduce_normal_form_ownership_cost.py'), str(OUT),
    '--candidate-evidence', str(VALIDATION / 'candidate'),
    '--candidate-source', str(ROOT / 'target/laurent-memo-sharing/candidate/source'),
    '--baseline-evidence', str(VALIDATION / 'baseline'),
    '--baseline-metadata', str(VALIDATION / 'baseline-libraries.json'),
    '--candidate-build-log', str(VALIDATION / 'candidate-library.log'),
    '--candidate-validation-last', 'candidate-library',
]
result = subprocess.run(command, cwd=ROOT)
if OUT.exists():
    (OUT / '.gitignore').write_text('/baseline-time\n/baseline-memory\n/candidate-time\n/candidate-memory\n')
    shutil.copyfile(__file__, OUT / 'wrapper.py')
    (OUT / 'wrapper-status.json').write_text(json.dumps({'command': command, 'exit_code': result.returncode}, indent=2) + '\n')
raise SystemExit(result.returncode)
