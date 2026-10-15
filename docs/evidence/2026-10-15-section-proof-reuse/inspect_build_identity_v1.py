"""Check the actual compiled dependency metadata of the paired probe libraries."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
out = E / 'build-identity-v1'
out.mkdir(exist_ok=False)
records = {}
for label, phase in [('baseline', 'baseline-v2'), ('candidate', 'candidate-v1')]:
    library = ROOT / 'target/section-proof-reuse' / phase / 'libcsgrs.rlib'
    command = ['rustc', '-Zls=root', str(library)]
    result = subprocess.run(command, capture_output=True, env=os.environ | {'RUSTC_BOOTSTRAP': '1'})
    (out / (label + '.stdout')).write_bytes(result.stdout)
    (out / (label + '.stderr')).write_bytes(result.stderr)
    records[label] = {'command': command, 'exit_code': result.returncode,
                      'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
                      'metadata_sha256': hashlib.sha256(result.stdout).hexdigest()}
    assert result.returncode == 0
texts = [(out / (label + '.stdout')).read_text() for label in ['baseline', 'candidate']]
same = texts[0].split('=External Dependencies=')[1] == texts[1].split('=External Dependencies=')[1]
report = {'passed': same, 'records': records,
          'claim': 'The complete actual external-dependency metadata matches byte for byte, including crate names, encoded hashes, host hashes, kinds and visibility. Inputs also hash-guard the exact native library bytes and both source/command records. The CSGRS root library itself intentionally changes.'}
(out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({'passed': same}))
assert same
