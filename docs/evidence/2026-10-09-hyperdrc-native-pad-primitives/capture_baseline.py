"""Capture the injected regression result before changing production geometry."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

record = Path(__file__).resolve().parent
manifest = json.loads((record / 'baseline-start.json').read_bytes())
sources = lambda: {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('crates/hyperdrc').rglob('*.rs') if 'target' not in p.parts}
assert sources() == manifest['source_hashes']
started = time.monotonic()
with (record / 'baseline.log').open('w') as log:
    result = subprocess.run(manifest['command'], stdout=log, stderr=subprocess.STDOUT)
manifest.update(exit_code=result.returncode, elapsed_seconds=time.monotonic()-started, unchanged=sources() == manifest['source_hashes'])
(record / 'baseline.json').write_text(json.dumps(manifest,indent=2)+'\n')
assert manifest['unchanged']
print('Baseline exit:',result.returncode,flush=True)
