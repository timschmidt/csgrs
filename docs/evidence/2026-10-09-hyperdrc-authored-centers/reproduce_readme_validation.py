"""Recheck README integration after the prose addition; Rust is unchanged."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=False)
paths = [Path('crates/hyperdrc/README.md'), Path('crates/hyperdrc/tests/readme.rs')]
sources = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
command = ['cargo', 'test', '--locked', '-p', 'hyperdrc', '--release', '--test', 'readme']
started = time.time()
with (out / 'readme.log').open('w') as log:
    result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
current = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
(out / 'manifest.json').write_text(json.dumps(dict(command=command, exit_code=result.returncode,
    elapsed_seconds=time.time()-started, source_hashes=sources, unchanged=sources == current), indent=2) + '\n')
assert sources == current
raise SystemExit(result.returncode)
