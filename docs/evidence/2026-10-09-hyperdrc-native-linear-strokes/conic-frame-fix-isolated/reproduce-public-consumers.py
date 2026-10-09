import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

evidence = Path(__file__).resolve().parent
work = Path('/tmp/drc-linear-conic-frame-fix-isolated')
source = work / 'source'
record = json.loads((evidence / 'sources.json').read_text())


def verify():
    assert all(hashlib.sha256((source / name).read_bytes()).hexdigest() == digest
               for name, digest in record['source_hashes'].items())


command = ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release',
           '--test', 'hypercurve_rational_bezier',
           '--test', 'hypercurve_curve_intersection',
           '--test', 'hypercurve_bezier_algebraic_parameter',
           '--test', 'hypercurve_arc_bezier', '--', '--test-threads=2']
verify()
started = time.monotonic()
with (evidence / 'public-consumers.log').open('w') as log:
    code = subprocess.run(command, cwd=source,
                          env={**os.environ, 'CARGO_TARGET_DIR': str(work / 'target'),
                               'CCACHE_DIR': str(work / 'ccache')},
                          stdout=log, stderr=subprocess.STDOUT).returncode
elapsed = time.monotonic() - started
verify()
(evidence / 'public-consumers-status.json').write_text(json.dumps(dict(
    command=command, exit_code=code, elapsed_seconds=elapsed,
    source_unchanged=True), indent=2) + '\n')
print('public consumers', code, flush=True)
raise SystemExit(code)
