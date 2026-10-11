#!/usr/bin/env python3
"""Run the final checkpoint sequentially and preserve the terminal result."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
log = E/'integer-scale-orchestration-v1404.log'
result = E/'integer-scale-orchestration-v1404.json'
assert not log.exists() and not result.exists()
command = [sys.executable, str(E/'validate_integer_scale_checkpoint_v2.py')]
started = time.monotonic()
with log.open('w') as output:
    completed = subprocess.run(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT)
result.write_text(json.dumps({
    'command':command,'exit_code':completed.returncode,'seconds':time.monotonic()-started,
    'source_record_sha256':'32b816463eacd2d2cef2f905c0455557c015117690a8a8dd635229a57726d2b2',
    'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    'validation_helper_sha256':hashlib.sha256((E/'validate_integer_scale_checkpoint_v2.py').read_bytes()).hexdigest(),
    'log_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),
    'scope':'Sequential final-source validations; no runtime admission.'
},indent=2)+'\n')
print(result.read_text(),end='')
sys.exit(completed.returncode)
