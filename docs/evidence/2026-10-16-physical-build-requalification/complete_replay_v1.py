"""Launch the frozen report comparison after the original validation commands finish."""
import hashlib
import json
from pathlib import Path
import runpy
import time

directory = Path(__file__).resolve().parent
runner = directory / 'replay_v1.py'
digest = hashlib.sha256(runner.read_bytes()).hexdigest()
print('Waiting for native validation before complete report replay.', flush=True)
while not (directory / 'validation-v2/result.json').exists():
    time.sleep(1)
assert json.loads((directory / 'validation-v2/result.json').read_text())['complete']
assert hashlib.sha256(runner.read_bytes()).hexdigest() == digest
runpy.run_path(str(runner), run_name='__main__')
