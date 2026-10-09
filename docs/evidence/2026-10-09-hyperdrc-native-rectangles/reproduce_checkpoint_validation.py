"""Run focused, consumer and full DRC gates sequentially in one target tree.

Usage: python3 reproduce_checkpoint_validation.py OUTPUT_ROOT
Do not overlap builds: csgrs' un-hashed rlib is shared by feature variants.
"""
import json
from pathlib import Path
import subprocess
import sys

record = Path(__file__).resolve().parent
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
steps = []
for script, directory in [
    ('reproduce_focused.py', 'expanded-fixed'),
    ('reproduce_consumer_validation.py', 'consumer-validation'),
    ('reproduce_validation.py', 'validation'),
]:
    command = ['python3',str(record/script),str(out/directory)]
    code = subprocess.run(command).returncode
    steps.append(dict(command=command,exit_code=code))
    (out/'checkpoint-validation.json').write_text(json.dumps(steps,indent=2)+'\n')
    if code: raise SystemExit(code)
