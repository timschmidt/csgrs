"""Run the downstream and full DRC phases sequentially, after focused tests.

Usage: python3 reproduce_final_validation.py OUTPUT_ROOT
"""
import json
from pathlib import Path
import subprocess
import sys

record = Path(__file__).resolve().parent
out = Path(sys.argv[1]).resolve(); out.mkdir(parents=True,exist_ok=True)
status = []
for script,folder in [('reproduce_consumer_validation.py','consumer-validation'),('reproduce_validation.py','validation')]:
    command = [sys.executable,str(record/script),str(out/folder)]
    result = subprocess.run(command)
    status.append(dict(command=command,exit_code=result.returncode))
    (out/'final-validation.json').write_text(json.dumps(status,indent=2)+'\n')
    if result.returncode: break
raise SystemExit(status[-1]['exit_code'])
