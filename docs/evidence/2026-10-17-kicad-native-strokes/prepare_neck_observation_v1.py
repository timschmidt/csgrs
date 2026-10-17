"""Diagnose the cold neck phase after the original full-board gate has failed."""
import hashlib,json
from pathlib import Path
W=Path(__file__).resolve().parent
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
rows=json.loads((W/'candidate-validation-v4/commands.json').read_text())
assert next(r for r in rows if r['name']=='full-board')['exit_code']==101
t=(W/'observe_gate_v2.py').read_text().replace("O=W/'gate-observation-v2'","O=W/'gate-observation-v3'")
with (W/'observe_gate_v3.py').open('x') as f:f.write(t)
t=(W/'backtrace_gate_v1.py').read_text().replace("O=W/'gate-backtrace-v1'","O=W/'gate-backtrace-v2'")
t=t.replace('gate-observation-v2/result.json','gate-observation-v3/result.json')
with (W/'backtrace_gate_v2.py').open('x') as f:f.write(t)
with (W/'neck-observer-preparation-v1.json').open('x') as f:json.dump({'complete':True,'runner_sha256':sha(__file__),
    'observer_sha256':sha(W/'observe_gate_v3.py'),'backtrace_sha256':sha(W/'backtrace_gate_v2.py'),
    'full_board_completed_failure':True,'cold_neck_still_incomplete':True},f,indent=2)
