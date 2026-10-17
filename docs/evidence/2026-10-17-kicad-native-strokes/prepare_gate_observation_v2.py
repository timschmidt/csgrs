"""Observe the revised candidate without altering either validation runner."""
import hashlib,json
from pathlib import Path
W=Path(__file__).resolve().parent
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
t=(W/'observe_gate_v1.py').read_text()
t=t.replace("PREFIX=str(W/'candidate-target-v3/release/deps/hyperdrc-')",
    "PREFIX='/tmp/csgrs-kicad-native-target-v4/release/deps/hyperdrc-'")
t=t.replace("O=W/'gate-observation-v1'","O=W/'gate-observation-v2'")
with (W/'observe_gate_v2.py').open('x') as f:f.write(t)
with (W/'gate-observer-preparation-v2.json').open('x') as f:json.dump({'complete':True,
    'runner_sha256':sha(__file__),'observer_sha256':sha(W/'observe_gate_v2.py')},f,indent=2)
