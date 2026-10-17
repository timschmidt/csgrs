"""Correct the baseline dependency search path; do not rerun passed candidate probes."""
import hashlib,json
from pathlib import Path
W=Path(__file__).resolve().parent
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
v2=json.loads((W/'cost-preflight-v2/result.json').read_text())
assert len(v2['observations'])==9 and all(all(r['certified_completion']) for r in v2['observations'].values())
t=(W/'cost_preflight_v2.py').read_text().replace("O=W/'cost-preflight-v2'","O=W/'cost-preflight-v3'")
t=t.replace("libs['hyperdrc'].parent","libs['hypercurve'].parent")
a='for variant,libs in libraries.items():\n    flags='
assert t.count(a)==1
t=t.replace(a,"for variant,libs in [('baseline',libraries['baseline'])]:\n    flags=")
t=t.replace("'candidate_is_rejected_v3':True,","'candidate_is_rejected_v3':True,'candidate_preflight_reuse':'cost-preflight-v2/result.json',")
with (W/'cost_preflight_v3.py').open('x') as f:f.write(t)
with (W/'cost-preflight-preparation-v3.json').open('x') as f:json.dump({'complete':True,'runner_sha256':sha(__file__),
    'v2_baseline_compile_exit':1,'reason':'Public hyperdrc rlib is in release/ but transitive dependencies are in release/deps/.',
    'v2_runner_sha256':sha(W/'cost_preflight_v2.py'),'v3_runner_sha256':sha(W/'cost_preflight_v3.py'),
    'passed_candidate_observations_sha256':sha(W/'cost-preflight-v2/result.json'),'candidate_probes_repeated':False},f,indent=2)
