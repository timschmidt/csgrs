"""Preserve v4 as a rejected, profiled, incomplete workload attempt."""
import hashlib,json
from pathlib import Path
W=Path(__file__).resolve().parent
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
t=(W/'stop_candidate_v3.py').read_text()
t=t.replace('candidate-v3-cancellation-v1','candidate-v4-cancellation-v1').replace('candidate_v3.py','candidate_v4.py')
t=t.replace('gate-observation-v1/result.json','gate-observation-v2/result.json').replace('candidate-validation-v3','candidate-validation-v4')
t=t.replace('Rejected candidate exceeded 120 seconds total worker CPU while still at first layer-sanity; source report decomposition is not retained.',
    'Revised candidate exceeded 120 seconds worker CPU before profiling; layer-sanity took 247.806 seconds and native copper-neck work remains expensive. Report-state tests pass. Full gate is cancelled incomplete.')
with (W/'stop_candidate_v4.py').open('x') as f:f.write(t)
with (W/'cancellation-preparation-v4.json').open('x') as f:json.dump({'complete':True,
    'runner_sha256':sha(__file__),'stopper_sha256':sha(W/'stop_candidate_v4.py'),
    'profiler_result_sha256':sha(W/'gate-profile-v1/result.json'),'backtrace_result_sha256':sha(W/'gate-backtrace-v1/result.json')},f,indent=2)
