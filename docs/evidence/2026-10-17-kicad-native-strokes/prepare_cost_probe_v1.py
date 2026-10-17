"""Reuse the admitted counting allocator with a public source/aggregation probe."""
import hashlib,json
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2]
OLD=ROOT/'docs/evidence/2026-10-17-native-stroke-completion'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
parent=json.loads((OLD/'checkpoint-v1.json').read_text())
p=OLD/'cost_probe_v2.rs'
assert sha(p)==parent['evidence_files_sha256'][str(p.relative_to(ROOT))]
header=p.read_text().split('use hypercurve::{',1)[0]
with (W/'source_cost_probe_v1.rs').open('x') as f:f.write(header+(W/'source_cost_body_v1.rs').read_text())
with (W/'cost-probe-preparation-v1.json').open('x') as f:json.dump({'complete':True,'runner_sha256':sha(__file__),
    'allocator_source':str(p),'allocator_source_sha256':sha(p),'body_sha256':sha(W/'source_cost_body_v1.rs'),
    'probe_sha256':sha(W/'source_cost_probe_v1.rs'),'measurements_executed':False},f,indent=2)
