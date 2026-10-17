"""Choose dependencies from retained Cargo JSON, not ambiguous directory globs."""
import hashlib,json
from pathlib import Path
W=Path(__file__).resolve().parent
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
t=(W/'cost_preflight_v1.py').read_text()
t=t.replace("O=W/'cost-preflight-v1'","O=W/'cost-preflight-v2'")
a="    matches=list(deps.glob('libserde_json-*.rlib'));assert len(matches)==1"
b="""    log = W.parents[2]/'target/properties/native-stroke-completion/consumer-validation-v1/native-cli.stdout' if variant=='baseline' else W/'candidate-validation-v3/native-build.stdout'
    artifacts=[json.loads(l) for l in log.read_text().splitlines() if l.startswith('{')]
    matches=[Path(p) for a in artifacts if a.get('reason')=='compiler-artifact' and a['target']['name']=='serde_json'
        for p in a['filenames'] if p.endswith('.rlib')]
    assert len(matches)==1"""
assert t.count(a)==1;t=t.replace(a,b)
with (W/'cost_preflight_v2.py').open('x') as f:f.write(t)
with (W/'cost-preflight-preparation-v2.json').open('x') as f:json.dump({'complete':True,'runner_sha256':sha(__file__),
    'v1_exit':1,'v1_error':'Baseline dependency directory contains multiple serde_json feature builds; no compilation or measurement ran.',
    'v1_runner_sha256':sha(W/'cost_preflight_v1.py'),'v2_runner_sha256':sha(W/'cost_preflight_v2.py')},f,indent=2)
