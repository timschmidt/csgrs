"""Preserve the failed v2 preparation and create a corrected, separate v3."""
import gzip, hashlib, json
from pathlib import Path
W=Path(__file__).resolve().parent
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
out=W/'aggregation-preparation-recovery-v1'; out.mkdir(exist_ok=False)
paths=[W/'prepare_candidate_v2.py',W/'candidate-preparation-v2']
files=[W/'prepare_candidate_v2.py']+[p for p in (W/'candidate-source-v2').rglob('*') if p.is_file()]
manifest={str(p):sha(p) for p in sorted(files)}
with gzip.open(out/'partial-v2-sha256.json.gz','wt') as f:json.dump(manifest,f,sort_keys=True)
text=(W/'prepare_candidate_v2.py').read_text()
text=text.replace('candidate-source-v2','candidate-source-v3').replace('candidate-preparation-v2','candidate-preparation-v3')
old="t=replace_once(t,'.extend(feature.region.try_to_multipolygon()?.0);','.push(&feature.region);')"
new="t=replace_once(t,'.or_default()\\n                    .extend(feature.region.try_to_multipolygon()?.0);','.or_default()\\n                    .push(&feature.region);')"
assert text.count(old)==1
text=text.replace(old,new)
with (W/'prepare_candidate_v3.py').open('x') as f:f.write(text)
(out/'result.json').write_text(json.dumps({'complete':True,'runner_sha256':sha(__file__),
    'v2_preparation_exit':1,'failure':'replace_once substring also matched the all_copper statement; assertion stopped before model.rs edit',
    'v2_source_inputs_retained':len(manifest),'v2_source_manifest_sha256':sha(out/'partial-v2-sha256.json.gz'),
    'v3_runner_sha256':sha(W/'prepare_candidate_v3.py'),'v2_source_executed':False,
    'main_sources_modified':False},indent=2)+'\n')
