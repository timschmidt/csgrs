"""Verify the formatted source without editing either preparation attempt."""
import gzip, hashlib, json, subprocess
from pathlib import Path
W=Path(__file__).resolve().parent
ROOT=W.parents[2]
S=W/'candidate-source-v3'; O=W/'candidate-preparation-v3'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):
    with Path(p).open('x') as f:json.dump(v,f,indent=2);f.write('\n')
assert not (O/'result.json').exists()
with gzip.open(W/'candidate-preparation-v1/source-hashes.json.gz','rt') as f:old=json.load(f)
added='crates/hyperdrc/tests/kicad_native_aggregation.rs'
inputs={p:sha(S/p) for p in sorted(set(old)|{added})}
changed={p:{'before':old.get(p),'after':h} for p,h in inputs.items() if old.get(p)!=h}
assert set(changed)=={'crates/hyperdrc/src/geometry.rs','crates/hyperdrc/src/geometry/region.rs',
    'crates/hyperdrc/src/kicad/model.rs','crates/hyperdrc/README.md',added}
test=(W/'probe-source-v2/kicad_native_aggregation.rs').read_text()
formatted=(S/added).read_text()
# The inspected diff contains only wrapping and two optional trailing commas.
def normalize(t):return ''.join(t.split()).replace(',)',')')
assert normalize(test)==normalize(formatted)
check=subprocess.run(['cargo','fmt','--all','--','--check'],cwd=S,capture_output=True,text=True)
with (O/'final-format.stdout').open('x') as f:f.write(check.stdout)
with (O/'final-format.stderr').open('x') as f:f.write(check.stderr)
assert check.returncode==0
assert all(sha(S/p)==h for p,h in inputs.items())
with gzip.open(O/'source-hashes.json.gz','xt') as f:json.dump(inputs,f,sort_keys=True)
dump(O/'result.json',{'complete':True,'source':str(S),'source_inputs':len(inputs),
    'source_hashes_sha256':sha(O/'source-hashes.json.gz'),'changed_from_v1':changed,
    'preparation_runner':str(W/'prepare_candidate_v3.py'),'preparation_runner_sha256':sha(W/'prepare_candidate_v3.py'),
    'preparation_exit':1,'preparation_assertion':'Byte-identical test guard rejected cargo fmt wrapping and optional trailing commas.',
    'finalizer_sha256':sha(__file__),'test_contract_unchanged':True,'format_exit':check.returncode,
    'all_source_bytes_unchanged_by_finalizer':True,'main_sources_modified':False,'native_runtime_admitted':False})
common=(W/'validation_common_v1.py').read_text()
for label in ['source','preparation','validation','target','scratch','executables']:
    common=common.replace('candidate-'+label+'-v1','candidate-'+label+'-v3')
with (W/'validation_common_v3.py').open('x') as f:f.write(common)
runner=(W/'candidate_v1.py').read_text().replace('from validation_common_v1','from validation_common_v3')
needle="if any(focus):"
assert runner.count(needle)==1
runner=runner.replace(needle,"focus.append(v.run('native-aggregation', ['cargo', 'test', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--test', 'kicad_native_aggregation', '--', '--nocapture', '--test-threads=1']))\n"+needle)
with (W/'candidate_v3.py').open('x') as f:f.write(runner)
print(json.dumps({'complete':True,'source_inputs':len(inputs),'changed_from_v1':list(changed),
    'test_changes':'formatting only; test assertions preserved'}))
