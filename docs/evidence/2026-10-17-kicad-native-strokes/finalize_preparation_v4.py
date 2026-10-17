"""Verify the new test with the crate's formatter, without editing prior inputs."""
import gzip,hashlib,json,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;S=W/'candidate-source-v4';O=W/'candidate-preparation-v4'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,v):
    with Path(p).open('x') as f:json.dump(v,f,indent=2);f.write('\n')
assert not (O/'result.json').exists()
with gzip.open(W/'candidate-preparation-v3/source-hashes.json.gz','rt') as f:old=json.load(f)
manifest={p:sha(S/p) for p in old}
changed={p:{'before':old[p],'after':h} for p,h in manifest.items() if old[p]!=h}
test='crates/hyperdrc/tests/kicad_native_aggregation.rs'
assert set(changed)=={'crates/hyperdrc/src/lib.rs','crates/hyperdrc/src/geometry/region.rs',
    'crates/hyperdrc/src/region/projection.rs','crates/hyperdrc/src/kicad.rs',
    'crates/hyperdrc/src/kicad/README.md','crates/hyperdrc/README.md',test}
probe=W/'probe-source-v4/kicad_native_aggregation.rs';probe_sha=sha(probe)
r=subprocess.run(['rustfmt','--edition','2024','--config-path',str(S/'crates/.rustfmt.toml'),
    '--emit','stdout',str(probe)],capture_output=True,text=True)
with (O/'verified-test-format.stdout').open('x') as f:f.write(r.stdout)
with (O/'verified-test-format.stderr').open('x') as f:f.write(r.stderr)
assert r.returncode==0 and sha(probe)==probe_sha
# rustfmt stdout emits a filename header for a supplied path.
formatted=r.stdout
header=str(probe)+':\n\n'
if formatted.startswith(header):formatted=formatted[len(header):]
assert formatted==(S/test).read_text()
r=subprocess.run(['cargo','fmt','--all','--','--check'],cwd=S,capture_output=True,text=True)
with (O/'final-format.stdout').open('x') as f:f.write(r.stdout)
with (O/'final-format.stderr').open('x') as f:f.write(r.stderr)
assert r.returncode==0 and all(sha(S/p)==h for p,h in manifest.items())
with gzip.open(O/'source-hashes.json.gz','xt') as f:json.dump(manifest,f,sort_keys=True)
dump(O/'result.json',{'complete':True,'source':str(S),'source_inputs':len(manifest),
    'source_hashes_sha256':sha(O/'source-hashes.json.gz'),'changed_from_v3':changed,
    'preparation_exit':1,'preparation_assertion':'Whitespace normalization did not admit optional trailing array commas introduced by Cargo fmt.',
    'preparation_runner_sha256':sha(W/'prepare_candidate_v4.py'),'finalizer_sha256':sha(__file__),
    'probe_sha256':probe_sha,'crate_rustfmt_output_matches_candidate_test':True,
    'format_exit_code':r.returncode,'all_source_bytes_unchanged_by_finalizer':True,
    'main_sources_modified':False,'native_runtime_admitted':False})
common=(W/'validation_common_v3.py').read_text()
for label in ['source','preparation','validation','target','scratch','executables']:
    common=common.replace('candidate-'+label+'-v3','candidate-'+label+'-v4')
assert common.count("self.target=WORK/'candidate-target-v4'")==1
common=common.replace("self.target=WORK/'candidate-target-v4'","self.target=Path('/tmp/csgrs-kicad-native-target-v4')")
assert common.count("assert 'ext4' in self.protocol['mount'] and 'fuse' not in self.protocol['mount']")==1
common=common.replace("assert 'ext4' in self.protocol['mount'] and 'fuse' not in self.protocol['mount']",
    "assert 'tmpfs' in self.protocol['mount'] and 'fuse' not in self.protocol['mount']")
with (W/'validation_common_v4.py').open('x') as f:f.write(common)
runner=(W/'candidate_v3.py').read_text().replace('from validation_common_v3','from validation_common_v4')
with (W/'candidate_v4.py').open('x') as f:f.write(runner)
print(json.dumps({'complete':True,'source_inputs':len(manifest),'target':'/tmp/csgrs-kicad-native-target-v4'}))
