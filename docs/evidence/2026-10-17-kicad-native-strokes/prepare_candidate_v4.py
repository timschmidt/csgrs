"""Retain native report sources and diagnostic history; preserve rejected v3."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2]
S=W/'candidate-source-v4';O=W/'candidate-preparation-v4'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
def replace(t,a,b):
    assert t.count(a)==1,a
    return t.replace(a,b,1)
assert read(W/'aggregation-reproduction-v3/result.json')['counts']==[['10','2','0']]
assert read(W/'candidate-v3-cancellation-v1/result.json')['cargo_target_closed']
assert read(W/'candidate-preparation-v3/result.json')['complete']
with gzip.open(W/'candidate-preparation-v3/source-hashes.json.gz','rt') as f:old=json.load(f)
assert all(sha(W/'candidate-source-v3'/p)==h for p,h in old.items())
pins={str(p):sha(p) for p in [Path(__file__),W/'contract-v1.md',W/'aggregation-contract-v1.md',
    W/'aggregation-provenance-contract-v1.md',W/'report-source-contract-v1.md',
    W/'aggregation-reproduction-v3/result.json',W/'candidate-v3-cancellation-v1/result.json',
    W/'candidate-preparation-v3/source-hashes.json.gz',W/'probe-source-v4/kicad_native_aggregation.rs']}
S.mkdir(exist_ok=False);O.mkdir(exist_ok=False)
for p,h in old.items():
    dest=S/p;dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(W/'candidate-source-v3'/p,dest);assert sha(dest)==h
n='crates/hyperdrc/src/lib.rs';t=(S/n).read_text()
t=replace(t,'    finite_projection: OnceLock<MultiPolygon<f64>>,',
    '''    finite_projection: OnceLock<MultiPolygon<f64>>,
    // Authored report decomposition, never proof of disjoint native components.
    finite_projection_sources: Option<Arc<[PcbRegion]>>,''')
assert t.count('            finite_projection: OnceLock::new(),')==2
t=t.replace('            finite_projection: OnceLock::new(),',
    '            finite_projection: OnceLock::new(),\n            finite_projection_sources: None,')
t=replace(t,'            finite_projection: projection,',
    '            finite_projection: projection,\n            finite_projection_sources: None,')
t=replace(t,'    pub(crate) fn new_with_exact_bounds_and_projection(','''    /// Attach already-validated source owners to their native aggregate.
    /// Report views retain authoring order without becoming topology evidence.
    pub(crate) fn new_aggregate(
        region: Arc<CurveRegion2>,
        metadata: Option<LayerMetadata>,
        sources: &[&Self],
    ) -> Self {
        if let [source] = sources {
            let mut result = Self::clone(source);
            result.metadata = metadata;
            return result;
        }
        let mut result = Self::new_shared(region, metadata);
        result.had_non_finite_input = sources.iter().any(|source| source.had_non_finite_input);
        if !sources.is_empty() {
            result.finite_projection_sources = Some(
                sources.iter().map(|source| Self::clone(source)).collect::<Vec<_>>().into(),
            );
        }
        result
    }

    pub(crate) fn new_with_exact_bounds_and_projection(''')
(S/n).write_text(t)
n='crates/hyperdrc/src/geometry/region.rs';t=(S/n).read_text()
t=replace(t,'    Ok(PcbRegion::new_shared(native, metadata))',
    '    Ok(PcbRegion::new_aggregate(native, metadata, regions))')
(S/n).write_text(t)
n='crates/hyperdrc/src/region/projection.rs';t=(S/n).read_text()
t=replace(t,'''                let components = self.exact_component_regions()?;
                let projected = finite_polygons(components, source)?;''','''                let projected = if let Some(sources) = &self.finite_projection_sources {
                    let mut polygons = Vec::new();
                    for source in sources.iter() {
                        polygons.extend(source.try_to_multipolygon()?.0);
                    }
                    MultiPolygon(polygons)
                } else {
                    let components = self.exact_component_regions()?;
                    finite_polygons(components, source)?
                };''')
t=replace(t,'/// count; they are not a claim that those authored profiles are disjoint.',
    '''/// count, including deferred views of aggregated source owners; they are not a
/// claim that those authored profiles are disjoint. Failed projections do not
/// populate a successful aggregate view and can be retried in a fresh scope.''')
(S/n).write_text(t)
n='crates/hyperdrc/src/kicad.rs';t=(S/n).read_text()
t=replace(t,'''/// Other straight-track bodies still use the legacy finite rectangular adapter;
/// their success does not certify round caps or exact source-stroke fidelity.''','''/// Distinct exact endpoints produce a native round-capped stroke, preserving
/// the source width and coordinates before normalization. Native queries and
/// board aggregation do not require a finite report projection; explicitly
/// requested report conversion can still fail outside its representable range.''')
(S/n).write_text(t)
n='crates/hyperdrc/src/kicad/README.md';t=(S/n).read_text()
t=replace(t,'- Tracks, vias, and zones.',
    '''- Straight tracks retain exact decimal/scientific endpoints and widths as
  native round-capped strokes (or disks for coincident endpoints). Required
  fields, arity, duplicates, numeric values and nonnegative width are checked
  before zero-width omission. Failed tracks return typed load errors instead
  of a complete board with missing copper. Vias and zones retain their existing
  contracts.''')
t=replace(t,'''Custom-pad local primitives, tracks, and graphics still use finite coordinates
or sampled strokes.''','''Custom-pad local primitives and graphics still use finite coordinates or sampled
strokes. Exact straight tracks and native board aggregation do not require
finite report coordinates; report and marker projection remain fallible.''')
(S/n).write_text(t)
n='crates/hyperdrc/README.md';t=(S/n).read_text()
t=replace(t,'''fallible and is not required to compose exact geometry.''','''fallible and is not required to compose exact geometry. Aggregated reports
retain authored source order and sample counts when requested, separately from
exact material-component topology. Failed projections cannot cache a successful
prefix. Source diagnostic history survives aggregation. Retained source owners
follow the aggregate's lifetime; no new retained-heap or total-work bound is
claimed.''')
(S/n).write_text(t)
test='crates/hyperdrc/tests/kicad_native_aggregation.rs'
shutil.copy2(W/'probe-source-v4/kicad_native_aggregation.rs',S/test)
r=subprocess.run(['cargo','fmt','--all'],cwd=S,capture_output=True,text=True)
(O/'format.stdout').write_text(r.stdout);(O/'format.stderr').write_text(r.stderr);assert r.returncode==0
normalize=lambda t:''.join(t.split()).replace(',)',')')
assert normalize((S/test).read_text())==normalize((W/'probe-source-v4/kicad_native_aggregation.rs').read_text())
manifest={p:sha(S/p) for p in old}
changed={p:{'before':old[p],'after':h} for p,h in manifest.items() if old[p]!=h}
assert set(changed)=={'crates/hyperdrc/src/lib.rs','crates/hyperdrc/src/geometry/region.rs',
    'crates/hyperdrc/src/region/projection.rs','crates/hyperdrc/src/kicad.rs',
    'crates/hyperdrc/src/kicad/README.md','crates/hyperdrc/README.md',test}
with gzip.open(O/'source-hashes.json.gz','wt') as f:json.dump(manifest,f,sort_keys=True)
assert all(sha(W/'candidate-source-v3'/p)==h for p,h in old.items())
assert all(sha(p)==h for p,h in pins.items())
dump(O/'result.json',{'complete':True,'source':str(S),'source_inputs':len(manifest),
    'source_hashes_sha256':sha(O/'source-hashes.json.gz'),'input_sha256':pins,
    'changed_from_v3':changed,'earlier_test_assertions_preserved':True,
    'main_sources_modified':False,'native_runtime_admitted':False})
common=(W/'validation_common_v3.py').read_text()
for label in ['source','preparation','validation','target','scratch','executables']:
    common=common.replace('candidate-'+label+'-v3','candidate-'+label+'-v4')
common=replace(common,"self.target=WORK/'candidate-target-v4'","self.target=Path('/tmp/csgrs-kicad-native-target-v4')")
common=replace(common,"assert 'ext4' in self.protocol['mount'] and 'fuse' not in self.protocol['mount']",
    "assert 'tmpfs' in self.protocol['mount'] and 'fuse' not in self.protocol['mount']")
with (W/'validation_common_v4.py').open('x') as f:f.write(common)
runner=(W/'candidate_v3.py').read_text().replace('from validation_common_v3','from validation_common_v4')
with (W/'candidate_v4.py').open('x') as f:f.write(runner)
print(json.dumps({'complete':True,'source_inputs':len(manifest),'changed_from_v3':list(changed),
    'target':'/tmp/csgrs-kicad-native-target-v4','earlier_targets_closed_and_unchanged':True}))
