"""Preserve v1; remove the required report projection at native aggregation."""
import gzip, hashlib, json, shutil, subprocess
from pathlib import Path
W = Path(__file__).resolve().parent
ROOT = W.parents[2]
SOURCE = W / 'candidate-source-v2'
O = W / 'candidate-preparation-v2'
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def read(p): return json.loads(Path(p).read_text())
def dump(p,v): Path(p).write_text(json.dumps(v,indent=2)+'\n')
def replace_once(t,a,b):
    assert t.count(a)==1, a
    return t.replace(a,b,1)
assert read(W/'aggregation-reproduction-v1/result.json')['counts']==[['6','2','0']]
assert read(W/'candidate-validation-v1/result.json')['unexpected_exits']==['source-failures']
with gzip.open(W/'candidate-preparation-v1/source-hashes.json.gz','rt') as f: inputs=json.load(f)
assert all(sha(W/'candidate-source-v1'/p)==h for p,h in inputs.items())
pins={str(p):sha(p) for p in [Path(__file__),W/'contract-v1.md',W/'aggregation-contract-v1.md',
    W/'aggregation-reproduction-v1/result.json',W/'candidate-preparation-v1/source-hashes.json.gz',
    W/'probe-source-v2/kicad_native_aggregation.rs']}
SOURCE.mkdir(exist_ok=False); O.mkdir(exist_ok=False)
for p,h in inputs.items():
    dest=SOURCE/p; dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(W/'candidate-source-v1'/p,dest); assert sha(dest)==h
name='crates/hyperdrc/src/geometry/region.rs'
t=(SOURCE/name).read_text()
t=replace_once(t,'use crate::{LayerMetadata, PcbRegion};','use crate::{LayerMetadata, PcbGeometryUncertainty, PcbRegion};')
a=t.index('fn aggregate_exact_regions(')
b=t.index('    let mut material = Vec::new();',a)
t=t[:a]+'''/// Combine checked PCB owners without requiring a finite report projection.
/// All selected sources are validated before empty or singleton identities.
pub(crate) fn regions_to_profile(
    regions: &[&PcbRegion],
    metadata: Option<LayerMetadata>,
) -> Result<PcbRegion, PcbGeometryUncertainty> {
    let source = metadata.as_ref().map(|m| m.name.as_str());
    let native = crate::operations::evaluate("profile-aggregation", source, || {
        let regions = regions.iter().map(|region| {
            region.ensure_exact_geometry("profile-aggregation")?;
            Ok(region.region.clone())
        }).collect::<Result<Vec<_>, PcbGeometryUncertainty>>()?;
        aggregate_native_regions(&regions)
            .map_err(|detail| crate::operations::failure("profile-aggregation", source, detail))
    })?;
    Ok(PcbRegion::new_shared(native, metadata))
}

fn aggregate_exact_regions(
    polygons: &[Polygon<f64>],
) -> Result<std::sync::Arc<CurveRegion2>, String> {
    let regions = polygons.iter().map(|polygon| {
        polygon.try_exact_region().map_err(|error| error.to_string())?;
        Ok(polygon.shared_exact_region())
    }).collect::<Result<Vec<_>, String>>()?;
    aggregate_native_regions(&regions)
}

fn aggregate_native_regions(
    regions: &[std::sync::Arc<CurveRegion2>],
) -> Result<std::sync::Arc<CurveRegion2>, String> {
    match regions {
        [] => return Ok(std::sync::Arc::new(CurveRegion2::empty())),
        [region] => return Ok(region.clone()),
        _ => {}
    }
'''+t[b:]
t=replace_once(t,'''    for polygon in polygons {
        let Some(native) = polygon
            .try_exact_region()
            .map_err(|error| error.to_string())?
            .native_contours_fast_path()''','''    for region in regions {
        let Some(native) = region.native_contours_fast_path()''')
t=replace_once(t,'''            let regions = polygons
                .iter()
                .map(Polygon::shared_exact_region)
                .collect::<Vec<_>>();
            return union_native_regions(&regions).map_err(|error| error.to_string());''','''            return union_native_regions(regions).map_err(|error| error.to_string());''')
(SOURCE/name).write_text(t)
name='crates/hyperdrc/src/geometry.rs'
t=(SOURCE/name).read_text()
t=replace_once(t,'pub use region::{empty_profile, polygon_to_profile, polygons_to_profile};',
    'pub use region::{empty_profile, polygon_to_profile, polygons_to_profile};\npub(crate) use region::regions_to_profile;')
(SOURCE/name).write_text(t)
name='crates/hyperdrc/src/kicad/model.rs'
t=(SOURCE/name).read_text()
t=replace_once(t,'use crate::geometry::{Polygon, empty_profile, polygons_to_profile};','use crate::geometry::regions_to_profile;')
t=replace_once(t,'use crate::{LayerMetadata, PcbGeometryUncertainty, PcbRegion, PcbRegionExt, Scalar};',
    'use crate::{LayerMetadata, PcbGeometryUncertainty, PcbRegion, Scalar};')
t=replace_once(t,'/// Build per-layer profiles, returning required geometry or projection failures.',
    '/// Build exact per-layer profiles, retaining geometry failures.\n    /// Finite report projection is a separate, fallible operation.')
t=replace_once(t,'BTreeMap<String, Vec<Polygon<f64>>>','BTreeMap<String, Vec<&PcbRegion>>')
t=replace_once(t,'.extend(feature.region.try_to_multipolygon()?.0);','.push(&feature.region);')
t=replace_once(t,'.map(|(layer, polygons)| {\n                    let region = polygons_to_profile(\n                        polygons,',
    '.map(|(layer, regions)| {\n                    let region = regions_to_profile(\n                        &regions,')
t=replace_once(t,'''                    );
                    region.ensure_exact_geometry("copper-layer")?;''','''                    )?;''')
a=t.index('    /// Combine all copper')
t=t[:a]+'''    /// Combine all copper natively, retaining failures without a report projection.
    pub fn all_copper(&self) -> Result<PcbRegion, PcbGeometryUncertainty> {
        crate::operations::evaluate("all-copper", Some(&self.source), || {
            let regions = self.copper.iter().map(|feature| &feature.region).collect::<Vec<_>>();
            regions_to_profile(&regions, Some(LayerMetadata {
                name: "KiCad copper".to_string(),
            }))
        })
    }
}
'''
(SOURCE/name).write_text(t)
name='crates/hyperdrc/README.md'
t=(SOURCE/name).read_text()
t=replace_once(t,'checked area, bounds, point queries and native handoff without a report projection.',
    '''checked area, bounds, point queries and native handoff without a report projection.
`BoardModel::all_copper` and `copper_layers` aggregate the native owners through
the shared profile implementation. Selected invalid inputs still fail before
empty or singleton shortcuts. A finite report projection remains explicitly
fallible and is not required to compose exact geometry.''')
(SOURCE/name).write_text(t)
added='crates/hyperdrc/tests/kicad_native_aggregation.rs'
shutil.copy2(W/'probe-source-v2/kicad_native_aggregation.rs',SOURCE/added)
r=subprocess.run(['cargo','fmt','--all'],cwd=SOURCE,capture_output=True,text=True)
(O/'format.stdout').write_text(r.stdout); (O/'format.stderr').write_text(r.stderr)
assert r.returncode==0
manifest={p:sha(SOURCE/p) for p in sorted(set(inputs)|{added})}
assert sha(SOURCE/added)==sha(W/'probe-source-v2/kicad_native_aggregation.rs')
changed={p:{'before':inputs.get(p),'after':h} for p,h in manifest.items() if inputs.get(p)!=h}
assert set(changed)=={'crates/hyperdrc/src/geometry.rs','crates/hyperdrc/src/geometry/region.rs',
    'crates/hyperdrc/src/kicad/model.rs','crates/hyperdrc/README.md',added}
with gzip.open(O/'source-hashes.json.gz','wt') as f: json.dump(manifest,f,sort_keys=True)
assert all(sha(W/'candidate-source-v1'/p)==h for p,h in inputs.items())
assert all(sha(p)==h for p,h in pins.items())
dump(O/'result.json',{'complete':True,'source':str(SOURCE),'source_inputs':len(manifest),
    'source_hashes_sha256':sha(O/'source-hashes.json.gz'),'input_sha256':pins,'changed_from_v1':changed,
    'main_sources_modified':False,'all_v1_sources_retained_unchanged':True,
    'shared_native_aggregation':True,'native_runtime_admitted':False})
print(json.dumps({'complete':True,'source_inputs':len(manifest),'changed_from_v1':list(changed)}))
