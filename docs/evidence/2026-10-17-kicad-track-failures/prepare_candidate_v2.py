"""Preserve full-board completion with native coincident disks and typed source failures."""
import gzip,hashlib,json,re,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;S=W/'candidate-source-v2';O=W/'candidate-preparation-v2'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
prior=read(W/'candidate-validation-v1/result.json')
assert prior['complete'] and prior['unexpected_exits']==['native-tests','full-board','copper-neck','clippy']
assert 'unused `std::result::Result` that must be used' in (W/'candidate-validation-v1/clippy.stderr').read_text()
with gzip.open(W/'candidate-preparation-v1/source-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(W/'candidate-source-v1'/p)==h for p,h in inputs.items())
S.mkdir(exist_ok=False);O.mkdir(exist_ok=False)
for p,h in inputs.items():
    dest=S/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/'candidate-source-v1'/p,dest);assert sha(dest)==h
for fixture in ['CPArti FPGA dev board.zip','HVP109A.zip']:
    source=W/'candidate-source-v1/crates/hyperdrc/docs'/fixture;dest=S/'crates/hyperdrc/docs'/fixture
    shutil.copy2(source,dest);assert sha(dest)==sha(source)
tracks='crates/hyperdrc/src/kicad/tracks.rs';p=S/tracks;s=p.read_text()
s=s.replace('use crate::{LayerMetadata, PcbGeometryUncertainty, Scalar};', 'use crate::{LayerMetadata, PcbGeometryUncertainty, PcbRegion, Scalar};')
begin=s.index('    let start_finite = finite_point(&start)?;');end=s.index('    let empty = region.try_is_empty();',begin)
finite=s[begin:end]
old='''    let region = polygons_to_profile(
        vec![polygon],
        Some(LayerMetadata {
            name: "KiCad segment".to_string(),
        }),
    );
'''
assert finite.count(old)==1
finite=finite.replace(old,'    polygons_to_profile(vec![polygon], metadata)\n')
replacement='''    let metadata = Some(LayerMetadata { name: "KiCad segment".to_string() });
    // SourceScalar parses rationals. Equality here certifies source coincidence;
    // endpoints that merely round to the same binary64 pair are not a disk.
    let region = if start[0].value == end[0].value && start[1].value == end[1].value {
        let center = [start[0].value.clone(), start[1].value.clone()];
        let native = crate::geometry::circle_region(&center, &crate::scalar::half(&width.value));
        check_budget()?;
        PcbRegion::new(native.map_err(TrackLoadFailure::Geometry)?, metadata)
    } else {
'''+finite+'''    };
'''
s=s[:begin]+replacement+s[end:];p.write_text(s)
path='crates/hyperdrc/src/kicad/native_circle_tests.rs';p=S/path;s=p.read_text()
old='    parse_tracks_and_vias(&root, &HashMap::new(), &[], &mut copper, &mut drills);'
assert s.count(old)==1
p.write_text(s.replace(old,old[:-1]+'.expect("valid via source loads");'))
path2='crates/hyperdrc/src/kicad.rs';p=S/path2;s=p.read_text()
s=s.replace('fn degenerate_and_non_copper_geometry_is_ignored_without_fallback_shapes()',
    'fn coincident_track_keeps_its_disk_without_non_copper_fallback_shapes()')
begin=s.index('    fn coincident_track_keeps_its_disk_without_non_copper_fallback_shapes()')
end=s.index('\n    #[test]',begin)
part=s[begin:end]
old='''        let board = load_kicad_pcb(&path).unwrap();

        assert!(board.copper.is_empty());
        assert!(board.panel_features.is_none());'''
new='''        // Keep the complete original fixture. Its positive-width coincident
        // track is the disk centered at (1, 1) with radius 1/10.
        let board = load_kicad_pcb(&path).unwrap();
        assert_eq!(board.copper.len(), 1);
        let disk = &board.copper[0];
        assert_eq!(disk.kind, CopperKind::Segment);
        assert_eq!(disk.location, [Scalar::one(), Scalar::one()]);
        assert_eq!(crate::scalar::compare(&disk.region.try_area().unwrap(),
            &(Scalar::pi() / Scalar::from(100)).unwrap()), Some(std::cmp::Ordering::Equal));
        assert_eq!(disk.region.classify_xy(Scalar::one(), Scalar::one()),
            Ok(hypercurve::RegionPointLocation::Inside));
        assert_eq!(disk.region.classify_xy(Scalar::from(2), Scalar::one()),
            Ok(hypercurve::RegionPointLocation::Outside));
        assert!(board.panel_features.is_none());'''
assert part.count(old)==1;s=s[:begin]+part.replace(old,new)+s[end:]
s=s.replace('fn incomplete_objects_retain_failed_pad_geometry_without_panicking()',
    'fn incomplete_track_rejects_load_while_pad_failure_remains_guarded()')
begin=s.index('    fn incomplete_track_rejects_load_while_pad_failure_remains_guarded()')
part=s[begin:]
old='''        let board = load_kicad_pcb(&path).unwrap();

        assert_eq!(board.copper.len(), 1);
        assert_eq!(board.copper[0].kind, CopperKind::Pad);
        assert!(board.copper[0].region.try_area().is_err());
        let findings = crate::checks::layer::layer_sanity("F.Cu", &board.copper[0].region, None);'''
new='''        // Keep the complete original fixture and require the missing track
        // endpoint to stop loading before a partial board can be published.
        let error = load_kicad_pcb(&path).unwrap_err();
        let track = error.downcast_ref::<TrackLoadError>().unwrap();
        assert_eq!(track.segment_index, 0);
        assert_eq!(track.failure, TrackLoadFailure::MissingField { field: "end" });

        // Exercise the same malformed pad directly, retaining its independent
        // failure and downstream layer-sanity assertions from the original test.
        let root = sexp::parse(&fs::read_to_string(&path).unwrap()).unwrap();
        let mut copper = Vec::new();
        let mut drills = Vec::new();
        parse_footprints(&root, &HashMap::new(), &[], &mut copper, &mut drills);
        for via in root.named_children("via") {
            let via_source = Sexp::List(vec![Sexp::Atom("kicad_pcb".into()), via.clone()]);
            parse_tracks_and_vias(&via_source, &HashMap::new(), &[], &mut copper, &mut drills)
                .expect("unchanged via-only source contract");
        }
        parse_zones(&root, &HashMap::new(), &mut copper);
        assert_eq!(copper.len(), 1);
        assert_eq!(copper[0].kind, CopperKind::Pad);
        assert!(copper[0].region.try_area().is_err());
        let findings = crate::checks::layer::layer_sanity("F.Cu", &copper[0].region, None);'''
assert part.count(old)==1;part=part.replace(old,new)
assert part.count('assert!(board.drills.is_empty());')==1
part=part.replace('assert!(board.drills.is_empty());','assert!(drills.is_empty());')
s=s[:begin]+part;p.write_text(s)
original=(W/'candidate-source-v1'/path2).read_text()
assert re.findall(r'r#"(.*?)"#',original,re.S)==re.findall(r'r#"(.*?)"#',s,re.S)
assert original.split('#[cfg(test)]',1)[0]==s.split('#[cfg(test)]',1)[0]
old='''/// Straight-track bodies still use the legacy finite rectangular adapter:
/// success does not certify round caps or fidelity to the exact source stroke.'''
new='''/// Coincident exact endpoints produce a native disk with radius `width / 2`.
/// Other straight-track bodies still use the legacy finite rectangular adapter;
/// their success does not certify round caps or exact source-stroke fidelity.'''
assert s.count(old)==1;s=s.replace(old,new);p.write_text(s)
readme='crates/hyperdrc/README.md';p=S/readme;s=p.read_text()
old='''invalidates version-20 check evidence. Ordinary admitted tracks retain their
existing geometry and metadata. Their body still comes from the legacy finite
rectangular adapter: exact round caps, exact coordinate fidelity, and extreme
source ranges remain open work.'''
new='''invalidates version-20 check evidence. Coincident exact endpoints now produce
a native disk with radius width/2; these tracks were previously omitted, so
their geometry and resulting findings intentionally change. Previously
materialized finite track bodies retain their geometry and metadata. Those
bodies still come from the legacy finite rectangular adapter: exact round
caps, exact coordinate fidelity, and extreme source ranges remain open work.'''
assert s.count(old)==1;p.write_text(s.replace(old,new))
tests='crates/hyperdrc/tests/kicad_track_failures.rs';p=S/tests;s=p.read_text()
assert s.count('        "(start 1 2) (end 1 2) (width 2)",\n')==1
s=s.replace('        "(start 1 2) (end 1 2) (width 2)",\n','')
s+='''
#[test]
fn coincident_tracks_are_exact_disks_including_decimal_source_precision() {
    for (fields, center, radius) in [
        ("(start 1 2) (end 1 2) (width 2)", ["1", "2"], "1"),
        ("(start 3.0000 4.00) (end 3 4) (width 2)", ["3", "4"], "1"),
        ("(start 9007199254740992.1 -0.00000000000000000000001) (end 9007199254740992.1 -0.00000000000000000000001) (width 0.20000000000000000000004)",
            ["9007199254740992.1", "-0.00000000000000000000001"], "0.10000000000000000000002"),
    ] {
        let center = center.map(|value| value.parse::<Scalar>().unwrap());
        let radius = radius.parse::<Scalar>().unwrap();
        let expected_area = Scalar::pi() * &radius * &radius;
        for policy in [hyperlimit::PredicatePolicy::STRICT, hyperlimit::PredicatePolicy::APPROXIMATE_512, hyperlimit::PredicatePolicy::STRICT] {
            let outcome = hypercurve::evaluate_under(policy, || {
                let model = board(fields).unwrap();
                assert_eq!(model.copper.len(), 1);
                let feature = &model.copper[0];
                assert_eq!(feature.location, center);
                assert_eq!(feature.layer, "F.Cu");
                assert_eq!(feature.net.as_deref(), Some("signal"));
                let region = &feature.region;
                assert_eq!(hyperlimit::compare_reals(&region.try_area().unwrap(), &expected_area, hyperlimit::PredicatePolicy::STRICT).value(), Some(std::cmp::Ordering::Equal));
                assert_eq!(region.classify_xy(center[0].clone(), center[1].clone()), Ok(RegionPointLocation::Inside));
                assert_eq!(region.classify_xy(&center[0] + &radius, center[1].clone()), Ok(RegionPointLocation::Boundary));
                assert_eq!(region.classify_xy(&center[0] + &radius * Scalar::from(2), center[1].clone()), Ok(RegionPointLocation::Outside));
                let native = region.clone().try_into_native().unwrap();
                let bounds = native.bounds().unwrap().unwrap();
                assert_eq!(bounds.min_x(), &(&center[0] - &radius));
                assert_eq!(bounds.max_x(), &(&center[0] + &radius));
                assert_eq!(hyperlimit::compare_reals(&native.filled_area().unwrap().unwrap(), &expected_area, hyperlimit::PredicatePolicy::STRICT).value(), Some(std::cmp::Ordering::Equal));
            });
            assert!(outcome.certified().is_some());
        }
    }
}
'''
p.write_text(s)
subprocess.run(['cargo','fmt','--all'],cwd=S,check=True)
delta={p:sha(S/p) for p,h in inputs.items() if sha(S/p)!=h}
assert set(delta)=={path,path2,tracks,readme,tests}
inputs.update(delta)
changes=read(W/'candidate-preparation-v1/result.json')['changes']|delta
with gzip.open(O/'source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
dump(O/'result.json',{'complete':True,'base_commit':'ac14e99174318291a548749395552ab982a7d097',
    'source_inputs':len(inputs),'source_hashes_sha256':sha(O/'source-hashes.json.gz'),'changes':changes,
    'runner_sha256':sha(__file__),'delta_from_v1':delta,
    'correction':'Use the existing native circle constructor for exact coincident endpoints, retaining full workload completion. Handle Result in the via test and explicitly migrate two silent-omission expectations without changing their complete source fixtures.',
    'preserved_failed_attempt':'candidate-validation-v1','main_sources_modified':False})
print(json.dumps({'complete':True,'source_inputs':len(inputs),'changed_from_v1':[path,path2]}),flush=True)
