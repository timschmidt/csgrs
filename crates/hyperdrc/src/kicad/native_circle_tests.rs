use super::*;
use crate::scalar::scalar;

#[test]
fn authored_via_contains_points_between_its_circle_and_report_chords() {
    let board = load_kicad_pcb(Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/native-circle-via.kicad_pcb"
    )))
    .unwrap();
    let x = scalar("0.999");
    let y = scalar("0.03");
    assert!(&x * &x + &y * &y < Scalar::one());
    assert_eq!(board.copper.len(), 2);
    assert_eq!(board.copper[0].layer, "F.Cu");
    assert_eq!(board.copper[1].layer, "B.Cu");
    for feature in &board.copper {
        assert_eq!(
            feature.region.classify_xy(x.clone(), y.clone()),
            Ok(hypercurve::RegionPointLocation::Inside)
        );
    }
}

fn parse_via(source: &str) -> (Vec<CopperFeature>, Vec<DrillFeature>) {
    let root = sexp::parse(source).unwrap();
    let mut copper = vec![];
    let mut drills = vec![];
    parse_tracks_and_vias(&root, &HashMap::new(), &[], &mut copper, &mut drills)
        .expect("valid via source loads");
    (copper, drills)
}

#[test]
fn via_source_center_and_size_retain_all_decimal_digits() {
    let (copper, drills) = parse_via(
        "(kicad_pcb (via (at 9007199254740992.1 -0.00000000000000000000001)
         (size 0.20000000000000000000004) (drill 0.05) (layers F.Cu B.Cu)))",
    );
    assert_eq!(copper.len(), 2);
    assert_eq!(drills.len(), 1);
    let center = [
        scalar("9007199254740992.1"),
        scalar("-0.00000000000000000000001"),
    ];
    let radius = scalar("0.10000000000000000000002");
    let tiny = scalar("0.000000000000000000000000000001");
    assert_eq!(drills[0].location, center);
    for feature in &copper {
        assert_eq!(feature.location, center);
        assert!(feature.region.exact_construction_error().is_none());
        let bounds = feature.region.try_native_bounds().unwrap().unwrap();
        assert_eq!(&bounds[0], &(&center[0] - &radius));
        assert_eq!(&bounds[2], &(&center[0] + &radius));
        assert_eq!(&bounds[1], &(&center[1] - &radius));
        assert_eq!(&bounds[3], &(&center[1] + &radius));
        assert_eq!(
            feature
                .region
                .classify_xy(&center[0] + &radius - &tiny, center[1].clone()),
            Ok(hypercurve::RegionPointLocation::Inside)
        );
        assert_eq!(
            feature
                .region
                .classify_xy(&center[0] + &radius + &tiny, center[1].clone()),
            Ok(hypercurve::RegionPointLocation::Outside)
        );
    }
}

#[test]
fn invalid_via_size_is_retained_as_a_geometry_failure() {
    for size in ["-2", "0", "NaN"] {
        let (copper, _) = parse_via(&format!(
            "(kicad_pcb (via (at 0 0) (size {size}) (drill 0.4) (layers F.Cu B.Cu)))"
        ));
        assert_eq!(copper.len(), 2);
        for feature in &copper {
            assert!(feature.region.exact_construction_error().is_some());
            assert!(feature.region.offset(scalar("0.1")).is_err());
            let findings = crate::checks::layer_sanity(&feature.layer, &feature.region, None);
            assert!(
                findings
                    .iter()
                    .any(|finding| finding.severity == crate::report::Severity::Error)
            );
        }
    }
}

#[test]
fn invalid_pad_geometry_is_retained_for_diagnostics_instead_of_area_filtered() {
    let root=sexp::parse("(kicad_pcb (footprint fixture (at 0 0) (pad 1 smd rect (at 0 0) (size inf 1) (layers F.Cu))))").unwrap();
    let mut copper = vec![];
    let mut drills = vec![];
    parse_footprints(&root, &HashMap::new(), &[], &mut copper, &mut drills);
    assert_eq!(copper.len(), 1);
    assert!(copper[0].region.try_area().is_err());
    assert!(copper[0].region.try_is_empty().is_err());
}

#[test]
fn certified_empty_pad_geometry_is_still_excluded() {
    let root=sexp::parse("(kicad_pcb (footprint fixture (at 0 0) (pad 1 smd circle (at 0 0) (size 0 0) (layers F.Cu))))").unwrap();
    let mut copper = vec![];
    let mut drills = vec![];
    parse_footprints(&root, &HashMap::new(), &[], &mut copper, &mut drills);
    assert!(copper.is_empty());
}
