//! Replay the original full-board pair and its independent overlap bound.
use hyperdrc::{PcbRegionExt, Scalar};
use hyperdrc::geometry::polygons_to_profile;
use hyperdrc::checks::layer::duplicate_layer_island_readiness;
use serde_json::json;

fn compare(a: &Scalar, b: &Scalar) -> std::cmp::Ordering {
    hyperlimit::compare_reals(a, b, hyperlimit::PredicatePolicy::STRICT).value().unwrap()
}

fn main() {
    let path = std::env::args().nth(1).unwrap();
    let board = hyperdrc::kicad::load_kicad_pcb(std::path::Path::new(&path)).unwrap();
    let indices = [1723, 1724];
    let features = indices.map(|i| &board.copper[i]);
    for f in features {
        assert_eq!(f.layer, "F.Cu");
        assert_eq!(f.kind, hyperdrc::kicad::CopperKind::Pad);
    }
    let regions = features.map(|f| &f.region);
    let bounds = regions.map(|r| {
        let native = r.clone().try_into_native().unwrap();
        let b = native.bounds().unwrap().unwrap();
        [b.min_x().clone(), b.min_y().clone(), b.max_x().clone(), b.max_y().clone()]
    });
    let width = Scalar::try_from(0.3).unwrap();
    let height = Scalar::try_from(1.15).unwrap();
    let radius = Scalar::try_from(0.075).unwrap();
    let expected = &width * height - (Scalar::from(4) - Scalar::pi()) * &radius * radius;
    let areas = regions.map(|r| r.try_area().unwrap());
    for a in &areas { assert_eq!(compare(a, &expected), std::cmp::Ordering::Equal); }
    let mut upper = Scalar::one();
    for axis in 0..2 {
        let lo = [&bounds[0][axis], &bounds[1][axis]].into_iter().max_by(|a,b|compare(a,b)).unwrap();
        let hi = [&bounds[0][axis+2], &bounds[1][axis+2]].into_iter().min_by(|a,b|compare(a,b)).unwrap();
        assert!(compare(hi,lo).is_gt());
        upper *= hi-lo;
    }
    let tolerance: Scalar = "1e-9".parse().unwrap();
    let floor: Scalar = "1e-12".parse().unwrap();
    let margins = areas.each_ref().map(|area| {
        let scaled = area * &tolerance;
        let tol = if compare(&scaled,&floor).is_gt() { scaled } else { floor.clone() };
        let margin = area - &upper - tol;
        assert!(compare(&margin,&Scalar::zero()).is_gt());
        margin.to_string()
    });
    let overlap = regions[0].try_intersection(regions[1]).unwrap();
    let overlap_area = overlap.try_area().unwrap();
    assert!(compare(&overlap_area, &Scalar::zero()).is_gt());
    assert!(compare(&overlap_area, &upper).is_lt());
    let overlap_error: Option<String> = None;
    let native_available = overlap.try_into_native().unwrap().native_contours_fast_path().unwrap().is_some();
    let pair = polygons_to_profile(regions.into_iter().flat_map(|r|r.try_to_multipolygon().unwrap().0).collect(), None);
    let findings = duplicate_layer_island_readiness("F.Cu", &pair, &tolerance);
    println!("{}",serde_json::to_string_pretty(&json!({
        "indices":indices,
        "source_markers":features.map(|f|f.location.each_ref().map(|s|s.to_string())),
        "bounds":bounds.each_ref().map(|b|b.each_ref().map(|s|s.to_string())),
        "areas":areas.each_ref().map(|s|s.to_string()),
        "independent_rounded_area":expected.to_string(),
        "overlap_bound":upper.to_string(),
        "positive_margin_beyond_tolerance":margins,
        "overlap_native_view_available":native_available,
        "overlap_area_error":overlap_error,
        "overlap_area":overlap_area.to_string(),
        "duplicate_findings":findings,
    })).unwrap());
}
