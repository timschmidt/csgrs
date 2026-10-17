//! Exact disk oracle from independently parsed full-source rational tokens.
use hypercurve::RegionPointLocation;
use hyperdrc::Scalar;
use hyperdrc::kicad::{CopperKind, load_kicad_pcb};
use hyperlimit::{PredicatePolicy, compare_reals};
use serde_json::{Value, json};

fn main() {
    let path = std::env::args().nth(1).unwrap();
    let manifest: Value =
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut rows = Vec::new();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || {
            let board =
                load_kicad_pcb(std::path::Path::new(manifest["source"].as_str().unwrap()))
                    .unwrap();
            let segments: Vec<_> = board
                .copper
                .iter()
                .filter(|f| f.kind == CopperKind::Segment)
                .collect();
            assert_eq!(
                segments.len() as u64,
                manifest["positive_segments"].as_u64().unwrap()
            );
            for disk in manifest["disks"].as_array().unwrap() {
                let feature =
                    segments[disk["positive_segment_index"].as_u64().unwrap() as usize];
                let center = [0, 1].map(|axis| {
                    disk["center"][axis]
                        .as_str()
                        .unwrap()
                        .parse::<Scalar>()
                        .unwrap()
                });
                let radius = disk["radius"].as_str().unwrap().parse::<Scalar>().unwrap();
                let expected_area = Scalar::pi() * &radius * &radius;
                assert_eq!(feature.location, center);
                assert_eq!(
                    compare_reals(
                        &feature.region.try_area().unwrap(),
                        &expected_area,
                        PredicatePolicy::STRICT
                    )
                    .value(),
                    Some(std::cmp::Ordering::Equal)
                );
                assert_eq!(
                    feature
                        .region
                        .classify_xy(center[0].clone(), center[1].clone()),
                    Ok(RegionPointLocation::Inside)
                );
                assert_eq!(
                    feature
                        .region
                        .classify_xy(&center[0] + &radius, center[1].clone()),
                    Ok(RegionPointLocation::Boundary)
                );
                assert_eq!(
                    feature.region.classify_xy(
                        &center[0] + &radius * Scalar::from(2),
                        center[1].clone()
                    ),
                    Ok(RegionPointLocation::Outside)
                );
                let native = feature.region.clone().try_into_native().unwrap();
                let bounds = native.bounds().unwrap().unwrap();
                assert_eq!(bounds.min_x(), &(&center[0] - &radius));
                assert_eq!(bounds.max_x(), &(&center[0] + &radius));
                rows.push(json!({"source_segment_index":disk["source_segment_index"],"policy":format!("{policy:?}"),
                    "certified_disk_area_and_membership":true,"native_bounds_preserved":true}));
            }
        });
        assert!(result.certified().is_some());
    }
    println!("{}", serde_json::to_string_pretty(&rows).unwrap());
}
