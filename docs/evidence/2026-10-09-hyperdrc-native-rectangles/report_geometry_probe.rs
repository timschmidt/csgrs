//! Untimed direct geometry replay for the three changed HVP109A findings.
use hyperdrc::{PcbRegionExt, Scalar};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind};
use serde_json::{Value, json};

fn feature(board: &BoardModel, marker: [f64; 2], kind: CopperKind, layer: &str) -> usize {
    let indices = board.copper.iter().enumerate().filter_map(|(i, f)| {
        (f.kind == kind && f.layer == layer && f.location.each_ref().map(|x| x.to_f64_lossy().unwrap()) == marker).then_some(i)
    }).collect::<Vec<_>>();
    assert_eq!(indices.len(), 1, "{marker:?} {kind:?} {layer}: {indices:?}");
    indices[0]
}

fn squared_distance(left: &CopperFeature, right: &CopperFeature) -> Scalar {
    let left = left.region.clone().try_into_native().unwrap();
    let right = right.region.clone().try_into_native().unwrap();
    let left = left.native_contours_fast_path().unwrap().unwrap();
    let right = right.native_contours_fast_path().unwrap().unwrap();
    let mut minimum: Option<Scalar> = None;
    for a in left.material_contours().iter().chain(left.hole_contours()).flat_map(|c| c.segments()) {
        for b in right.material_contours().iter().chain(right.hole_contours()).flat_map(|c| c.segments()) {
            let distance = a.distance_squared(b).unwrap();
            if minimum.as_ref().is_none_or(|min| {
                hyperlimit::compare_reals(&distance, min, hyperlimit::PredicatePolicy::STRICT).value().unwrap().is_lt()
            }) {
                minimum = Some(distance);
            }
        }
    }
    minimum.unwrap()
}

fn details(feature: &CopperFeature, index: usize) -> Value {
    let native = feature.region.clone().try_into_native().unwrap();
    let bounds = native.bounds().unwrap().unwrap();
    json!({
        "source_index": index, "kind": format!("{:?}", feature.kind), "layer": feature.layer,
        "authored_marker": feature.location.each_ref().map(|x| x.to_string()),
        "native_bounds": [bounds.min_x().to_string(), bounds.min_y().to_string(), bounds.max_x().to_string(), bounds.max_y().to_string()],
        "native_area": feature.region.try_area().unwrap().to_string(),
    })
}

fn main() {
    let path = std::env::args().nth(1).unwrap();
    let board = hyperdrc::kicad::load_kicad_pcb(std::path::Path::new(&path)).unwrap();
    let mut pairs = vec![];
    for (label, markers) in [
        ("previous-selected-mask-pair", [[171.5481, 127.2274], [171.6497, 127.1512]]),
        ("candidate-selected-mask-pair", [[160.03387, 91.20129], [160.70632, 92.39634]]),
    ] {
        let indices = markers.map(|marker| feature(&board, marker, CopperKind::Pad, "F.Cu"));
        let distance = squared_distance(&board.copper[indices[0]], &board.copper[indices[1]]);
        let zero_order = hyperlimit::compare_reals(&distance, &Scalar::zero(), hyperlimit::PredicatePolicy::STRICT).value().unwrap();
        pairs.push(json!({"label": label, "indices": indices,
            "features": indices.map(|i| details(&board.copper[i], i)),
            "squared_distance": distance.to_string(), "zero_order": format!("{zero_order:?}"),
        }));
    }
    let pad = feature(&board, [146.09444, 106.51035], CopperKind::Pad, "F.Cu");
    let mut overlaps = vec![];
    for layer in ["F.Cu", "B.Cu"] {
        let via = feature(&board, [146.65425, 107.12827], CopperKind::Via, layer);
        let overlap = board.copper[via].region.offset("0.15".parse().unwrap()).unwrap().try_intersection(&board.copper[pad].region).unwrap();
        let shapes = hyperdrc::geometry::multipolygon_to_shapes_scalar(&overlap.try_to_multipolygon().unwrap(), &Scalar::zero()).unwrap();
        overlaps.push(json!({"layer":layer,"pad":details(&board.copper[pad],pad),"via":details(&board.copper[via],via),"shapes":shapes}));
    }
    println!("{}", serde_json::to_string_pretty(&json!({
        "version": hyperdrc::readiness::CHECK_IMPLEMENTATION_VERSION,
        "mask_pairs": pairs, "via_overlaps": overlaps,
        "scope": "Identify retained authored markers, then exhaustively compare all native boundary-segment pairs under STRICT policy. Recompute the two via keepout overlaps from the selected source features. No report-corner metric or geometric tolerance decides the results."
    })).unwrap());
}
