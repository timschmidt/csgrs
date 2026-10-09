use hyperdrc::Scalar;
use hyperdrc::kicad::load_kicad_pcb;
use serde_json::{Value, json};

fn rational(value: &Value) -> Scalar {
    let a: Scalar = value[0].as_str().unwrap().parse().unwrap();
    let b: Scalar = value[1].as_str().unwrap().parse().unwrap();
    (a / b).unwrap()
}
fn equal(a: &Scalar, b: &Scalar) -> bool {
    hyperlimit::compare_reals(a, b, hyperlimit::PredicatePolicy::STRICT).value()
        == Some(std::cmp::Ordering::Equal)
}
fn main() {
    let input = std::env::args().nth(1).unwrap();
    let oracle: Value = serde_json::from_slice(&std::fs::read(input).unwrap()).unwrap();
    let board = load_kicad_pcb(std::path::Path::new(oracle["source"].as_str().unwrap())).unwrap();
    let tracks = board.copper.iter().filter(|feature| {
        feature.region.metadata().as_ref().is_some_and(|m| m.name == "KiCad segment")
    }).collect::<Vec<_>>();
    let expected = oracle["rows"].as_array().unwrap();
    let mut wrong = Vec::new();
    let mut matched = 0;
    for (index, (feature, row)) in tracks.iter().zip(expected).enumerate() {
        let anchor = equal(&feature.location[0], &rational(&row["center"][0]))
            && equal(&feature.location[1], &rational(&row["center"][1]));
        let metadata = feature.layer == row["layer"].as_str().unwrap()
            && feature.net.as_deref() == row["net"].as_str();
        let area = rational(&row["width"]) * rational(&row["length_squared"]).sqrt().unwrap()
            + rational(&row["area_pi"]) * Scalar::pi();
        let area_matches = feature.region.try_area().is_ok_and(|value| equal(&value, &area));
        let actual_bounds = feature.region.clone().try_into_native().ok()
            .and_then(|native| native.bounds().ok().flatten());
        let bounds_match = actual_bounds.as_ref().is_some_and(|bounds| {
            [bounds.min_x(), bounds.min_y(), bounds.max_x(), bounds.max_y()].iter()
                .zip(row["bounds"].as_array().unwrap()).all(|(actual, expected)| equal(actual, &rational(expected)))
        });
        if anchor && metadata && area_matches && bounds_match { matched += 1; }
        else { wrong.push(json!({"source_index":index,"anchor":anchor,"metadata":metadata,"area":area_matches,"bounds":bounds_match})); }
    }
    println!("{}", json!({"source":oracle["source"],"source_sha256":oracle["sha256"],"expected":expected.len(),"parsed":tracks.len(),"matched":matched,"wrong":wrong,"all_admitted":matched == expected.len() && tracks.len() == expected.len()}));
}
