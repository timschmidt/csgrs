use hyperdrc::Scalar;
use hyperdrc::kicad::{CopperKind, load_kicad_pcb};
use serde_json::{Value, json};
use std::collections::BTreeMap;

fn rational(value: &Value) -> Scalar {
    let numerator: Scalar = value[0].as_str().unwrap().parse().unwrap();
    let denominator: Scalar = value[1].as_str().unwrap().parse().unwrap();
    (numerator / denominator).unwrap()
}
fn surd(value: &Value) -> Scalar {
    rational(&value[0]) + rational(&value[1]) * Scalar::from(2).sqrt().unwrap()
}
fn area(value: &Value) -> Scalar {
    rational(&value[0]) + rational(&value[1]) * Scalar::pi()
}
fn equal(a: &Scalar, b: &Scalar) -> bool {
    hyperlimit::compare_reals(a, b, hyperlimit::PredicatePolicy::STRICT).value()
        == Some(std::cmp::Ordering::Equal)
}
struct Expected {
    anchor: [Scalar; 2],
    bounds: Option<[Scalar; 4]>,
    area: Option<Scalar>,
    source: Value,
}
fn main() {
    let expectations = std::env::args().nth(1).unwrap();
    let expectations: Value =
        serde_json::from_slice(&std::fs::read(expectations).unwrap()).unwrap();
    let board = load_kicad_pcb(std::path::Path::new(expectations["source"].as_str().unwrap()))
        .unwrap();
    let expected = expectations["rows"]
        .as_array()
        .unwrap()
        .iter()
        .map(|row| Expected {
            anchor: [surd(&row["anchor"][0]), surd(&row["anchor"][1])],
            bounds: row["bounds"]
                .as_array()
                .map(|v| [surd(&v[0]), surd(&v[1]), surd(&v[2]), surd(&v[3])]),
            area: row["area"].as_array().map(|_| area(&row["area"])),
            source: row.clone(),
        })
        .collect::<Vec<_>>();
    let mut by_net_layer = BTreeMap::<(String, Option<String>), Vec<usize>>::new();
    for (i, row) in expected.iter().enumerate() {
        by_net_layer
            .entry((
                row.source["layer"].as_str().unwrap().into(),
                row.source["net"].as_str().map(str::to_owned),
            ))
            .or_default()
            .push(i);
    }
    let mut seen = vec![false; expected.len()];
    let mut wrong_anchor = Vec::new();
    let mut wrong_geometry = Vec::new();
    let mut matched_native = 0;
    let mut matched_custom = 0;
    let mut pads = 0;
    for (i, feature) in board
        .copper
        .iter()
        .enumerate()
        .filter(|(_, f)| f.kind == CopperKind::Pad)
    {
        pads += 1;
        let candidates = by_net_layer
            .get(&(feature.layer.clone(), feature.net.clone()))
            .into_iter()
            .flatten()
            .copied()
            .filter(|&j| {
                equal(&feature.location[0], &expected[j].anchor[0])
                    && equal(&feature.location[1], &expected[j].anchor[1])
            })
            .collect::<Vec<_>>();
        if candidates.is_empty() {
            wrong_anchor.push(json!({"feature":i,"layer":feature.layer,"net":feature.net,"actual":feature.location.iter().map(ToString::to_string).collect::<Vec<_>>()}));
            continue;
        }
        let actual_area = feature.region.try_area();
        let actual_bounds = feature
            .region
            .clone()
            .try_into_native()
            .ok()
            .and_then(|r| r.bounds().ok().flatten())
            .map(|b| {
                [
                    b.min_x().clone(),
                    b.min_y().clone(),
                    b.max_x().clone(),
                    b.max_y().clone(),
                ]
            });
        let mut matched = false;
        for &j in &candidates {
            let row = &expected[j];
            if row.area.is_none() {
                seen[j] = true;
                matched_custom += 1;
                matched = true;
                break;
            }
            if seen[j] {
                continue;
            }
            if actual_area
                .as_ref()
                .is_ok_and(|a| equal(a, row.area.as_ref().unwrap()))
                && actual_bounds.as_ref().zip(row.bounds.as_ref()).is_some_and(
                    |(actual, expected)| actual.iter().zip(expected).all(|(a, b)| equal(a, b)),
                )
            {
                seen[j] = true;
                matched_native += 1;
                matched = true;
                break;
            }
        }
        if !matched {
            wrong_geometry.push(json!({"feature":i,"candidates":candidates,"area":actual_area.map(|a|a.to_string()).map_err(|e|e.to_string()),"bounds":actual_bounds.map(|b|b.map(|v|v.to_string()))}));
        }
    }
    let missing = seen
        .iter()
        .enumerate()
        .filter_map(|(i, seen)| (!seen).then_some(i))
        .collect::<Vec<_>>();
    println!(
        "{}",
        json!({"source":expectations["source"],"input_sha256":expectations["input_sha256"],"expected_rows":expected.len(),"parsed_pad_features":pads,"matched_native_components":matched_native,"custom_placement_only":matched_custom,"wrong_anchor":wrong_anchor,"wrong_geometry":wrong_geometry,"missing_expected_rows":missing,"all_admitted":wrong_anchor.is_empty()&&wrong_geometry.is_empty()&&missing.is_empty()})
    );
}
