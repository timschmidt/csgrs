use hyperdrc::Scalar;
use serde_json::{Value, json};
fn rational(v: &Value) -> Scalar {
    let n: Scalar = v[0].as_str().unwrap().parse().unwrap();
    let d: Scalar = v[1].as_str().unwrap().parse().unwrap();
    (n / d).unwrap()
}
fn surd(v: &Value) -> Scalar {
    rational(&v[0]) + rational(&v[1]) * Scalar::from(2).sqrt().unwrap()
}
fn equal(a: &Scalar, b: &Scalar) -> bool {
    hyperlimit::compare_reals(a, b, hyperlimit::PredicatePolicy::STRICT).value()
        == Some(std::cmp::Ordering::Equal)
}
fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let source: Value = serde_json::from_slice(&std::fs::read(&args[0]).unwrap()).unwrap();
    let report: Value = serde_json::from_slice(&std::fs::read(&args[1]).unwrap()).unwrap();
    let board = hyperdrc::kicad::load_kicad_pcb(std::path::Path::new(
        source["source"].as_str().unwrap(),
    ))
    .unwrap();
    let expected = source["drills"].as_array().unwrap();
    assert_eq!(board.drills.len(), expected.len());
    for (index, (drill, expected)) in board.drills.iter().zip(expected).enumerate() {
        assert!(
            equal(&drill.location[0], &surd(&expected["center"][0])),
            "drill {index} x"
        );
        assert!(
            equal(&drill.location[1], &surd(&expected["center"][1])),
            "drill {index} y"
        );
        assert!(
            equal(&drill.diameter, &surd(&expected["diameter"])),
            "drill {index} diameter"
        );
        assert_eq!(drill.plated, expected["plated"].as_bool().unwrap());
    }
    let mut actual_pairs = report["violations"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|v| v["check"] == "drill-spacing")
        .map(|v| serde_json::to_string(&v["locations"]).unwrap())
        .collect::<Vec<_>>();
    let mut expected_pairs = source["pairs"]
        .as_array()
        .unwrap()
        .iter()
        .map(|pair| {
            let points = pair
                .as_array()
                .unwrap()
                .iter()
                .map(|index| {
                    let expected = &expected[index.as_u64().unwrap() as usize];
                    [
                        surd(&expected["center"][0]).to_f64_lossy().unwrap(),
                        surd(&expected["center"][1]).to_f64_lossy().unwrap(),
                    ]
                })
                .collect::<Vec<_>>();
            serde_json::to_string(&points).unwrap()
        })
        .collect::<Vec<_>>();
    actual_pairs.sort();
    expected_pairs.sort();
    assert_eq!(actual_pairs, expected_pairs);
    println!(
        "{}",
        json!({"source":source["source"],"source_sha256":source["input_sha256"],"exact_drills_matched":expected.len(),"exact_clearance_pairs":expected_pairs.len(),"complete_report_pair_multiset_matches":true})
    );
}
