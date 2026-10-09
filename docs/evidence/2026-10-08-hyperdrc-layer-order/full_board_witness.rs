use hyperdrc::Scalar;
use hyperlimit::{PredicatePolicy, compare_reals};

fn main() {
    let path = std::env::args().nth(1).unwrap();
    let board = hyperdrc::kicad::load_kicad_pcb(std::path::Path::new(&path)).unwrap();
    let layers = board
        .copper_layers(&["In1.Cu".into(), "B.Cu".into()])
        .unwrap();
    let inner = &layers.iter().find(|(name, _)| name == "In1.Cu").unwrap().1;
    let bottom = &layers.iter().find(|(name, _)| name == "B.Cu").unwrap().1;
    for layer in [inner, bottom] {
        assert_eq!(
            compare_reals(
                &layer.try_area().unwrap(),
                &Scalar::from(10000),
                PredicatePolicy::STRICT
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }
    let xor = inner.try_xor(bottom).unwrap();
    assert!(xor.try_is_empty().unwrap());
    let mut rows = Vec::new();
    for input in [
        vec![("In1.Cu".to_owned(), inner), ("B.Cu".to_owned(), bottom)],
        vec![("B.Cu".to_owned(), bottom), ("In1.Cu".to_owned(), inner)],
    ] {
        let findings =
            hyperdrc::checks::duplicate_layer_geometry_readiness(&input, &Scalar::zero());
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0].check, "layer-sanity");
        rows.push(serde_json::to_value(&findings[0]).unwrap());
    }
    for field in [
        "check",
        "severity",
        "message",
        "polygons",
        "total_area",
        "island_index",
    ] {
        assert_eq!(rows[0][field], rows[1][field], "{field}");
    }
    assert_eq!(rows[0], rows[1]);
    assert_eq!(rows[0]["layers"], serde_json::json!(["B.Cu", "In1.Cu"]));
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "native_areas_equal_10000":true,"native_symmetric_difference_empty":true,
            "input_order_results_identical":true,"input_order_results":rows
        }))
        .unwrap()
    );
}
