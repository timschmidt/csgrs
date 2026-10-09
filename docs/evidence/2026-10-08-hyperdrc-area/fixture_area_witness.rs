use hyperdrc::{PcbRegionExt, Scalar};
use hyperlimit::{PredicatePolicy, compare_reals};
use std::cmp::Ordering;

fn compare(left: &Scalar, right: &Scalar) -> Ordering {
    compare_reals(left, right, PredicatePolicy::STRICT)
        .value()
        .unwrap()
}

fn main() {
    let path = std::env::args().nth(1).unwrap();
    let board = hyperdrc::kicad::load_kicad_pcb(std::path::Path::new(&path)).unwrap();
    let layers = board
        .copper_layers(&["In1.Cu".into(), "B.Cu".into()])
        .unwrap();
    let mut sums = Vec::new();
    let mut rows = Vec::new();
    for name in ["In1.Cu", "B.Cu"] {
        let region = &layers.iter().find(|(layer, _)| layer == name).unwrap().1;
        let native = region.try_area().unwrap();
        assert_eq!(compare(&native, &Scalar::from(10000)), Ordering::Equal);
        let report = region.try_to_multipolygon().unwrap();
        let areas = report
            .0
            .iter()
            .map(|p| p.try_area().unwrap())
            .collect::<Vec<_>>();
        // Replay the old required-area gate using the exact authored area sum.
        // Scalar reduction order does not change the exact rational value.
        let authored = Scalar::sum_owned(areas);
        rows.push(serde_json::json!({"layer":name, "native_area_equals_10000":true, "authored_profiles":report.0.len(), "authored_area_display":format!("{authored:#.12}")}));
        sums.push(authored);
    }
    let left = &sums[0];
    let right = &sums[1];
    let mut scale = if compare(&left.abs(), &right.abs()).is_gt() {
        left.abs()
    } else {
        right.abs()
    };
    if compare(&scale, &Scalar::one()).is_lt() {
        scale = Scalar::one();
    }
    let tolerance: Scalar = "0.000001".parse().unwrap();
    let old_gate = compare(&(left - right).abs(), &(&tolerance * scale)).is_le();
    assert!(
        !old_gate,
        "the old duplicate-layer area gate rejected this pair"
    );
    let left = &layers.iter().find(|(name, _)| name == "In1.Cu").unwrap().1;
    let right = &layers.iter().find(|(name, _)| name == "B.Cu").unwrap().1;
    let difference = left.try_xor(right).unwrap();
    assert!(difference.try_is_empty().unwrap());
    assert_eq!(
        compare(&difference.try_area().unwrap(), &Scalar::zero()),
        Ordering::Equal
    );
    let findings = hyperdrc::checks::duplicate_layer_geometry_readiness(
        &[("In1.Cu".to_owned(), left), ("B.Cu".to_owned(), right)],
        &Scalar::zero(),
    );
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "layer-sanity");
    println!("{}",serde_json::to_string_pretty(&serde_json::json!({"layers":rows,"old_authored_area_gate":old_gate,"native_symmetric_difference_empty":true,"native_symmetric_difference_area_zero":true,"duplicate_findings":findings.len(),"message":findings[0].message})).unwrap());
}
