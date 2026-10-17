use hyperdrc::kicad::{BoardModel, load_kicad_pcb};
use hyperdrc::{PcbGeometryUncertainty, PcbRegion, PcbRegionExt, Scalar};
use serde_json::{Value, json};
use std::fmt::Debug;
use std::path::Path;

fn observation<T: Debug>(value: Result<T, PcbGeometryUncertainty>) -> Value {
    match value {
        Ok(value) => json!({"status":"ok","value":format!("{value:?}")}),
        Err(error) => json!({"status":"error","operation":error.operation,
            "source":error.source,"detail":error.detail}),
    }
}

fn inspect(board: &BoardModel) -> Value {
    let empty = PcbRegion::new(hypercurve::CurveRegion2::empty(), None);
    let features: Vec<_> = board.copper.iter().map(|feature| {
        let region = &feature.region;
        json!({"kind":format!("{:?}",feature.kind),"layer":feature.layer,"net":feature.net,
            "location_debug":format!("{:?}",feature.location),
            "empty":observation(region.try_is_empty()),
            "area":observation(region.try_area()),
            "bounds_present":observation(region.try_bounding_box().map(|b|b.is_some())),
            "classify_origin":observation(region.classify_xy(Scalar::zero(),Scalar::zero())),
            "native_handoff":observation(region.clone().try_into_native().map(|_|())),
            "report_components":observation(region.try_to_multipolygon().map(|v|v.0.len())),
            "union_with_empty":observation(region.try_union(&empty).map(|_|())),
            "empty_union":observation(empty.try_union(region).map(|_|())),
            "intersection_with_empty":observation(region.try_intersection(&empty).map(|_|())),
            "difference_empty_left":observation(empty.try_difference(region).map(|_|())),
            "offset_zero":observation(region.offset(Scalar::zero()).map(|_|()))})
    }).collect();
    json!({"copper_count":board.copper.len(),"features":features,
        "all_copper":observation(board.all_copper().map(|_|())),
        "copper_layers":observation(board.copper_layers(&[]).map(|layers|layers.len()))})
}

fn load(path: &Path, fields: &str) -> BoardModel {
    std::fs::write(path,format!("(kicad_pcb (net 1 signal) (segment (layer F.Cu) (net 1) {fields}))")).unwrap();
    load_kicad_pcb(path).unwrap()
}

fn deny_refinement() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620].into_iter().map(Scalar::from).collect(),
    ).unwrap();
    assert!(matches!(polynomial.isolate_unit_interval_roots(),
        Err(hypercurve::ExactCurveError::Blocked(blocker))
        if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted));
}

fn main() {
    let root = std::env::args().nth(1).expect("fresh fixture output directory");
    let root = Path::new(&root);
    std::fs::create_dir(root).unwrap();
    let cases: Vec<Value> = serde_json::from_str(include_str!("cases.json")).unwrap();
    let mut observations = Vec::new();
    for case in &cases {
        for (label, policy) in [
            ("strict",hyperlimit::PredicatePolicy::STRICT),
            ("approximate",hyperlimit::PredicatePolicy::APPROXIMATE_512),
            ("strict_after",hyperlimit::PredicatePolicy::STRICT),
        ] {
            let path=root.join(format!("{}-{label}.kicad_pcb",case["name"].as_str().unwrap()));
            let loaded = hypercurve::evaluate_under(policy,||load(&path,case["fields"].as_str().unwrap()));
            let receipt=format!("{:?}",loaded.certainty());
            let board=loaded.into_unverified();
            observations.push(json!({"name":case["name"],"fields":case["fields"],"expected":case["expected"],
                "ambient_policy":label,"outer_receipt":receipt,"observation":inspect(&board)}));
        }
    }
    let mut escaped=None;
    let denied=hypercurve::evaluate_with_budget(hyperlimit::PredicatePolicy::STRICT,
        hypercurve::WorkBudget::ZERO,||{
            deny_refinement();
            escaped=Some(load(&root.join("denied.kicad_pcb"),"(start 10 20) (end 12 20) (width 2)"));
        });
    assert!(denied.result.is_err());
    let denied=inspect(&escaped.unwrap());
    let recovery=inspect(&load(&root.join("recovery.kicad_pcb"),"(start 10 20) (end 12 20) (width 2)"));
    println!("{}",serde_json::to_string_pretty(&json!({"observations":observations,
        "exhausted_scope_escaped_board":denied,"recovery":recovery})).unwrap());
}
