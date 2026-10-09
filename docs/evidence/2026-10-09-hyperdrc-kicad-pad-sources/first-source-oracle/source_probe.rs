use hypercurve::RegionPointLocation;
use hyperdrc::Scalar;
use hyperdrc::geometry::{SourceScalar, SourceUnit};
use hyperdrc::kicad::load_kicad_pcb;
use serde_json::{Value, json};
use std::path::Path;
use std::time::Instant;

fn scalar(s: &str) -> Scalar {
    SourceScalar::parse(SourceUnit::KiCadMillimeter, s)
        .unwrap()
        .value
}
fn equal(a: &Scalar, b: &Scalar) -> bool {
    hyperlimit::compare_reals(a, b, hyperlimit::PredicatePolicy::STRICT).value()
        == Some(std::cmp::Ordering::Equal)
}
struct Case {
    name: &'static str,
    footprint: &'static str,
    shape: &'static str,
    details: &'static str,
    witness: fn(&hyperdrc::kicad::BoardModel) -> Value,
}
fn exact_edge(board: &hyperdrc::kicad::BoardModel) -> Value {
    let query = [
        scalar("9007199254740992.20000000000000000000002"),
        scalar("0"),
    ];
    json!({"feature_count":board.copper.len(),"exact_edge":board.copper.first().is_some_and(|f| f.region.classify_xy(query[0].clone(),query[1].clone())==Ok(RegionPointLocation::Boundary))})
}
fn source_error(board: &hyperdrc::kicad::BoardModel) -> Value {
    json!({"feature_count":board.copper.len(),"retained_failure":board.copper.first().is_some_and(|f| f.region.try_area().is_err())})
}
fn native_nonzero(board: &hyperdrc::kicad::BoardModel) -> Value {
    json!({"feature_count":board.copper.len(),"native_nonzero":board.copper.first().is_some_and(|f| f.region.try_is_empty()==Ok(false))})
}
fn native_eight(board: &hyperdrc::kicad::BoardModel) -> Value {
    json!({"feature_count":board.copper.len(),"area_eight":board.copper.first().is_some_and(|f| f.region.try_area().is_ok_and(|a|equal(&a,&8.into())))})
}
fn huge_center(board: &hyperdrc::kicad::BoardModel) -> Value {
    let mut result = native_eight(board);
    result["exact_center_inside"] = json!(board.copper.first().is_some_and(|feature| {
        feature.region.classify_xy(scalar("1e400"), Scalar::zero())
            == Ok(RegionPointLocation::Inside)
    }));
    result
}
fn position(board: &hyperdrc::kicad::BoardModel) -> Value {
    json!({"feature_count":board.copper.len(),"correct_position":board.copper.first().is_some_and(|f|equal(&f.location[0],&scalar("10.1"))&&equal(&f.location[1],&scalar("17.9")))})
}
fn orientation(board: &hyperdrc::kicad::BoardModel) -> Value {
    json!({"feature_count":board.copper.len(),"absolute_orientation":board.copper.first().is_some_and(|f|f.region.classify_xy(1.into(),2.into())==Ok(RegionPointLocation::Boundary))})
}
fn mixed(board: &hyperdrc::kicad::BoardModel) -> Value {
    let expected = Scalar::from(8)
        - Scalar::from(3) * (Scalar::one() - scalar("0.25") * Scalar::pi()) * scalar("0.25")
        - scalar("0.03125");
    json!({"feature_count":board.copper.len(),"mixed_area":board.copper.first().is_some_and(|f|f.region.try_area().is_ok_and(|a|equal(&a,&expected)))})
}
fn marker(board: &hyperdrc::kicad::BoardModel) -> Value {
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        hyperdrc::checks::copper_net_intent(board, &[])
    }));
    match result {
        Ok(findings) => {
            json!({"marker":findings.iter().map(|v|v.check.as_str()).collect::<Vec<_>>()})
        },
        Err(_) => json!({"marker":"panic"}),
    }
}
fn main() {
    std::panic::set_hook(Box::new(|_| {}));
    let directory = std::env::args().nth(1).expect("input directory");
    let directory = Path::new(&directory);
    std::fs::create_dir_all(directory).unwrap();
    let cases = [
        Case {
            name: "decimal_rectangle",
            footprint: "",
            shape: "rect",
            details: "(at 9007199254740992.1 0) (size 0.20000000000000000000004 0.4)",
            witness: exact_edge,
        },
        Case {
            name: "decimal_roundrect",
            footprint: "",
            shape: "roundrect",
            details: "(at 9007199254740992.1 0) (size 0.20000000000000000000004 0.4)",
            witness: exact_edge,
        },
        Case {
            name: "decimal_circle",
            footprint: "",
            shape: "circle",
            details: "(at 9007199254740992.1 0) (size 0.20000000000000000000004 0.20000000000000000000004)",
            witness: exact_edge,
        },
        Case {
            name: "decimal_trapezoid",
            footprint: "",
            shape: "trapezoid",
            details: "(at 9007199254740992.1 0) (size 0.20000000000000000000004 0.4)",
            witness: exact_edge,
        },
        Case {
            name: "subrange_rectangle",
            footprint: "",
            shape: "rect",
            details: "(size 1e-400 1e-400)",
            witness: native_nonzero,
        },
        Case {
            name: "subrange_circle",
            footprint: "",
            shape: "circle",
            details: "(size 1e-400 1e-400)",
            witness: native_nonzero,
        },
        Case {
            name: "subrange_roundrect",
            footprint: "",
            shape: "roundrect",
            details: "(size 1e-400 1e-400)",
            witness: native_nonzero,
        },
        Case {
            name: "subrange_trapezoid",
            footprint: "",
            shape: "trapezoid",
            details: "(size 1e-400 1e-400)",
            witness: native_nonzero,
        },
        Case {
            name: "subrange_oval",
            footprint: "",
            shape: "oval",
            details: "(size 1e-400 1e-400)",
            witness: native_nonzero,
        },
        Case {
            name: "huge_center",
            footprint: "(at 1e400 0)",
            shape: "rect",
            details: "(size 2 4)",
            witness: huge_center,
        },
        Case {
            name: "invalid_ratio",
            footprint: "",
            shape: "roundrect",
            details: "(size 2 4) (roundrect_rratio oops)",
            witness: source_error,
        },
        Case {
            name: "invalid_delta",
            footprint: "",
            shape: "trapezoid",
            details: "(size 2 4) (rect_delta oops 0)",
            witness: source_error,
        },
        Case {
            name: "footprint_position",
            footprint: "(at 10.1 20.2 90)",
            shape: "rect",
            details: "(at 2.3 0 0) (size 2 4)",
            witness: position,
        },
        Case {
            name: "absolute_angle",
            footprint: "(at 0 0 90)",
            shape: "rect",
            details: "(at 0 0 0) (size 2 4)",
            witness: orientation,
        },
        Case {
            name: "mixed_corners",
            footprint: "",
            shape: "roundrect",
            details: "(size 2 4) (roundrect_rratio 0.25) (chamfer_ratio 0.125) (chamfer top_left)",
            witness: mixed,
        },
        Case {
            name: "failed_source_marker",
            footprint: "(at oops 0)",
            shape: "rect",
            details: "(size 2 4)",
            witness: marker,
        },
        Case {
            name: "huge_marker",
            footprint: "(at 1e400 0)",
            shape: "rect",
            details: "(size 2 4)",
            witness: marker,
        },
        Case {
            name: "ordinary_control",
            footprint: "",
            shape: "rect",
            details: "(size 2 4)",
            witness: native_eight,
        },
    ];
    for case in cases {
        let path = directory.join(format!("{}.kicad_pcb", case.name));
        let text = format!(
            "(kicad_pcb (footprint fixture {} (pad 1 smd {} (layers F.Cu) {})))",
            case.footprint, case.shape, case.details
        );
        std::fs::write(&path, &text).unwrap();
        let initial = load_kicad_pcb(&path).unwrap();
        let witness = (case.witness)(&initial);
        let started = Instant::now();
        for _ in 0..50 {
            std::hint::black_box(load_kicad_pcb(&path).unwrap());
        }
        println!(
            "{}",
            json!({"name":case.name,"iterations":50,"elapsed_ns":started.elapsed().as_nanos(),"input":text,"witness":witness})
        );
    }
}
