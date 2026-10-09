use geometry::{
    BooleanMode, Polyline, Shape, curve_boolean_clip_contour,
    curve_boolean_lower_clip_contour, curve_showcase_contour, curve_showcase_polylines,
};
use hypercurve::BooleanOp;
use serde_json::json;
use std::{fs, path::PathBuf, time::Instant};

fn main() {
    let arguments: Vec<_> = std::env::args_os().collect();
    let fixture = fs::read(&arguments[1]).unwrap();
    let output = PathBuf::from(&arguments[2]);
    fs::create_dir_all(&output).unwrap();
    let shared: Vec<Polyline> = serde_json::from_slice(&fixture).unwrap();
    assert_eq!(shared.len(), 2);
    let scenes = [
        (
            "default",
            Shape::from_materials(vec![curve_showcase_contour(0.0, 0.0, 6.2)]),
            Shape::from_materials(vec![curve_boolean_lower_clip_contour(0.0, 0.0, 6.2)]),
        ),
        (
            "multi",
            Shape::from_polylines(curve_showcase_polylines(0.0, 0.0, 6.2)),
            Shape::from_materials(vec![curve_boolean_clip_contour(0.0, 0.0, 6.2)]),
        ),
        (
            "shared",
            Shape::from_materials(vec![shared[0].clone()]),
            Shape::from_materials(vec![shared[1].clone()]),
        ),
    ];
    let mut rows = Vec::new();
    for (scene, first, second) in scenes {
        for (name, mode, native_mode) in [
            ("union", BooleanMode::Union, BooleanOp::Union),
            ("intersection", BooleanMode::Intersection, BooleanOp::Intersection),
            ("difference", BooleanMode::Difference, BooleanOp::Difference),
            ("xor", BooleanMode::Xor, BooleanOp::Xor),
        ] {
            let start = Instant::now();
            let result = first.boolean(&second, mode);
            let elapsed_ns = start.elapsed().as_nanos();
            let id = format!("{scene}-{name}");
            let mut row = json!({"name": id, "elapsed_ns": elapsed_ns});
            match result {
                Ok(Some(shape)) => {
                    let preview = serde_json::to_vec(&shape).unwrap();
                    fs::write(output.join(format!("{id}.json")), &preview).unwrap();
                    // Independent invocation outside the timer: a successful preview
                    // must retain the whole exact result for another operation.
                    let expected = first.to_curve_region().unwrap()
                        .boolean_region(&second.to_curve_region().unwrap(), native_mode)
                        .unwrap();
                    row["status"] = json!("decided");
                    row["materials"] = json!(shape.materials.len());
                    row["holes"] = json!(shape.holes.len());
                    row["preview_bytes"] = json!(preview.len());
                    row["retains_exact_region"] = json!(shape.to_curve_region()
                        .is_ok_and(|region| region == expected));
                    row["segmented_retains_exact_region"] = json!(shape.segmented_for_display()
                        .to_curve_region().is_ok_and(|region| region == expected));
                }
                Ok(None) => row["status"] = json!("uncertain"),
                Err(error) => {
                    row["status"] = json!("blocked");
                    row["error"] = json!(error);
                }
            }
            rows.push(row);
        }
    }
    println!("{}", serde_json::to_string(&rows).unwrap());
}
