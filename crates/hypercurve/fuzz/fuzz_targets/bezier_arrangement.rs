#![no_main]

use hypercurve::{
    BezierArrangementGraph2, BezierParameter2, Classification, CurvePolicy, Point2,
    QuadraticBezier2, Real,
};
use libfuzzer_sys::fuzz_target;

fn real_from_byte(byte: u8) -> Real {
    Real::from(byte as i32 - 128)
}

fn unit_from_byte(byte: u8) -> Real {
    (Real::from((byte % 17) as i32) / Real::from(16_i32)).unwrap()
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(real_from_byte(x), real_from_byte(y))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }

    let policy = CurvePolicy::certified();
    let mut materializations = Vec::new();
    for chunk in data.chunks(8).take(8) {
        if chunk.len() < 8 {
            break;
        }
        let curve = QuadraticBezier2::new(
            point(chunk[0], chunk[1]),
            point(chunk[2], chunk[3]),
            point(chunk[4], chunk[5]),
        );
        let mut parameters = Vec::new();
        if let Ok(Classification::Decided(parameter)) =
            BezierParameter2::exact(unit_from_byte(chunk[6]), &policy)
        {
            parameters.push(parameter);
        }
        if let Ok(Classification::Decided(materialization)) =
            curve.split_at_parameters(&parameters, &policy)
        {
            materializations.push(materialization);
        }
    }

    let graph = BezierArrangementGraph2::from_split_materializations(&materializations);
    let _ = graph.traverse_branch_free(&policy);
});
