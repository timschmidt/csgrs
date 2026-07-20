#![no_main]

use hypercurve::{BooleanOp, BulgeVertex2, Contour2, CurvePolicy, FillRule, Point2, Real, Region2};
use libfuzzer_sys::fuzz_target;

fn r(value: i32) -> Real {
    value.into()
}

fn rectangle(x: u8, y: u8, width: u8, height: u8) -> Region2 {
    let min_x = r(x as i32 - 128);
    let min_y = r(y as i32 - 128);
    let max_x = &min_x + r((width % 32) as i32 + 1);
    let max_y = &min_y + r((height % 32) as i32 + 1);
    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(Point2::new(min_x.clone(), min_y.clone()), Real::zero()),
        BulgeVertex2::new(Point2::new(max_x.clone(), min_y), Real::zero()),
        BulgeVertex2::new(Point2::new(max_x, max_y.clone()), Real::zero()),
        BulgeVertex2::new(Point2::new(min_x, max_y), Real::zero()),
    ])
    .expect("positive rectangle dimensions form a valid contour");
    Region2::from_material_contours(vec![contour])
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }

    let first = rectangle(data[0], data[1], data[2], data[3]);
    let second = rectangle(data[4], data[5], data[6], data[7]);
    let policy = CurvePolicy::certified();
    let first_view = first.as_view();
    let second_view = second.as_view();
    let first_prepared = first.prepare_topology_queries(&policy);
    let second_prepared = second.prepare_topology_queries(&policy);

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let direct = first_view.boolean_region(&second_view, op, FillRule::EvenOdd, &policy);
        let prepared =
            first_prepared.boolean_region(&second_prepared, op, FillRule::EvenOdd, &policy);
        assert_eq!(direct, prepared);
    }

    let query = Point2::new(r(data[8] as i32 - 128), r(data[9] as i32 - 128));
    assert_eq!(
        first_view.classify_point(&query, &policy),
        first_prepared.classify_point(&query, &policy),
    );
});
