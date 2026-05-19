use hypercurve::{BooleanOp, Classification, Contour2, CurvePolicy, FillRule, Region2, Real};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> hypercurve::BulgeVertex2 {
    hypercurve::BulgeVertex2::new(p(x, y), s(bulge))
}

fn contour(vertices: &[hypercurve::BulgeVertex2]) -> Contour2 {
    Contour2::from_bulge_vertices(vertices).unwrap()
}

fn rect(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    contour(&[
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
    ])
}

fn region(contours: Vec<Contour2>) -> Region2 {
    Region2::from_material_contours(contours)
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

#[test]
fn probe_antagonistic_pipeline_uncertainty_reasons() {
    let first = region(vec![rect(0, 0, 2, 2)]);
    let second = region(vec![rect(2, 2, 4, 4)]);
    let pol = policy();
    let op = BooleanOp::Union;

    let bcont = first.boolean_boundary_contours(&second, op, FillRule::NonZero, &pol);
    println!("boundary_contours => {:?}", bcont);
    let bcont_report = first.boolean_boundary_contour_report(&second, op, FillRule::NonZero, &pol);
    println!("boundary_contour_report => {:?}", bcont_report);
    let bl = first.boolean_region(&second, op, FillRule::NonZero, &pol);
    println!("region => {:?}", bl);
    let breg_report = first.boolean_region_report(&second, op, FillRule::NonZero, &pol);
    println!("region_report => {:?}", breg_report);
    let pipeline = first.boolean_region_pipeline_report(&second, op, FillRule::NonZero, &pol);
    println!("pipeline => {:?}", pipeline);
    match pipeline.unwrap() {
        Classification::Decided(report) => {
            println!("pipeline decided {} contours", report.boundary_contours.len());
        }
        Classification::Uncertain(reason) => {
            panic!("pipeline uncertain: {reason:?}");
        }
    }
}
