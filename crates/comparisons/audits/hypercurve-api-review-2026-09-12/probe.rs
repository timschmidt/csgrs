//! Standalone architectural probes; no changes to the Hypercurve kernel.
use hypercurve::{
    BooleanOp, Classification, Curve2, CurveBoundaryInteriorSide2, CurveContext, CurveOutcome,
    CurvePath2, CurveRegion2, CurveRegionLoopRole, ExactCurveResult, FillRule, LineSeg2,
    OffsetCornerStyle2, Point2, QuadraticBezier2, RationalBezier2, RegionPointLocation,
};
use hyperreal::Real;

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}
fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn line(a: Point2, b: Point2) -> Curve2 {
    LineSeg2::try_new(a, b).unwrap().into()
}
fn region(path: CurvePath2) -> CurveRegion2 {
    CurveRegion2::try_from_boundary_paths_with_loop_topology(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &[CurveBoundaryInteriorSide2::Left],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value()
}
fn show_region(label: &str, result: ExactCurveResult<CurveOutcome<CurveRegion2>>) {
    println!(
        "{label}: {:?}",
        result.map(|r| (r.certainty, r.value.boundary_loops().len()))
    );
}

fn cap_path() -> CurvePath2 {
    // Positive weights: W(t) = 1 + 5t + t^5. The cap is simple and pole free.
    let curve = RationalBezier2::try_new(
        vec![p(0, 0), p(1, 1), p(2, 2), p(3, 2), p(4, 1), p(5, 0)],
        [1, 2, 3, 4, 5, 7].into_iter().map(Real::from).collect(),
    )
    .unwrap();
    CurvePath2::try_new(vec![curve.into(), line(p(5, 0), p(0, 0))]).unwrap()
}

fn orientation() {
    let path = cap_path();
    let ordinary = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        std::slice::from_ref(&path),
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &CurveContext::STRICT,
    )
    .unwrap();
    println!("orientation: construction {:?}", ordinary.certainty);
    let ordinary = ordinary.into_value();
    println!(
        "orientation: area {:?}",
        ordinary.signed_area(&CurveContext::STRICT)
    );
    println!(
        "orientation: filled side {:?}",
        ordinary.filled_side_is_left(&CurveContext::STRICT)
    );
    show_region(
        "orientation: offset",
        ordinary.offset(q(1, 10), &OffsetCornerStyle2::Round, &CurveContext::STRICT),
    );
    let supplied = CurveRegion2::try_from_boundary_paths_with_loop_topology(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &[CurveBoundaryInteriorSide2::Right],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    println!(
        "orientation: supplied side {:?}",
        supplied.filled_side_is_left(&CurveContext::STRICT)
    );
}

fn normalization() {
    let ordinary = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[cap_path()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    match ordinary.regularized_region(&CurveContext::STRICT) {
        Ok(normalized) => {
            println!(
                "normalization: {:?}, {} loops",
                normalized.certainty,
                normalized.value.boundary_loops().len()
            );
            println!(
                "normalization: filled side {:?}",
                normalized.value.filled_side_is_left(&CurveContext::STRICT)
            );
            println!(
                "normalization: area {:?}",
                normalized.value.signed_area(&CurveContext::STRICT)
            );
            if std::env::args().nth(2).as_deref() == Some("offset") {
                show_region(
                    "normalization: offset",
                    normalized.value.offset(
                        q(1, 10),
                        &OffsetCornerStyle2::Round,
                        &CurveContext::STRICT,
                    ),
                );
            }
        }
        Err(error) => println!("normalization: {error:?}"),
    }
}

fn carriers() {
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(p(1, 0), p(1, 1), p(0, 1)).into(),
        QuadraticBezier2::new(p(0, 1), p(-1, 1), p(-1, 0)).into(),
        QuadraticBezier2::new(p(-1, 0), p(-1, -1), p(0, -1)).into(),
        QuadraticBezier2::new(p(0, -1), p(1, -1), p(1, 0)).into(),
    ])
    .unwrap();
    let offset = region(path)
        .offset(q(1, 10), &OffsetCornerStyle2::Round, &CurveContext::STRICT)
        .unwrap();
    println!("carriers: offset {:?}", offset.certainty);
    let offset = offset.into_value();
    println!(
        "carriers: paths {:?}",
        offset
            .materialized_boundary_paths(&CurveContext::STRICT)
            .map(|r| r.map(|c| c.map(|p| p.len())))
    );
    show_region(
        "carriers: affine shear",
        offset.transform_affine(
            &Real::one(),
            &Real::one(),
            &Real::zero(),
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
            &CurveContext::STRICT,
        ),
    );
}

fn power_curve(degree: usize, height: i32) -> Curve2 {
    RationalBezier2::try_new(
        (0..=degree)
            .map(|i| {
                Point2::new(
                    q(i as i32, degree as i32),
                    Real::from(if i == degree { height } else { 0 }),
                )
            })
            .collect(),
        vec![Real::one(); degree + 1],
    )
    .unwrap()
    .into()
}
fn lens(degree: usize, lower: i32, upper: i32) -> CurveRegion2 {
    region(
        CurvePath2::try_new(vec![
            power_curve(degree, lower),
            line(p(1, lower), p(1, upper)),
            power_curve(degree, upper)
                .reversed(&CurveContext::STRICT)
                .unwrap()
                .into_value(),
        ])
        .unwrap(),
    )
}
fn tangent(degree: usize) {
    // Disjoint interiors; a single common point at (0,0). All derivatives
    // below `degree` agree at that point on all four nonlinear branches.
    let first = lens(degree, 1, 2);
    let second = lens(degree, 3, 4);
    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let result = first
            .boolean_region(&second, op, &CurveContext::STRICT)
            .unwrap();
        let expected_loops = match op {
            BooleanOp::Union | BooleanOp::Xor => 2,
            BooleanOp::Intersection => 0,
            BooleanOp::Difference => 1,
        };
        assert_eq!(result.value.boundary_loops().len(), expected_loops);
        for numerator in [1, 3, 5, 7, 9] {
            let in_first = numerator == 3;
            let in_second = numerator == 7;
            let expected_inside = match op {
                BooleanOp::Union => in_first || in_second,
                BooleanOp::Intersection => in_first && in_second,
                BooleanOp::Difference => in_first && !in_second,
                BooleanOp::Xor => in_first != in_second,
            };
            let sample = Point2::new(q(1, 2), q(numerator, 2_i32.pow(degree as u32 + 1)));
            let location = result
                .value
                .classify_point(&sample, &CurveContext::STRICT)
                .unwrap();
            assert_eq!(
                location.value,
                Classification::Decided(if expected_inside {
                    RegionPointLocation::Inside
                } else {
                    RegionPointLocation::Outside
                })
            );
        }
        show_region(
            &format!("tangent degree {degree} {op:?}; five analytic membership checks passed"),
            Ok(result),
        );
    }
}

fn policy() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Real::one();
    let x = Real::pi() + Real::e();
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(
            Point2::new(x.clone(), Real::zero()),
            p(0, 1),
            Point2::new(x + zero, Real::zero()),
        )
        .into(),
    ])
    .unwrap();
    let authored =
        CurveRegion2::try_from_boundary_paths(&[path], &CurveContext::APPROXIMATE_512).unwrap();
    println!("policy: construction {:?}", authored.certainty);
    let copied = authored
        .value
        .boolean_region(
            &CurveRegion2::empty(),
            BooleanOp::Union,
            &CurveContext::STRICT,
        )
        .unwrap();
    println!("policy: identity union {:?}", copied.certainty);
    let exported = copied
        .value
        .materialized_boundary_paths(&CurveContext::STRICT)
        .unwrap();
    assert!(matches!(exported.value, Classification::Uncertain(_)));
    println!(
        "policy: strict closing seam {:?}",
        exported.map(|r| r.map(|p| p.len()))
    );
}

fn run() {
    match std::env::args().nth(1).as_deref() {
        Some("orientation") => orientation(),
        Some("normalization") => normalization(),
        Some("carriers") => carriers(),
        Some("policy") => policy(),
        Some("tangent") => tangent(std::env::args().nth(2).unwrap().parse().unwrap()),
        _ => {
            panic!("usage: probe orientation|normalization [offset]|carriers|policy|tangent DEGREE")
        }
    }
}

fn main() {
    #[cfg(feature = "dispatch-trace")]
    {
        hyperreal::dispatch_trace::reset();
        hyperreal::dispatch_trace::with_recording(run);
        for entry in hyperreal::dispatch_trace::take_trace().dispatch {
            if entry.layer == "hypercurve" {
                println!(
                    "dispatch/{}/{}/{}={}",
                    entry.layer, entry.operation, entry.path, entry.count
                );
            }
        }
    }
    #[cfg(not(feature = "dispatch-trace"))]
    run();
}
