use hypercurve::{Point2, QuadraticBezier2, QuadraticBezierMidpointInterpolationStage2, Real};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

#[test]
fn quadratic_midpoint_interpolation_solves_control_and_replays_constraint() {
    let result =
        QuadraticBezier2::interpolate_midpoint_with_report(p(0, 0), p(2, 3), p(4, 0)).unwrap();
    let report = result.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        QuadraticBezierMidpointInterpolationStage2::SegmentMaterialization
    );
    assert_eq!(report.interpolation_parameter(), &q(1, 2));
    assert_eq!(report.start_point(), &p(0, 0));
    assert_eq!(report.midpoint_constraint(), &p(2, 3));
    assert_eq!(report.end_point(), &p(4, 0));
    assert_eq!(report.solved_control_point(), Some(&p(2, 6)));
    assert_eq!(report.replayed_midpoint(), Some(&p(2, 3)));
    assert_eq!(report.blocker(), None);

    let curve = result
        .curve()
        .expect("exact interpolation should materialize");
    assert_eq!(curve.start(), &p(0, 0));
    assert_eq!(curve.control(), &p(2, 6));
    assert_eq!(curve.end(), &p(4, 0));
    assert_eq!(curve.point_at(q(1, 2)), p(2, 3));
}

#[test]
fn quadratic_midpoint_interpolation_preserves_exact_collinear_shape() {
    let curve = QuadraticBezier2::interpolate_midpoint(p(0, 0), p(2, 0), p(4, 0)).unwrap();

    assert_eq!(curve.start(), &p(0, 0));
    assert_eq!(curve.control(), &p(2, 0));
    assert_eq!(curve.end(), &p(4, 0));
    assert_eq!(curve.point_at(q(1, 2)), p(2, 0));
}
