use hypercurve::{
    BezierEndpoint, Classification, CubicBezier2, CubicBezierHermiteInterpolationStage2,
    CurvePolicy, EndpointTangent2, Point2, QuadraticBezier2,
    QuadraticBezierMidpointInterpolationStage2, QuadraticBezierPointInterpolationStage2, Real,
    UncertaintyReason,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

#[test]
fn cubic_hermite_interpolation_solves_controls_and_replays_endpoint_tangents() {
    let start_tangent = EndpointTangent2::new(r(3), r(6));
    let end_tangent = EndpointTangent2::new(r(6), r(-3));
    let result = CubicBezier2::interpolate_hermite_with_report(
        p(0, 0),
        start_tangent.clone(),
        p(6, 0),
        end_tangent.clone(),
    )
    .unwrap();
    let report = result.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        CubicBezierHermiteInterpolationStage2::SegmentMaterialization
    );
    assert_eq!(report.start_point(), &p(0, 0));
    assert_eq!(report.start_tangent(), &start_tangent);
    assert_eq!(report.end_point(), &p(6, 0));
    assert_eq!(report.end_tangent(), &end_tangent);
    assert_eq!(report.solved_first_control_point(), Some(&p(1, 2)));
    assert_eq!(report.solved_second_control_point(), Some(&p(4, 1)));
    assert_eq!(report.replayed_start_tangent(), Some(&start_tangent));
    assert_eq!(report.replayed_end_tangent(), Some(&end_tangent));
    assert_eq!(report.blocker(), None);

    let curve = result
        .curve()
        .expect("Hermite interpolation should materialize");
    assert_eq!(curve.start(), &p(0, 0));
    assert_eq!(curve.control1(), &p(1, 2));
    assert_eq!(curve.control2(), &p(4, 1));
    assert_eq!(curve.end(), &p(6, 0));
    assert_eq!(curve.endpoint_tangent(BezierEndpoint::Start), start_tangent);
    assert_eq!(curve.endpoint_tangent(BezierEndpoint::End), end_tangent);
}

#[test]
fn cubic_hermite_interpolation_preserves_zero_endpoint_derivative_evidence() {
    let curve = CubicBezier2::interpolate_hermite(
        p(0, 0),
        EndpointTangent2::new(r(0), r(0)),
        p(3, 0),
        EndpointTangent2::new(r(0), r(0)),
    )
    .unwrap();

    assert_eq!(curve.control1(), &p(0, 0));
    assert_eq!(curve.control2(), &p(3, 0));
    assert_eq!(
        curve.endpoint_tangent(BezierEndpoint::Start),
        EndpointTangent2::new(r(0), r(0))
    );
    assert_eq!(
        curve.endpoint_tangent(BezierEndpoint::End),
        EndpointTangent2::new(r(0), r(0))
    );
}

#[test]
fn quadratic_point_interpolation_solves_non_midpoint_control_and_replays_constraint() {
    let t = q(1, 4);
    let result = QuadraticBezier2::interpolate_point_at_parameter_with_report(
        p(0, 0),
        t.clone(),
        Point2::new(r(1), q(3, 2)),
        p(4, 0),
        &policy(),
    )
    .unwrap();
    let report = result.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        QuadraticBezierPointInterpolationStage2::SegmentMaterialization
    );
    assert_eq!(report.interpolation_parameter(), &t);
    assert_eq!(report.start_point(), &p(0, 0));
    assert_eq!(report.interpolation_point(), &Point2::new(r(1), q(3, 2)));
    assert_eq!(report.end_point(), &p(4, 0));
    assert_eq!(report.solved_control_point(), Some(&p(2, 4)));
    assert_eq!(report.replayed_point(), Some(&Point2::new(r(1), q(3, 2))));
    assert_eq!(report.blocker(), None);

    let curve = result
        .curve()
        .expect("interior parameter should materialize");
    assert_eq!(curve.control(), &p(2, 4));
    assert_eq!(curve.point_at(t), Point2::new(r(1), q(3, 2)));
}

#[test]
fn quadratic_point_interpolation_convenience_returns_decided_curve() {
    let curve = QuadraticBezier2::interpolate_point_at_parameter(
        p(0, 0),
        q(3, 4),
        Point2::new(r(3), q(3, 2)),
        p(4, 0),
        &policy(),
    )
    .unwrap();

    let Classification::Decided(curve) = curve else {
        panic!("interior exact interpolation should decide");
    };
    assert_eq!(curve.control(), &p(2, 4));
    assert_eq!(curve.point_at(q(3, 4)), Point2::new(r(3), q(3, 2)));
}

#[test]
fn quadratic_point_interpolation_reports_endpoint_parameter_blocker() {
    let result = QuadraticBezier2::interpolate_point_at_parameter_with_report(
        p(0, 0),
        r(0),
        p(0, 0),
        p(4, 0),
        &policy(),
    )
    .unwrap();
    let report = result.report();

    assert!(result.curve().is_none());
    assert!(report.status().is_retained_evidence());
    assert_eq!(
        report.stage(),
        QuadraticBezierPointInterpolationStage2::ParameterValidation
    );
    assert_eq!(report.interpolation_parameter(), &r(0));
    assert_eq!(report.solved_control_point(), None);
    assert_eq!(report.replayed_point(), None);
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn quadratic_point_interpolation_reports_out_of_domain_parameter_blocker() {
    let result = QuadraticBezier2::interpolate_point_at_parameter_with_report(
        p(0, 0),
        r(2),
        p(8, 0),
        p(4, 0),
        &policy(),
    )
    .unwrap();
    let report = result.report();

    assert!(result.curve().is_none());
    assert!(report.status().is_retained_evidence());
    assert_eq!(
        report.stage(),
        QuadraticBezierPointInterpolationStage2::ParameterValidation
    );
    assert_eq!(report.interpolation_parameter(), &r(2));
    assert_eq!(report.solved_control_point(), None);
    assert_eq!(report.replayed_point(), None);
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn quadratic_point_interpolation_convenience_returns_boundary_uncertainty() {
    let result = QuadraticBezier2::interpolate_point_at_parameter(
        p(0, 0),
        r(1),
        p(4, 0),
        p(4, 0),
        &policy(),
    )
    .unwrap();

    assert_eq!(
        result,
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
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
