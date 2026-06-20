use hypercurve::{
    BezierEndpoint, BezierInterpolationReplayPath2, BezierInterpolationSolvePath2, Classification,
    CubicBezier2, CubicBezierHermiteInterpolationStage2, CurvePolicy, EndpointTangent2, Point2,
    QuadraticBezier2, QuadraticBezierMidpointInterpolationStage2,
    QuadraticBezierPointInterpolationStage2, Real, UncertaintyReason,
};
use proptest::prelude::*;

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

fn bounded_real() -> impl Strategy<Value = Real> {
    (-12_i32..=12).prop_map(r)
}

fn bounded_point() -> impl Strategy<Value = Point2> {
    (bounded_real(), bounded_real()).prop_map(|(x, y)| Point2::new(x, y))
}

fn interior_parameter() -> impl Strategy<Value = Real> {
    (1_i32..=5).prop_map(|numerator| q(numerator, 6))
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
    assert_eq!(
        report.solve_path(),
        Some(BezierInterpolationSolvePath2::CubicHermiteEndpointDerivatives)
    );
    assert_eq!(
        report.replay_path(),
        Some(BezierInterpolationReplayPath2::ExactEvaluationReplay)
    );
    assert_eq!(report.replayed_start_tangent(), Some(&start_tangent));
    assert_eq!(report.replayed_end_tangent(), Some(&end_tangent));
    assert_eq!(report.blocker(), None);
    assert!(matches!(
        result.curve_classification(),
        Classification::Decided(curve) if curve.control1() == &p(1, 2)
    ));
    let owned_report = result.clone().into_report();
    assert_eq!(&owned_report, result.report());
    let (owned_curve, owned_parts_report) = result.clone().into_parts();
    assert_eq!(owned_curve.as_ref(), result.curve());
    assert_eq!(&owned_parts_report, result.report());
    assert!(matches!(
        result.clone().into_curve_classification(),
        Classification::Decided(curve) if curve.control1() == &p(1, 2)
    ));

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
    let result = CubicBezier2::interpolate_hermite_with_report(
        p(0, 0),
        EndpointTangent2::new(r(0), r(0)),
        p(3, 0),
        EndpointTangent2::new(r(0), r(0)),
    )
    .unwrap();
    let report = result.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        CubicBezierHermiteInterpolationStage2::SegmentMaterialization
    );
    assert_eq!(
        report.replayed_start_tangent(),
        Some(&EndpointTangent2::new(r(0), r(0)))
    );
    assert_eq!(
        report.replayed_end_tangent(),
        Some(&EndpointTangent2::new(r(0), r(0)))
    );
    assert_eq!(report.blocker(), None);
    let curve = result
        .curve()
        .expect("Hermite interpolation should materialize");
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
    assert_eq!(
        report.solve_path(),
        Some(BezierInterpolationSolvePath2::QuadraticBernsteinInteriorPoint)
    );
    assert_eq!(
        report.replay_path(),
        Some(BezierInterpolationReplayPath2::ExactEvaluationReplay)
    );
    assert_eq!(report.replayed_point(), Some(&Point2::new(r(1), q(3, 2))));
    assert_eq!(report.blocker(), None);

    let curve = result
        .curve()
        .expect("interior parameter should materialize");
    assert_eq!(curve.control(), &p(2, 4));
    assert_eq!(curve.point_at(t), Point2::new(r(1), q(3, 2)));
}

#[test]
fn quadratic_point_interpolation_report_materializes_decided_curve() {
    let result = QuadraticBezier2::interpolate_point_at_parameter_with_report(
        p(0, 0),
        q(3, 4),
        Point2::new(r(3), q(3, 2)),
        p(4, 0),
        &policy(),
    )
    .unwrap();
    let report = result.report();

    assert!(report.status().is_native_exact());
    assert_eq!(report.blocker(), None);
    assert!(matches!(
        result.curve_classification(),
        Classification::Decided(curve) if curve.control() == &p(2, 4)
    ));
    let owned_report = result.clone().into_report();
    assert_eq!(&owned_report, result.report());
    let (owned_curve, owned_parts_report) = result.clone().into_parts();
    assert_eq!(owned_curve.as_ref(), result.curve());
    assert_eq!(&owned_parts_report, result.report());
    assert!(matches!(
        result.clone().into_curve_classification(),
        Classification::Decided(curve) if curve.control() == &p(2, 4)
    ));
    let curve = result
        .curve()
        .expect("interior exact interpolation should materialize");
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
    assert_eq!(report.solve_path(), None);
    assert_eq!(report.replay_path(), None);
    assert_eq!(report.replayed_point(), None);
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(
        result.curve_classification(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    let owned_report = result.clone().into_report();
    assert_eq!(&owned_report, result.report());
    let (owned_curve, owned_parts_report) = result.clone().into_parts();
    assert_eq!(owned_curve, None);
    assert_eq!(&owned_parts_report, result.report());
    assert_eq!(
        result.into_curve_classification(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
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
fn quadratic_point_interpolation_report_returns_boundary_blocker() {
    let result = QuadraticBezier2::interpolate_point_at_parameter_with_report(
        p(0, 0),
        r(1),
        p(4, 0),
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
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
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
    assert_eq!(
        report.solve_path(),
        Some(BezierInterpolationSolvePath2::QuadraticBernsteinMidpoint)
    );
    assert_eq!(
        report.replay_path(),
        Some(BezierInterpolationReplayPath2::ExactEvaluationReplay)
    );
    assert_eq!(report.replayed_midpoint(), Some(&p(2, 3)));
    assert_eq!(report.blocker(), None);
    assert!(matches!(
        result.curve_classification(),
        Classification::Decided(curve) if curve.control() == &p(2, 6)
    ));
    let owned_report = result.clone().into_report();
    assert_eq!(&owned_report, result.report());
    let (owned_curve, owned_parts_report) = result.clone().into_parts();
    assert_eq!(owned_curve.as_ref(), result.curve());
    assert_eq!(&owned_parts_report, result.report());
    assert!(matches!(
        result.clone().into_curve_classification(),
        Classification::Decided(curve) if curve.control() == &p(2, 6)
    ));

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
    let result =
        QuadraticBezier2::interpolate_midpoint_with_report(p(0, 0), p(2, 0), p(4, 0)).unwrap();
    let report = result.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        QuadraticBezierMidpointInterpolationStage2::SegmentMaterialization
    );
    assert_eq!(report.solved_control_point(), Some(&p(2, 0)));
    assert_eq!(report.replayed_midpoint(), Some(&p(2, 0)));
    assert_eq!(report.blocker(), None);
    let curve = result
        .curve()
        .expect("midpoint interpolation should materialize");
    assert_eq!(curve.start(), &p(0, 0));
    assert_eq!(curve.control(), &p(2, 0));
    assert_eq!(curve.end(), &p(4, 0));
    assert_eq!(curve.point_at(q(1, 2)), p(2, 0));
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 48,
        ..ProptestConfig::default()
    })]

    #[test]
    fn generated_quadratic_point_interpolation_replays_known_exact_curve(
        start in bounded_point(),
        control in bounded_point(),
        end in bounded_point(),
        t in interior_parameter(),
    ) {
        let source = QuadraticBezier2::new(start.clone(), control, end.clone());
        let point = source.point_at(t.clone());

        let result = QuadraticBezier2::interpolate_point_at_parameter_with_report(
            start,
            t.clone(),
            point.clone(),
            end,
            &policy(),
        )?;
        let report = result.report();

        prop_assert!(report.status().is_native_exact());
        prop_assert_eq!(
            report.stage(),
            QuadraticBezierPointInterpolationStage2::SegmentMaterialization
        );
        prop_assert_eq!(report.interpolation_parameter(), &t);
        prop_assert_eq!(report.interpolation_point(), &point);
        prop_assert_eq!(report.replayed_point(), Some(&point));
        prop_assert_eq!(report.blocker(), None);
        let curve = result.curve().expect("interior parameter should materialize");
        prop_assert_eq!(curve.point_at(t), point);
    }

    #[test]
    fn generated_cubic_hermite_interpolation_replays_endpoint_tangents(
        start in bounded_point(),
        control1 in bounded_point(),
        control2 in bounded_point(),
        end in bounded_point(),
    ) {
        let source = CubicBezier2::new(start.clone(), control1, control2, end.clone());
        let start_tangent = source.endpoint_tangent(BezierEndpoint::Start);
        let end_tangent = source.endpoint_tangent(BezierEndpoint::End);

        let result = CubicBezier2::interpolate_hermite_with_report(
            start,
            start_tangent.clone(),
            end,
            end_tangent.clone(),
        )?;
        let report = result.report();

        prop_assert!(report.status().is_native_exact());
        prop_assert_eq!(
            report.stage(),
            CubicBezierHermiteInterpolationStage2::SegmentMaterialization
        );
        prop_assert_eq!(report.replayed_start_tangent(), Some(&start_tangent));
        prop_assert_eq!(report.replayed_end_tangent(), Some(&end_tangent));
        prop_assert_eq!(report.blocker(), None);
        let curve = result.curve().expect("Hermite interpolation should materialize");
        prop_assert_eq!(curve.endpoint_tangent(BezierEndpoint::Start), start_tangent);
        prop_assert_eq!(curve.endpoint_tangent(BezierEndpoint::End), end_tangent);
    }
}
