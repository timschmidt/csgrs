use hypercurve::{
    CurveError, CurveFamily2, CurveOperation2, CurvePolicy, CurveSource2, ExactCurveError,
    NurbsCurve2, NurbsInterpolationParameterization2, NurbsInterpolationSolvePath2, Point2,
    RationalBezier2, Real,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

#[test]
fn uniform_quadratic_nurbs_interpolation_retains_exact_bareiss_evidence() {
    let source = CurveSource2::with_version(160, 7);
    let interpolation =
        NurbsCurve2::interpolate_uniform_with_source(2, vec![p(0, 0), p(2, 2), p(4, 0)], source)
            .unwrap();
    let curve = interpolation.curve();
    let report = interpolation.report();

    assert_eq!(curve.degree(), 2);
    assert_eq!(curve.source(), Some(source));
    assert_eq!(curve.control_points(), &[p(0, 0), p(2, 4), p(4, 0)]);
    assert_eq!(curve.weights(), &[r(1), r(1), r(1)]);
    assert_eq!(curve.knots(), &[r(0), r(0), r(0), r(1), r(1), r(1)]);
    assert_eq!(report.source(), Some(source));
    assert_eq!(report.parameters(), &[r(0), q(1, 2), r(1)]);
    assert_eq!(report.data_points(), &[p(0, 0), p(2, 2), p(4, 0)]);
    assert_eq!(
        report.coefficient_matrix()[1],
        vec![q(1, 4), q(1, 2), q(1, 4)]
    );
    assert_eq!(report.determinant(), &q(1, 2));
    assert_eq!(
        report.solve_path(),
        NurbsInterpolationSolvePath2::DenseBareissCramerResidualReplay
    );
    assert_eq!(
        report.parameterization(),
        NurbsInterpolationParameterization2::Uniform
    );
    assert_eq!(curve.point_at(&q(1, 2)).unwrap(), p(2, 2));
    assert!(curve.is_bezier_decomposition_cached());
}

#[test]
fn chord_length_and_centripetal_interpolation_retain_exact_parameters() {
    let points = vec![p(0, 0), p(1, 0), p(5, 0), p(14, 0)];
    let chord = NurbsCurve2::interpolate_chord_length(2, points.clone()).unwrap();
    assert_eq!(
        chord.report().parameterization(),
        NurbsInterpolationParameterization2::ChordLength
    );
    assert_eq!(
        chord.report().parameters(),
        &[r(0), q(1, 14), q(5, 14), r(1)]
    );

    let centripetal = NurbsCurve2::interpolate_centripetal(2, points.clone()).unwrap();
    assert_eq!(
        centripetal.report().parameterization(),
        NurbsInterpolationParameterization2::Centripetal
    );
    assert_eq!(
        centripetal.report().parameters(),
        &[r(0), q(1, 6), q(1, 2), r(1)]
    );
    for (parameter, point) in centripetal.report().parameters().iter().zip(points) {
        assert_eq!(centripetal.curve().point_at(parameter).unwrap(), point);
    }
}

#[test]
fn symbolic_centripetal_interpolation_retains_exact_cramer_identity() {
    let source = CurveSource2::new(164);
    let points = vec![p(0, 0), p(1, 0), p(3, 0), p(6, 0)];
    let interpolation =
        NurbsCurve2::interpolate_centripetal_with_source(2, points.clone(), source).unwrap();
    let sqrt_two = r(2).sqrt().unwrap();
    let sqrt_three = r(3).sqrt().unwrap();
    let total = r(1) + &sqrt_two + &sqrt_three;
    let first = (r(1) / total.clone()).unwrap();
    let second = ((r(1) + sqrt_two) / total).unwrap();

    assert_eq!(interpolation.curve().source(), Some(source));
    assert_eq!(interpolation.report().data_points(), points);
    assert_eq!(
        interpolation.report().parameters(),
        &[r(0), first, second, r(1)]
    );
    assert_eq!(
        interpolation.report().parameterization(),
        NurbsInterpolationParameterization2::Centripetal
    );
    assert_eq!(
        interpolation.report().solve_path(),
        NurbsInterpolationSolvePath2::DenseBareissCramerIdentity
    );
    for (parameter, expected) in interpolation.report().parameters().iter().zip(&points) {
        let actual = interpolation.curve().point_at(parameter).unwrap();
        let dx = actual.x().to_f64_lossy().unwrap() - expected.x().to_f64_lossy().unwrap();
        let dy = actual.y().to_f64_lossy().unwrap() - expected.y().to_f64_lossy().unwrap();
        assert!(dx.abs() < 1.0e-12);
        assert!(dy.abs() < 1.0e-12);
    }
}

#[test]
fn distance_parameterization_rejects_coincident_consecutive_constraints() {
    let source = CurveSource2::new(163);
    for error in [
        NurbsCurve2::interpolate_chord_length_with_source(
            2,
            vec![p(0, 0), p(0, 0), p(2, 1)],
            source,
        )
        .unwrap_err(),
        NurbsCurve2::interpolate_centripetal_with_source(
            2,
            vec![p(0, 0), p(0, 0), p(2, 1)],
            source,
        )
        .unwrap_err(),
    ] {
        assert_eq!(error.operation(), CurveOperation2::Interpolation);
        assert_eq!(error.source(), Some(source));
        assert!(matches!(
            error,
            ExactCurveError::Invalid {
                cause: CurveError::InvalidNurbsInterpolation,
                ..
            }
        ));
    }
}

#[test]
fn fixed_weight_rational_nurbs_interpolation_recovers_exact_control_net() {
    let controls = vec![p(0, 0), p(2, 4), p(4, 0)];
    let weights = vec![r(1), r(2), r(1)];
    let source_curve = RationalBezier2::try_new(controls.clone(), weights.clone()).unwrap();
    let parameters = vec![r(0), q(1, 2), r(1)];
    let data_points = parameters
        .iter()
        .map(|parameter| {
            source_curve
                .point_at(parameter, &CurvePolicy::certified())
                .unwrap()
        })
        .collect::<Vec<_>>();
    let interpolation = NurbsCurve2::interpolate_with_parameters_and_knots(
        2,
        data_points.clone(),
        parameters.clone(),
        weights.clone(),
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
    )
    .unwrap();

    assert_eq!(interpolation.curve().control_points(), controls);
    assert_eq!(interpolation.curve().weights(), weights);
    for (parameter, point) in parameters.iter().zip(data_points) {
        assert_eq!(interpolation.curve().point_at(parameter).unwrap(), point);
    }
}

#[test]
fn nonuniform_global_interpolation_derives_averaged_knots_and_replays_every_point() {
    let parameters = vec![r(2), r(3), r(5), r(8), r(12)];
    let points = vec![p(0, 0), p(1, 3), p(4, 2), p(7, 5), p(9, 0)];
    let interpolation =
        NurbsCurve2::interpolate_global(2, points.clone(), parameters.clone()).unwrap();

    assert_eq!(
        interpolation.report().knots(),
        &[r(2), r(2), r(2), r(4), q(13, 2), r(12), r(12), r(12)]
    );
    for (parameter, point) in parameters.iter().zip(points) {
        assert_eq!(interpolation.curve().point_at(parameter).unwrap(), point);
    }
}

#[test]
fn nurbs_interpolation_rejects_invalid_and_singular_systems_with_context() {
    let source = CurveSource2::new(161);
    let invalid = NurbsCurve2::interpolate_global_with_source(
        2,
        vec![p(0, 0), p(1, 1), p(2, 0)],
        vec![r(0), r(0), r(1)],
        source,
    )
    .unwrap_err();
    assert_eq!(invalid.operation(), CurveOperation2::Interpolation);
    assert_eq!(invalid.family(), CurveFamily2::Nurbs);
    assert_eq!(invalid.source(), Some(source));
    assert!(matches!(
        invalid,
        ExactCurveError::Invalid {
            cause: CurveError::InvalidNurbsInterpolation,
            ..
        }
    ));

    let singular = NurbsCurve2::interpolate_with_parameters_and_knots_with_source(
        1,
        vec![p(0, 0), p(1, 1), p(2, 0)],
        vec![r(0), r(1), r(2)],
        vec![r(1), r(1), r(1)],
        vec![r(0), r(0), r(0), r(2), r(2)],
        source,
    )
    .unwrap_err();
    assert_eq!(singular.operation(), CurveOperation2::Interpolation);
    assert_eq!(singular.source(), Some(source));
    assert!(matches!(
        singular,
        ExactCurveError::Invalid {
            cause: CurveError::SingularNurbsInterpolation { .. },
            ..
        }
    ));
}

#[test]
fn rational_interpolation_reports_zero_denominator_during_curve_replay() {
    let source = CurveSource2::new(162);
    let error = NurbsCurve2::interpolate_with_parameters_and_knots_with_source(
        2,
        vec![p(0, 0), p(1, 1), p(2, 0)],
        vec![r(0), q(1, 2), r(1)],
        vec![r(1), r(-1), r(1)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        source,
    )
    .unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::Interpolation);
    assert_eq!(error.source(), Some(source));
    assert!(
        matches!(
            &error,
            ExactCurveError::Invalid {
                cause: CurveError::ZeroNurbsDenominator,
                ..
            }
        ),
        "{error:?}"
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(32))]

    #[test]
    fn generated_uniform_cubic_interpolation_recovers_exact_polynomial_controls(
        coordinates in prop::collection::vec(-8_i32..=8, 8)
    ) {
        let controls = coordinates
            .chunks_exact(2)
            .map(|coordinate| p(coordinate[0], coordinate[1]))
            .collect::<Vec<_>>();
        let source_curve = RationalBezier2::try_new(controls.clone(), vec![r(1); 4]).unwrap();
        let parameters = [r(0), q(1, 3), q(2, 3), r(1)];
        let data_points = parameters
            .iter()
            .map(|parameter| {
                source_curve
                    .point_at(parameter, &CurvePolicy::certified())
                    .unwrap()
            })
            .collect::<Vec<_>>();

        let interpolation = NurbsCurve2::interpolate_uniform(3, data_points).unwrap();

        prop_assert_eq!(interpolation.curve().control_points(), controls.as_slice());
        prop_assert_eq!(interpolation.curve().weights(), &[r(1), r(1), r(1), r(1)]);
    }
}
