use hypercurve::{CurveContext, NurbsCurve2, Point2, RationalBezier2, Real};
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
fn chord_length_and_centripetal_interpolation_retain_exact_parameters() {
    let points = vec![p(0, 0), p(1, 0), p(5, 0), p(14, 0)];
    let chord = NurbsCurve2::interpolate_chord_length(2, points.clone()).unwrap();
    let chord_parameters = [r(0), q(1, 14), q(5, 14), r(1)];
    for (parameter, point) in chord_parameters.iter().zip(&points) {
        assert_eq!(chord.point_at(parameter).unwrap(), point.clone());
    }

    let centripetal = NurbsCurve2::interpolate_centripetal(2, points.clone()).unwrap();
    let centripetal_parameters = [r(0), q(1, 6), q(1, 2), r(1)];
    for (parameter, point) in centripetal_parameters.iter().zip(points) {
        assert_eq!(centripetal.point_at(parameter).unwrap(), point);
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
                .point_at(parameter, &CurveContext::STRICT)
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

    assert_eq!(interpolation.control_points(), controls);
    assert_eq!(interpolation.weights(), weights);
    for (parameter, point) in parameters.iter().zip(data_points) {
        assert_eq!(interpolation.point_at(parameter).unwrap(), point);
    }
}

#[test]
fn nonuniform_global_interpolation_derives_averaged_knots_and_replays_every_point() {
    let parameters = vec![r(2), r(3), r(5), r(8), r(12)];
    let points = vec![p(0, 0), p(1, 3), p(4, 2), p(7, 5), p(9, 0)];
    let interpolation =
        NurbsCurve2::interpolate_global(2, points.clone(), parameters.clone()).unwrap();

    assert_eq!(
        interpolation.knots(),
        &[r(2), r(2), r(2), r(4), q(13, 2), r(12), r(12), r(12)]
    );
    for (parameter, point) in parameters.iter().zip(points) {
        assert_eq!(interpolation.point_at(parameter).unwrap(), point);
    }
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
                    .point_at(parameter, &CurveContext::STRICT)
                    .unwrap()
            })
            .collect::<Vec<_>>();

        let interpolation = NurbsCurve2::interpolate_uniform(3, data_points).unwrap();

        prop_assert_eq!(interpolation.control_points(), controls.as_slice());
        prop_assert_eq!(interpolation.weights(), &[r(1), r(1), r(1), r(1)]);
    }
}
