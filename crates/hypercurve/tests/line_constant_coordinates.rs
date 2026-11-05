use hypercurve::{LineSeg2, Point2};
use hyperreal::{Rational, Real};

fn rational(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

#[test]
fn line_subdivision_preserves_constant_coordinate_identity() {
    let x = (Real::from(5).sqrt().unwrap() - Real::from(3)) / Real::from(6);
    let x = x.unwrap();
    let line = LineSeg2::try_new(
        Point2::new(x.clone(), Real::zero()),
        Point2::new(x.clone(), Real::one()),
    )
    .unwrap();

    let interior = line.point_at(rational(2, 5));
    assert_eq!(interior.x(), &x);
}

#[test]
fn native_lines_locate_selected_points_in_their_original_chart() {
    use hypercurve::{Curve2, CurveParameter2, CurveParameterRange2, CurvePointLocations2};
    use std::cmp::Ordering;
    let roots = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-1, 1, 0, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap();
    let alpha: CurveParameter2 = roots[0].clone().into();
    let source = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(2, 1)).unwrap(),
    );
    let query = source.point_at(&alpha).unwrap();
    assert!(query.coordinates().is_none());
    let query = query.definition().unwrap().into_point().unwrap();
    let target = Curve2::from(
        LineSeg2::try_new(Point2::from_values(-2, -1), Point2::from_values(4, 2)).unwrap(),
    );
    let third = rational(1, 3);
    let expected = alpha.affine_image(&third, &third).unwrap();
    for (curve, expected) in [
        (target.clone(), expected.clone()),
        (
            target.reversed().unwrap(),
            expected
                .affine_image(&(-Real::one()), &Real::one())
                .unwrap(),
        ),
        (
            target
                .restrict_source(
                    CurveParameterRange2::try_new(expected.clone(), Real::one().into()).unwrap(),
                )
                .unwrap(),
            expected,
        ),
    ] {
        let CurvePointLocations2::Locations(locations) = curve.point_locations(&query).unwrap()
        else {
            panic!("one finite parameter");
        };
        assert_eq!(locations.len(), 1);
        let parameter = locations[0].parameter().unwrap();
        assert_eq!(parameter.compare(&expected).unwrap(), Ordering::Equal);
        assert!(
            curve
                .point_at(&parameter)
                .unwrap()
                .coincides_with(&query)
                .unwrap()
        );
    }
    let clipped = target
        .restrict_source(
            CurveParameterRange2::try_new(Real::zero().into(), rational(1, 2).into()).unwrap(),
        )
        .unwrap();
    assert!(
        matches!(clipped.point_locations(&query).unwrap(), CurvePointLocations2::Locations(v) if v.is_empty())
    );
    let off_support = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 1), Point2::from_values(2, 2)).unwrap(),
    );
    assert!(
        matches!(off_support.point_locations(&query).unwrap(), CurvePointLocations2::Locations(v) if v.is_empty())
    );
    let epsilon = rational(1, 1_u64 << 60);
    let nearby = Curve2::from(
        LineSeg2::try_new(
            Point2::new(Real::zero(), epsilon.clone()),
            Point2::new(2.into(), Real::one() + epsilon),
        )
        .unwrap(),
    );
    assert!(
        matches!(nearby.point_locations(&query).unwrap(), CurvePointLocations2::Locations(v) if v.is_empty())
    );
}

#[test]
fn selected_chord_reversal_transports_the_point_order_chart() {
    use hypercurve::{Curve2, CurveParameter2};
    use std::cmp::Ordering;
    let roots = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-1, 1, 0, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap();
    let alpha: CurveParameter2 = roots[0].clone().into();
    let point = |y| {
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(0, y), Point2::from_values(2, y)).unwrap(),
        )
        .point_at(&alpha)
        .unwrap()
    };
    let chord = Curve2::try_line(point(0), point(2)).unwrap();
    let reversed = chord.reversed().unwrap();
    let (scale, offset) = chord.reversal_parameter_affine_map().unwrap();
    assert_eq!(
        (scale.clone(), offset.clone()),
        (-Real::one(), Real::zero())
    );
    for point in [point(0), point(1), point(2)] {
        let parameter = match chord.point_locations(&point).unwrap() {
            hypercurve::CurvePointLocations2::Locations(locations) => {
                locations[0].parameter().unwrap()
            }
            _ => panic!("finite line"),
        };
        let transported = parameter.affine_image(&scale, &offset).unwrap();
        assert!(
            reversed
                .point_at(&transported)
                .unwrap()
                .coincides_with(&point)
                .unwrap()
        );
        let restored = transported.affine_image(&scale, &offset).unwrap();
        assert_eq!(restored.compare(&parameter).unwrap(), Ordering::Equal);
    }
}
