//! A probe with unresolved or zero tangent cross cannot omit a boundary event.
use hypercurve::{
    Curve2, CurveParameterRange2, CurvePath2, CurveRegion2, FillRule, LineSeg2, Point2,
    PolynomialCurveSet2, Real, RegionPointLocation,
};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn branches(equation: BivariatePolynomial, y: [i32; 2]) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [y[0].into(), y[1].into()],
        },
        Default::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap()
}

fn line(start: Point2, end: Point2) -> Curve2 {
    LineSeg2::try_new(start, end).unwrap().into()
}

fn parabola_path() -> CurvePath2 {
    let source = branches(
        BivariatePolynomial::new(vec![vec![0.into(), 0.into(), 1.into()], vec![(-1).into()]]),
        [-2, 2],
    );
    assert_eq!(source.curves().len(), 2);
    let range = CurveParameterRange2::try_new(Real::zero().into(), q(1, 4).into()).unwrap();
    CurvePath2::try_new(vec![
        source.curves()[0].restrict_source(range.clone()).unwrap(),
        line(
            Point2::new(q(1, 4), q(-1, 2)),
            Point2::new(q(1, 4), q(1, 2)),
        ),
        source.curves()[1]
            .restrict_source(range)
            .unwrap()
            .reversed()
            .unwrap(),
    ])
    .unwrap()
}

fn cubic_path(y: [i32; 2]) -> CurvePath2 {
    // y - x^3 - 2x = 0, traversed from (-1,-3) to (1,3).
    let source = branches(
        BivariatePolynomial::new(vec![
            vec![0.into(), 1.into()],
            vec![(-2).into()],
            vec![],
            vec![(-1).into()],
        ]),
        y,
    );
    assert_eq!(source.curves().len(), 1);
    CurvePath2::try_new(vec![
        source.curves()[0].clone(),
        line(Point2::from_values(1, 3), Point2::from_values(1, -3)),
        line(Point2::from_values(1, -3), Point2::from_values(-1, -3)),
    ])
    .unwrap()
}

#[test]
fn parabola_interior_survives_unresolved_probe_contact() {
    // Independently: (3/16)^2 = 9/256 < 1/8 < 1/4.
    let region =
        CurveRegion2::try_from_boundary_paths(&[parabola_path()], FillRule::NonZero).unwrap();
    assert_eq!(
        region
            .classify_point(&Point2::new(q(1, 8), q(3, 16)).into())
            .unwrap(),
        RegionPointLocation::Inside
    );
}

#[test]
fn cubic_interior_survives_stationary_probe_crossing() {
    // Independently: -3 < 1 < (1/2)^3 + 2(1/2) = 9/8 and -1 < 1/2 < 1.
    // The line y=2x has an odd-order contact with this graph at the origin.
    for y in [[-3, 3], [-4, 4]] {
        let region =
            CurveRegion2::try_from_boundary_paths(&[cubic_path(y)], FillRule::NonZero).unwrap();
        assert_eq!(
            region
                .classify_point(&Point2::new(q(1, 2), 1.into()).into())
                .unwrap(),
            RegionPointLocation::Inside,
            "classification with enclosing y bounds {y:?}"
        );
    }
}

#[test]
fn polynomial_probe_contacts_survive_regularization_and_reconstruction() {
    for (path, inside) in [
        (parabola_path(), Point2::new(q(1, 8), q(3, 16))),
        (cubic_path([-3, 3]), Point2::new(q(1, 2), 1.into())),
        (cubic_path([-4, 4]), Point2::new(q(1, 2), 1.into())),
    ] {
        for path in [path.clone(), path.reversed().unwrap()] {
            for fill in [FillRule::NonZero, FillRule::EvenOdd] {
                let region =
                    CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path), fill)
                        .unwrap();
                let regularized = region.regularized_region().unwrap();
                let rebuilt = CurveRegion2::try_from_boundary_paths(
                    &regularized.boundary_paths().unwrap(),
                    fill,
                )
                .unwrap();
                for result in [&region, &regularized, &rebuilt] {
                    assert_eq!(
                        result.classify_point(&inside.clone().into()).unwrap(),
                        RegionPointLocation::Inside
                    );
                    assert_eq!(
                        result
                            .classify_point(&Point2::from_values(2, 0).into())
                            .unwrap(),
                        RegionPointLocation::Outside
                    );
                }
                assert!(
                    rebuilt
                        .boolean_region(&region, hypercurve::BooleanOp::Xor)
                        .unwrap()
                        .is_empty()
                );
            }
        }
    }
}
