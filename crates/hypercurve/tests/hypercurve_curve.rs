use hypercurve::{
    BezierSplitFragment2, BezierSubcurve2, CircularArc2, Classification, CubicBezier2, Curve2,
    CurveContext, CurveCornerMode2, CurveCornerNoSolution2, CurveCornerSolutions2, CurveError,
    CurveFamily2, CurveGeometry2, CurveOperation2, CurvePath2, CurveRegion2, ExactCurveError,
    LineSeg2, Point2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2, Real,
    RegionPointLocation, UncertaintyReason,
};
use hypercurve::{ContourPointLocation, CurveCertainty};
use hyperreal::CertifiedRealEquality;
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

fn linear_family_curve(family: CurveFamily2, vertical: bool) -> Curve2 {
    let (start, middle, end) = if vertical {
        (p(0, 0), p(0, 1), p(0, 2))
    } else {
        (p(-2, 0), p(-1, 0), p(0, 0))
    };
    match family {
        CurveFamily2::Line => Curve2::from(LineSeg2::try_new(start, end).unwrap()),
        CurveFamily2::QuadraticBezier => Curve2::from(QuadraticBezier2::new(start, middle, end)),
        CurveFamily2::CubicBezier => {
            Curve2::from(CubicBezier2::new(start, middle.clone(), middle, end))
        }
        CurveFamily2::RationalQuadraticBezier => Curve2::from(
            RationalQuadraticBezier2::try_new(start, middle, end, r(1), r(1), r(1)).unwrap(),
        ),
        CurveFamily2::RationalBezier => Curve2::from(
            RationalBezier2::try_new(vec![start, middle, end], vec![r(1), r(1), r(1)]).unwrap(),
        ),
        CurveFamily2::PolynomialBSpline => Curve2::try_polynomial_bspline(
            1,
            vec![start, end],
            vec![r(0), r(0), r(1), r(1)],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value(),
        CurveFamily2::Nurbs => Curve2::try_nurbs(
            1,
            vec![start, end],
            vec![r(1), r(1)],
            vec![r(0), r(0), r(1), r(1)],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value(),
        CurveFamily2::CircularArc => panic!("linear test carrier excludes circular arcs"),
    }
}

fn every_family_open_chain() -> Vec<Curve2> {
    vec![
        Curve2::from(LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap()),
        Curve2::from(CircularArc2::from_bulge(p(1, 0), p(3, 0), r(1)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(3, 0), p(4, 1), p(5, 0))),
        Curve2::from(CubicBezier2::new(p(5, 0), p(6, 1), p(7, 1), p(8, 0))),
        Curve2::from(
            RationalQuadraticBezier2::try_new(p(8, 0), p(9, 1), p(10, 0), r(1), r(2), r(1))
                .unwrap(),
        ),
        Curve2::from(
            RationalBezier2::try_new(vec![p(10, 0), p(11, 1), p(12, 0)], vec![r(1), r(2), r(1)])
                .unwrap(),
        ),
        Curve2::try_polynomial_bspline(
            2,
            vec![p(12, 0), p(13, 2), p(14, 0)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value(),
        Curve2::try_nurbs(
            2,
            vec![p(14, 0), p(15, 2), p(16, 0)],
            vec![r(1), r(2), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value(),
    ]
}

fn every_family_closed_path() -> CurvePath2 {
    let mut curves = every_family_open_chain();
    curves.extend([
        Curve2::from(LineSeg2::try_new(p(16, 0), p(16, -3)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(16, -3), p(0, -3)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, -3), p(0, 0)).unwrap()),
    ]);
    CurvePath2::try_new(curves).unwrap()
}

fn right_angle_line_path(length: i32) -> CurvePath2 {
    CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-length, 0), p(0, 0)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, 0), p(0, length)).unwrap()),
    ])
    .unwrap()
}

#[test]
fn top_level_curve_carries_every_public_family() {
    let curves = every_family_open_chain();

    assert_eq!(
        curves.iter().map(Curve2::family).collect::<Vec<_>>(),
        vec![
            CurveFamily2::Line,
            CurveFamily2::CircularArc,
            CurveFamily2::QuadraticBezier,
            CurveFamily2::CubicBezier,
            CurveFamily2::RationalQuadraticBezier,
            CurveFamily2::RationalBezier,
            CurveFamily2::PolynomialBSpline,
            CurveFamily2::Nurbs,
        ]
    );
}
#[test]
fn top_level_curve_region_classifies_points_and_shares_results() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_boundary_paths(&[every_family_closed_path()], &policy)
        .unwrap()
        .into_value();
    let clone = region.clone();
    let signed_area = match region.signed_area(&policy).unwrap().into_value() {
        Classification::Decided(Some(area)) => area,
        other => panic!("degree-two rational boundary contributions are exact: {other:?}"),
    };
    assert_eq!(
        clone
            .signed_area(&policy)
            .map(|outcome| outcome.into_value()),
        Ok(Classification::Decided(Some(signed_area)))
    );
    assert_eq!(
        region
            .classify_point(&p(8, -1), &CurveContext::STRICT)
            .map(|outcome| outcome.into_value()),
        Ok(Classification::Decided(RegionPointLocation::Inside))
    );
    assert_eq!(
        clone
            .classify_point(&p(8, -4), &CurveContext::STRICT)
            .map(|outcome| outcome.into_value()),
        Ok(Classification::Decided(RegionPointLocation::Outside))
    );
    assert_eq!(
        clone
            .classify_point(&p(0, 0), &CurveContext::STRICT)
            .map(|outcome| outcome.into_value()),
        Ok(Classification::Decided(RegionPointLocation::Boundary))
    );
    let debug = format!("{region:?}");
    assert!(!debug.contains("native_boundary"));
    assert!(!debug.contains("signed_area_cache"));

    let square = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, 0), p(2, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, 2), p(0, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, 2), p(0, 0)).unwrap()),
    ])
    .unwrap();
    let bounded = CurveRegion2::try_from_boundary_paths(&[square], &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let bounded_clone = bounded.clone();
    assert_eq!(
        bounded
            .classify_point(&p(1, 1), &CurveContext::STRICT)
            .map(|outcome| outcome.into_value()),
        Ok(Classification::Decided(RegionPointLocation::Inside))
    );
    assert_eq!(
        bounded_clone
            .classify_point(&p(1, 1), &CurveContext::STRICT)
            .map(|outcome| outcome.into_value()),
        Ok(Classification::Decided(RegionPointLocation::Inside))
    );
}

#[test]
fn curve_path_boundary_and_classification_report_terminal_closure() {
    let start = Point2::new(Real::pi() + Real::e(), Real::zero());
    let end = Point2::new(Real::e() + Real::pi(), Real::zero());
    let origin = p(0, 0);
    let upper = p(0, 2);
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(start.clone(), origin.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(origin, upper.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(upper, end).unwrap()),
    ])
    .unwrap();

    let boundary = path
        .bezier_boundary_loop(&CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must validate the symbolic closing seam");
    assert_eq!(boundary.certainty, CurveCertainty::Approximate512Consumed);
    assert_eq!(boundary.value.len(), 3);

    let strict_boundary = path
        .bezier_boundary_loop(&CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        strict_boundary,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::Arrangement
                && blocker.reason() == UncertaintyReason::RealSign
    ));

    let approximate = path
        .classify_point(&p(1, 1), &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must classify through the retained boundary");
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        approximate.value,
        Classification::Decided(ContourPointLocation::Inside)
    );

    let strict = path
        .classify_point(&p(1, 1), &CurveContext::STRICT)
        .expect("strict classification returns explicit uncertainty");
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert_eq!(
        strict.value,
        Classification::Uncertain(UncertaintyReason::RealSign)
    );

    let repeated = path
        .classify_point(&p(1, 1), &CurveContext::APPROXIMATE_512)
        .expect("cached terminal evidence must remain observable");
    assert_eq!(repeated.certainty, CurveCertainty::Approximate512Consumed);
    assert_eq!(repeated.value, approximate.value);
}

#[test]
fn top_level_curve_region_rejects_open_boundary_paths_with_context() {
    let path = CurvePath2::try_new(vec![Curve2::from(
        LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap(),
    )])
    .unwrap();

    let error = CurveRegion2::try_from_boundary_paths(&[path], &CurveContext::STRICT).unwrap_err();

    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            operation: CurveOperation2::Construction,
            family: CurveFamily2::Line,
            cause: CurveError::OpenCurvePath,
            ..
        }
    ));
}
#[test]
fn top_level_curve_evaluates_native_and_spline_parameters() {
    let half = (r(1) / r(2)).unwrap();
    let line = Curve2::from(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap());
    let quadratic = Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0)));
    let spline = Curve2::try_polynomial_bspline(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert_eq!(
        line.point_at(&half, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(1, 0)
    );
    assert_eq!(
        (
            line.parameter_domain().start(),
            line.parameter_domain().end()
        ),
        (&r(0), &r(1))
    );
    assert_eq!(
        quadratic
            .as_view()
            .point_at(&half, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(1, 1)
    );
    assert_eq!(
        spline
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(1, 1)
    );
    assert_eq!(
        (
            spline.parameter_domain().start(),
            spline.parameter_domain().end()
        ),
        (&r(0), &r(2))
    );
    assert!(std::ptr::eq(
        spline.parameter_domain(),
        spline.clone().as_view().parameter_domain()
    ));
}

#[test]
fn top_level_curve_reuses_retained_native_endpoints() {
    for curve in every_family_open_chain() {
        assert_eq!(
            curve
                .point_at(curve.parameter_domain().start(), &CurveContext::STRICT)
                .unwrap()
                .into_value(),
            curve.start().clone()
        );
        assert_eq!(
            curve
                .point_at(curve.parameter_domain().end(), &CurveContext::STRICT)
                .unwrap()
                .into_value(),
            curve.end().clone()
        );
    }

    let rational =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 2), p(2, 0)], vec![r(1), r(2), r(3)]).unwrap();
    let top_level = Curve2::from(rational.clone());
    assert_eq!(
        top_level
            .point_at(&r(0), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(0, 0)
    );
    assert_eq!(
        top_level
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(2, 0)
    );
    assert_eq!(
        top_level
            .point_at(&(r(1) / r(2)).unwrap(), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        rational
            .point_at(&(r(1) / r(2)).unwrap(), &CurveContext::STRICT)
            .unwrap()
    );
}

#[test]
fn top_level_curve_derivatives_preserve_parameter_domains_and_share_evaluators() {
    let half = (r(1) / r(2)).unwrap();
    let line = Curve2::from(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap());
    let line_clone = line.clone();
    let line_derivative = line
        .as_view()
        .derivative_at(&half, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(line_derivative.dx(), &r(2));
    assert_eq!(line_derivative.dy(), &r(0));
    assert_eq!(
        line_clone
            .derivative_at(&half, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        line_derivative
    );

    let quadratic = Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0)));
    let quadratic_derivative = quadratic
        .derivative_at(&half, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(quadratic_derivative.dx(), &r(2));
    assert_eq!(quadratic_derivative.dy(), &r(0));

    let spline = Curve2::try_polynomial_bspline(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let CurveGeometry2::PolynomialBSpline(retained_spline) = spline.geometry() else {
        panic!("top-level polynomial constructor returned another family");
    };
    let spline_derivative = spline
        .derivative_at(&r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(spline_derivative.dx(), &r(1));
    assert_eq!(spline_derivative.dy(), &r(0));
    assert_eq!(
        retained_spline
            .derivative_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        spline_derivative
    );
    assert_eq!(
        spline
            .derivative_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        spline_derivative
    );
}

#[test]
fn top_level_curve_and_view_expose_exact_higher_derivatives() {
    let curve = Curve2::from(
        hypercurve::RationalBezier2::try_new(vec![p(0, 0), p(4, 0)], vec![r(1), r(3)]).unwrap(),
    );
    let half = (r(1) / r(2)).unwrap();

    let derivatives = curve
        .as_view()
        .derivatives_at(&half, 3, &CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert_eq!(derivatives.len(), 3);
    assert_eq!(derivatives[0].dx(), &r(3));
    assert_eq!(derivatives[1].dx(), &r(-6));
    assert_eq!(derivatives[2].dx(), &r(18));
}
#[test]
fn mixed_curve_path_chamfer_trims_every_public_family_exactly() {
    let path = CurvePath2::try_new(every_family_open_chain()).unwrap();

    for vertex_index in 1..path.curves().len() {
        let chamfered = path
            .chamfer_vertex_by_parameters(vertex_index, q(1, 2), q(1, 2), &CurveContext::STRICT)
            .unwrap()
            .into_value();
        assert_eq!(chamfered.curves().len(), path.curves().len() + 1);
        assert_eq!(
            chamfered.curves()[vertex_index - 1].family(),
            path.curves()[vertex_index - 1].family()
        );
        assert_eq!(
            chamfered.curves()[vertex_index].family(),
            CurveFamily2::Line
        );
        assert_eq!(
            chamfered.curves()[vertex_index + 1].family(),
            path.curves()[vertex_index].family()
        );
        assert_eq!(
            chamfered.curves()[vertex_index - 1].end(),
            chamfered.curves()[vertex_index].start()
        );
        assert_eq!(
            chamfered.curves()[vertex_index].end(),
            chamfered.curves()[vertex_index + 1].start()
        );
    }
}

#[test]
fn mixed_curve_path_fillet_accepts_every_non_arc_family_pair() {
    let families = [
        CurveFamily2::Line,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
    ];

    for previous_family in families {
        for next_family in families {
            let path = CurvePath2::try_new(vec![
                linear_family_curve(previous_family, false),
                linear_family_curve(next_family, true),
            ])
            .unwrap();
            let filleted = path
                .fillet_vertex_by_parameters(
                    1,
                    q(1, 2),
                    q(1, 2),
                    &p(-1, 1),
                    false,
                    &CurveContext::STRICT,
                )
                .unwrap_or_else(|error| {
                    panic!("{previous_family:?}/{next_family:?} fillet failed: {error}")
                })
                .into_value();

            assert_eq!(filleted.curves().len(), 3);
            assert_eq!(filleted.curves()[0].family(), previous_family);
            assert_eq!(filleted.curves()[1].family(), CurveFamily2::CircularArc);
            assert_eq!(filleted.curves()[2].family(), next_family);
            assert_eq!(filleted.curves()[0].end(), &p(-1, 0));
            assert_eq!(filleted.curves()[1].start(), &p(-1, 0));
            assert_eq!(filleted.curves()[1].end(), &p(0, 1));
            assert_eq!(filleted.curves()[2].start(), &p(0, 1));
        }
    }
}

#[test]
fn higher_order_curve_path_fillet_obeys_terminal_policy_once() {
    let path = CurvePath2::try_new(vec![
        linear_family_curve(CurveFamily2::QuadraticBezier, false),
        linear_family_curve(CurveFamily2::QuadraticBezier, true),
    ])
    .unwrap();
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let center = Point2::new(Real::from(-1) + undecidable_zero, Real::one());

    let strict = path
        .fillet_vertex_by_parameters(1, q(1, 2), q(1, 2), &center, false, &CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        strict,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::Fillet
                && blocker.reason() == hypercurve::UncertaintyReason::RealSign
    ));

    let approximate = path
        .fillet_vertex_by_parameters(
            1,
            q(1, 2),
            q(1, 2),
            &center,
            false,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    let CurveGeometry2::CircularArc(fillet) = approximate.value.curves()[1].geometry() else {
        panic!("the exact higher-order fillet must insert a circular arc");
    };
    assert_eq!(fillet.center(), &center);
    for family in [
        CurveFamily2::Line,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
    ] {
        let family_path = CurvePath2::try_new(vec![
            linear_family_curve(family, false),
            linear_family_curve(family, true),
        ])
        .unwrap();
        let family_fillet = family_path
            .fillet_vertex_by_parameters(
                1,
                q(1, 2),
                q(1, 2),
                &center,
                false,
                &CurveContext::APPROXIMATE_512,
            )
            .unwrap_or_else(|error| panic!("{family:?} terminal fillet failed: {error}"));
        assert_eq!(
            family_fillet.certainty,
            CurveCertainty::Approximate512Consumed,
            "{family:?} did not report its terminal radius/tangency decision"
        );
    }

    let sweep_center = Point2::new(
        Real::from(3) + ((Real::pi() + Real::e()) - (Real::e() + Real::pi())),
        Real::one(),
    );
    let sweep_source = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(4, 0), p(3, 4), p(2, 0))),
    ])
    .unwrap();
    let sweep_fillet = sweep_source
        .fillet_vertex_by_parameters(
            1,
            q(3, 4),
            q(1, 2),
            &sweep_center,
            false,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap()
        .into_value();
    let CurveGeometry2::CircularArc(sweep_arc) = sweep_fillet.curves()[1].geometry() else {
        panic!("the sweep-sensitive fillet must insert a circular arc");
    };
    assert!(matches!(
        sweep_arc
            .point_at_sweep_fraction(&q(1, 2), &CurveContext::APPROXIMATE_512)
            .unwrap(),
        Classification::Decided(_)
    ));
    assert_eq!(
        sweep_arc
            .point_at_sweep_fraction(&q(1, 2), &CurveContext::STRICT)
            .unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::RealSign),
        "an approximate-first angular cache must retain the strict blocker"
    );
    let mut closed_curves = sweep_fillet.curves().to_vec();
    closed_curves.extend([
        Curve2::from(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, -2), p(0, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, -2), p(0, 0)).unwrap()),
    ]);
    let closed = CurvePath2::try_new(closed_curves).unwrap();
    let promoted = CurveRegion2::try_from_boundary_paths(
        std::slice::from_ref(&closed),
        &CurveContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(promoted.certainty, CurveCertainty::Approximate512Consumed);
    assert!(matches!(
        CurveRegion2::try_from_boundary_paths(&[closed], &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::RealSign
    ));
}

#[test]
fn mixed_curve_path_fillet_preserves_arc_family_and_exact_tangency() {
    let source_arc = CircularArc2::try_from_center(p(5, 0), p(5, 2), p(5, 1), true).unwrap();
    let Classification::Decided(next_parameter) = source_arc
        .sweep_fraction(&p(4, 1), &CurveContext::STRICT)
        .unwrap()
    else {
        panic!("arc tangent point should have an exact source parameter");
    };
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(0, 0), p(5, 0)).unwrap()),
        Curve2::from(source_arc),
    ])
    .unwrap();

    let filleted = path
        .fillet_vertex_by_parameters(
            1,
            q(3, 5),
            next_parameter,
            &p(3, 1),
            false,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();

    assert_eq!(
        filleted
            .curves()
            .iter()
            .map(Curve2::family)
            .collect::<Vec<_>>(),
        vec![
            CurveFamily2::Line,
            CurveFamily2::CircularArc,
            CurveFamily2::CircularArc,
        ]
    );
    assert_eq!(filleted.curves()[0].end(), &p(3, 0));
    assert_eq!(filleted.curves()[1].end(), &p(4, 1));
    assert_eq!(filleted.curves()[2].start(), &p(4, 1));

    let previous_arc = CircularArc2::try_from_center(p(5, 2), p(5, 0), p(5, 1), false).unwrap();
    let Classification::Decided(previous_parameter) = previous_arc
        .sweep_fraction(&p(4, 1), &CurveContext::STRICT)
        .unwrap()
    else {
        panic!("previous arc tangent point should have an exact source parameter");
    };
    let reversed_pair = CurvePath2::try_new(vec![
        Curve2::from(previous_arc),
        Curve2::from(LineSeg2::try_new(p(5, 0), p(0, 0)).unwrap()),
    ])
    .unwrap();

    let reversed_fillet = reversed_pair
        .fillet_vertex_by_parameters(
            1,
            previous_parameter,
            q(2, 5),
            &p(3, 1),
            true,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    assert_eq!(
        reversed_fillet.curves()[0].family(),
        CurveFamily2::CircularArc
    );
    assert_eq!(reversed_fillet.curves()[0].end(), &p(4, 1));
    assert_eq!(reversed_fillet.curves()[1].end(), &p(3, 0));
    assert_eq!(reversed_fillet.curves()[2].family(), CurveFamily2::Line);
}

#[test]
fn closed_curve_path_corner_edits_support_the_start_end_seam() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, 0), p(2, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, 2), p(0, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, 2), p(0, 0)).unwrap()),
    ])
    .unwrap();

    let chamfered = path
        .as_view()
        .chamfer_vertex_by_parameters(0, q(1, 2), q(1, 2), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(chamfered.start(), &p(0, 1));
    assert_eq!(chamfered.end(), chamfered.start());
    assert_eq!(chamfered.curves()[0].end(), &p(1, 0));

    let filleted = path
        .fillet_vertex_by_parameters(0, q(1, 2), q(1, 2), &p(1, 1), false, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(filleted.start(), &p(0, 1));
    assert_eq!(filleted.end(), filleted.start());
    assert_eq!(filleted.curves()[0].family(), CurveFamily2::CircularArc);
    assert_eq!(filleted.curves()[0].end(), &p(1, 0));
    CurveRegion2::try_from_boundary_paths(&[filleted], &CurveContext::STRICT)
        .unwrap()
        .into_value();

    let CurveCornerSolutions2::Unique(solved_chamfer) = path
        .chamfer_vertex_by_setbacks(
            0,
            r(1),
            r(1),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    else {
        panic!("the closed seam must have one setback chamfer");
    };
    assert_eq!(solved_chamfer.start(), &p(0, 1));
    assert_eq!(solved_chamfer.end(), solved_chamfer.start());

    let CurveCornerSolutions2::Unique(solved_fillet) = path
        .fillet_vertex_by_radius(0, r(1), CurveCornerMode2::TrimOnly, &CurveContext::STRICT)
        .unwrap()
        .into_value()
    else {
        panic!("the closed seam must have one radius fillet");
    };
    assert_eq!(solved_fillet.start(), &p(0, 1));
    assert_eq!(solved_fillet.end(), solved_fillet.start());
    let CurveGeometry2::CircularArc(arc) = solved_fillet.curves()[0].geometry() else {
        panic!("the seam fillet must lead with its circular carrier");
    };
    assert_eq!(arc.center(), &p(1, 1));
    assert_eq!(arc.end(), &p(1, 0));
}
#[test]
fn mixed_curve_path_corner_edits_reject_invalid_parameters_and_tangency() {
    let path = CurvePath2::try_new(vec![
        linear_family_curve(CurveFamily2::Line, false),
        linear_family_curve(CurveFamily2::Line, true),
    ])
    .unwrap();

    let boundary = path
        .chamfer_vertex_by_parameters(1, r(1), q(1, 2), &CurveContext::STRICT)
        .unwrap_err();
    assert_eq!(boundary.operation(), CurveOperation2::Chamfer);
    assert!(matches!(
        boundary,
        ExactCurveError::Invalid {
            cause: CurveError::InvalidCurveParameter,
            ..
        }
    ));

    let nontangent = path
        .fillet_vertex_by_parameters(1, q(1, 2), q(1, 2), &p(0, 0), false, &CurveContext::STRICT)
        .unwrap_err();
    assert_eq!(nontangent.operation(), CurveOperation2::Fillet);
    assert!(matches!(
        nontangent,
        ExactCurveError::Invalid {
            cause: CurveError::RadiusMismatch | CurveError::InvalidFilletTangency,
            ..
        }
    ));

    let open_seam = path
        .chamfer_vertex_by_parameters(0, q(1, 2), q(1, 2), &CurveContext::STRICT)
        .unwrap_err();
    assert_eq!(open_seam.operation(), CurveOperation2::Chamfer);
    assert!(matches!(
        open_seam,
        ExactCurveError::Invalid {
            cause: CurveError::OpenCurvePath,
            ..
        }
    ));
}

#[test]
fn line_corner_solvers_derive_unique_trimmed_fillet_and_chamfer() {
    let path = right_angle_line_path(4);
    let chamfer = path
        .chamfer_vertex_by_setbacks(
            1,
            r(1),
            r(1),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        )
        .unwrap();
    let CurveCornerSolutions2::Unique(chamfer) = chamfer.into_value() else {
        panic!("equal line setbacks must have one trimmed solution");
    };
    assert_eq!(chamfer.curves().len(), 3);
    assert_eq!(chamfer.curves()[0].end(), &p(-1, 0));
    assert_eq!(chamfer.curves()[1].start(), &p(-1, 0));
    assert_eq!(chamfer.curves()[1].end(), &p(0, 1));
    assert_eq!(chamfer.curves()[2].start(), &p(0, 1));

    let fillet = path
        .fillet_vertex_by_radius(1, r(1), CurveCornerMode2::TrimOnly, &CurveContext::STRICT)
        .unwrap();
    let CurveCornerSolutions2::Unique(fillet) = fillet.into_value() else {
        panic!("a convex right-angle line corner must have one trimmed fillet");
    };
    assert_eq!(fillet.curves().len(), 3);
    assert_eq!(fillet.curves()[0].end(), &p(-1, 0));
    let CurveGeometry2::CircularArc(arc) = fillet.curves()[1].geometry() else {
        panic!("the solved fillet must be an exact circular carrier");
    };
    assert_eq!(arc.start(), &p(-1, 0));
    assert_eq!(arc.end(), &p(0, 1));
    assert_eq!(arc.center(), &p(-1, 1));
    assert_eq!(arc.radius_squared(), r(1));
    assert!(!arc.is_clockwise());
}

#[test]
fn oblique_line_corner_solvers_preserve_exact_orientation() {
    for (next_y, next_cut_y, center_y, clockwise) in
        [(4, q(4, 5), r(1), false), (-4, q(-4, 5), r(-1), true)]
    {
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-5, 0), p(0, 0)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(0, 0), p(3, next_y)).unwrap()),
        ])
        .unwrap();
        let CurveCornerSolutions2::Unique(chamfer) = path
            .chamfer_vertex_by_setbacks(
                1,
                Real::one(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value()
        else {
            panic!("a 3-4-5 line corner must have one exact chamfer");
        };
        assert_eq!(chamfer.curves()[0].end(), &p(-1, 0));
        assert_eq!(
            chamfer.curves()[2].start(),
            &Point2::new(q(3, 5), next_cut_y)
        );

        let CurveCornerSolutions2::Unique(fillet) = path
            .fillet_vertex_by_radius(
                1,
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value()
        else {
            panic!("a 3-4-5 line corner must have one exact fillet");
        };
        let CurveGeometry2::CircularArc(arc) = fillet.curves()[1].geometry() else {
            panic!("the oblique fillet must remain circular");
        };
        assert_eq!(arc.start(), &Point2::new(q(-1, 2), Real::zero()));
        assert_eq!(arc.end(), &Point2::new(q(3, 10), q(next_y, 10)));
        assert_eq!(arc.center(), &Point2::new(q(-1, 2), center_y));
        assert_eq!(arc.radius_squared(), Real::one());
        assert_eq!(arc.is_clockwise(), clockwise);
    }
}

#[test]
fn exact_chamfer_solver_handles_native_line_arc_and_arc_arc_carriers() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let next_arc = CircularArc2::try_from_center(p(1, 0), p(2, 1), p(1, 1), false).unwrap();
        let line_arc = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-1, 0), p(1, 0)).unwrap()),
            Curve2::from(next_arc.clone()),
        ])
        .unwrap();
        let CurveCornerSolutions2::Unique(chamfered) = line_arc
            .chamfer_vertex_by_setbacks(
                1,
                q(1, 2),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap()
            .into_value()
        else {
            panic!("the line-arc carrier pair must have one exact trim solution");
        };
        assert_eq!(
            chamfered
                .curves()
                .iter()
                .map(Curve2::family)
                .collect::<Vec<_>>(),
            vec![
                CurveFamily2::Line,
                CurveFamily2::Line,
                CurveFamily2::CircularArc,
            ]
        );
        let line_cut = Point2::new(q(1, 2), Real::zero());
        let next_cut = chamfered.curves()[2].start();
        assert_eq!(chamfered.curves()[0].end(), &line_cut);
        assert_eq!(chamfered.curves()[1].start(), &line_cut);
        assert_eq!(chamfered.curves()[1].end(), next_cut);
        assert!(matches!(
            next_cut
                .distance_squared(&p(1, 0))
                .certified_eq_until(&Real::one(), -4096),
            CertifiedRealEquality::Equal { .. }
        ));

        let CurveCornerSolutions2::Multiple(extended) = line_arc
            .chamfer_vertex_by_setbacks(
                1,
                q(1, 2),
                Real::one(),
                CurveCornerMode2::TrimOrExtend,
                &policy,
            )
            .unwrap()
            .into_value()
        else {
            panic!("line-arc extension mode must retain all four support choices");
        };
        assert_eq!(extended.len(), 4);
        assert_eq!(
            extended[0].curves()[0].end(),
            &Point2::new(q(1, 2), Real::zero())
        );
        assert_eq!(extended[0].curves()[0].end(), extended[1].curves()[0].end());
        assert_eq!(
            extended[2].curves()[0].end(),
            &Point2::new(q(3, 2), Real::zero())
        );
        assert_eq!(extended[2].curves()[0].end(), extended[3].curves()[0].end());
        assert_eq!(
            extended[0].curves()[2].start(),
            extended[2].curves()[2].start()
        );
        assert_eq!(
            extended[1].curves()[2].start(),
            extended[3].curves()[2].start()
        );
        assert_ne!(
            extended[0].curves()[2].start(),
            extended[1].curves()[2].start()
        );
        let mut retained_corner_membership = extended[..2]
            .iter()
            .map(|candidate| {
                let CurveGeometry2::CircularArc(arc) = candidate.curves()[2].geometry() else {
                    panic!("every line-arc candidate must retain a circular carrier");
                };
                assert_eq!(arc.center(), &p(1, 1));
                assert_eq!(arc.end(), &p(2, 1));
                arc.contains_point(&p(1, 0), &policy)
            })
            .collect::<Vec<_>>();
        retained_corner_membership
            .sort_by_key(|classification| matches!(classification, Classification::Decided(true)));
        assert_eq!(
            retained_corner_membership,
            vec![
                Classification::Decided(false),
                Classification::Decided(true)
            ]
        );
        assert_eq!(
            line_arc
                .chamfer_vertex_by_setbacks(
                    1,
                    q(1, 2),
                    r(2).sqrt().unwrap(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap()
                .into_value(),
            CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::OutsideTrimDomain),
            "a cut at the opposite arc endpoint is not an interior trim"
        );

        let previous_arc =
            CircularArc2::try_from_center(p(0, -1), p(1, 0), p(1, -1), true).unwrap();
        let arc_arc =
            CurvePath2::try_new(vec![Curve2::from(previous_arc), Curve2::from(next_arc)]).unwrap();
        let CurveCornerSolutions2::Unique(chamfered) = arc_arc
            .chamfer_vertex_by_setbacks(
                1,
                Real::one(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap()
            .into_value()
        else {
            panic!("the arc-arc carrier pair must have one exact trim solution");
        };
        assert_eq!(
            chamfered
                .curves()
                .iter()
                .map(Curve2::family)
                .collect::<Vec<_>>(),
            vec![
                CurveFamily2::CircularArc,
                CurveFamily2::Line,
                CurveFamily2::CircularArc,
            ]
        );
        let previous_cut = chamfered.curves()[0].end();
        let next_cut = chamfered.curves()[2].start();
        assert_eq!(chamfered.curves()[1].start(), previous_cut);
        assert_eq!(chamfered.curves()[1].end(), next_cut);
        for cut in [previous_cut, next_cut] {
            assert!(matches!(
                cut.distance_squared(&p(1, 0))
                    .certified_eq_until(&Real::one(), -4096),
                CertifiedRealEquality::Equal { .. }
            ));
        }
    }
}

#[test]
fn exact_arc_chamfer_solver_preserves_both_major_sweep_cuts() {
    let major_arc =
        CircularArc2::try_from_center(p(1, 0), Point2::new(q(3, 5), q(4, 5)), p(0, 0), true)
            .unwrap();
    let half_root_three = (r(3).sqrt().unwrap() / r(2)).unwrap();
    for cut in [
        Point2::new(q(1, 2), -half_root_three.clone()),
        Point2::new(q(1, 2), half_root_three),
    ] {
        let contains = major_arc.contains_point(&cut, &CurveContext::STRICT);
        assert!(
            matches!(contains, Classification::Decided(true)),
            "major-arc cut incidence must be strict: {contains:?}"
        );
        let sweep = major_arc
            .sweep_fraction(&cut, &CurveContext::STRICT)
            .unwrap();
        let Classification::Decided(sweep) = sweep else {
            panic!("major-arc cut sweep fraction must be strict: {sweep:?}");
        };
        let parameter = major_arc
            .parameter_at_sweep_fraction(&sweep, &CurveContext::STRICT)
            .unwrap();
        assert!(
            matches!(parameter, Classification::Decided(_)),
            "major-arc public parameter must be strict: {parameter:?}"
        );
    }
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(1, -2), p(1, 0)).unwrap()),
        Curve2::from(major_arc),
    ])
    .unwrap();

    let CurveCornerSolutions2::Unique(tangent) = path
        .chamfer_vertex_by_setbacks(
            1,
            Real::one(),
            r(2),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    else {
        panic!("the diametric major-arc setback must retain its tangent cut");
    };
    assert_eq!(tangent.curves()[0].end(), &p(1, -1));
    assert_eq!(tangent.curves()[1].end(), &p(-1, 0));
    assert_eq!(tangent.curves()[2].start(), &p(-1, 0));
    assert_eq!(
        path.chamfer_vertex_by_setbacks(
            1,
            Real::one(),
            r(3),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value(),
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::OutsideTrimDomain)
    );

    let CurveCornerSolutions2::Multiple(chamfers) = path
        .chamfer_vertex_by_setbacks(
            1,
            Real::one(),
            Real::one(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    else {
        panic!("a major arc crossing the setback circle twice must retain both cuts");
    };
    assert_eq!(chamfers.len(), 2);
    let next_starts = chamfers
        .iter()
        .map(|chamfer| {
            assert_eq!(chamfer.curves()[0].end(), &p(1, -1));
            assert_eq!(chamfer.curves()[1].start(), &p(1, -1));
            assert_eq!(chamfer.curves()[2].family(), CurveFamily2::CircularArc);
            chamfer.curves()[2].start().clone()
        })
        .collect::<Vec<_>>();
    assert_ne!(next_starts[0], next_starts[1]);
    for cut in next_starts {
        assert!(matches!(
            cut.distance_squared(&p(1, 0))
                .certified_eq_until(&Real::one(), -4096),
            CertifiedRealEquality::Equal { .. }
        ));
    }
}

#[test]
fn exact_native_arc_fillet_solver_handles_every_native_pair_order() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let line_arc = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-2, 0), p(0, 0)).unwrap()),
            Curve2::from(CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap()),
        ])
        .unwrap();
        let CurveCornerSolutions2::Unique(fillet) = line_arc
            .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)
            .unwrap()
            .into_value()
        else {
            panic!("the line/arc corner must have one exact radius fillet");
        };
        assert_eq!(
            fillet
                .curves()
                .iter()
                .map(Curve2::family)
                .collect::<Vec<_>>(),
            vec![
                CurveFamily2::Line,
                CurveFamily2::CircularArc,
                CurveFamily2::CircularArc,
            ]
        );
        let CurveGeometry2::CircularArc(inserted) = fillet.curves()[1].geometry() else {
            panic!("the solved fillet must remain a certified circular arc");
        };
        let expected_center = Point2::new(Real::one() - r(2).sqrt().unwrap(), q(1, 2));
        assert!(matches!(
            inserted
                .center()
                .distance_squared(&expected_center)
                .certified_eq_until(&Real::zero(), -4096),
            CertifiedRealEquality::Equal { .. }
        ));
        assert!(matches!(
            inserted
                .radius_squared()
                .certified_eq_until(&q(1, 4), -4096),
            CertifiedRealEquality::Equal { .. }
        ));
        assert_eq!(fillet.curves()[0].end(), inserted.start());
        assert_eq!(fillet.curves()[2].start(), inserted.end());
        let CurveGeometry2::CircularArc(retained_next) = fillet.curves()[2].geometry() else {
            panic!("the source circular carrier must be retained");
        };
        assert_eq!(retained_next.center(), &p(1, 0));
        assert_eq!(retained_next.end(), &p(1, 1));

        let extended = line_arc
            .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOrExtend, &policy)
            .unwrap()
            .into_value();
        assert_eq!(extended.candidate_count(), 3);

        let reversed = line_arc.clone().reversed(&policy).unwrap().into_value();
        let CurveCornerSolutions2::Unique(reversed_fillet) = reversed
            .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)
            .unwrap()
            .into_value()
        else {
            panic!("the reversed arc/line corner must have one exact radius fillet");
        };
        assert_eq!(
            reversed_fillet
                .curves()
                .iter()
                .map(Curve2::family)
                .collect::<Vec<_>>(),
            vec![
                CurveFamily2::CircularArc,
                CurveFamily2::CircularArc,
                CurveFamily2::Line,
            ]
        );

        let arc_arc = CurvePath2::try_new(vec![
            Curve2::from(
                CircularArc2::try_from_center(p(-1, -1), p(0, 0), p(0, -1), true).unwrap(),
            ),
            Curve2::from(CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap()),
        ])
        .unwrap();
        let CurveCornerSolutions2::Unique(fillet) = arc_arc
            .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)
            .unwrap()
            .into_value()
        else {
            panic!("the arc/arc corner must have one exact radius fillet");
        };
        assert!(
            fillet
                .curves()
                .iter()
                .all(|curve| curve.family() == CurveFamily2::CircularArc)
        );
        let CurveGeometry2::CircularArc(inserted) = fillet.curves()[1].geometry() else {
            unreachable!();
        };
        for source_center in [p(0, -1), p(1, 0)] {
            assert!(matches!(
                inserted
                    .center()
                    .distance_squared(&source_center)
                    .certified_eq_until(&q(9, 4), -4096),
                CertifiedRealEquality::Equal { .. }
            ));
        }
        assert!(matches!(
            inserted
                .radius_squared()
                .certified_eq_until(&q(1, 4), -4096),
            CertifiedRealEquality::Equal { .. }
        ));
        assert_eq!(fillet.curves()[0].end(), inserted.start());
        assert_eq!(fillet.curves()[2].start(), inserted.end());

        let extended = arc_arc
            .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOrExtend, &policy)
            .unwrap()
            .into_value();
        assert_eq!(extended.candidate_count(), 2);

        let cross_center = arc_arc
            .fillet_vertex_by_radius(1, r(2), CurveCornerMode2::TrimOrExtend, &policy)
            .unwrap()
            .into_value();
        let CurveCornerSolutions2::Multiple(cross_center) = cross_center else {
            panic!("a radius larger than both source radii must retain all exact branches");
        };
        assert!(cross_center.iter().any(|candidate| {
            let CurveGeometry2::CircularArc(fillet) = candidate.curves()[1].geometry() else {
                return false;
            };
            fillet.is_clockwise()
                && fillet
                    .radius_squared()
                    .certified_eq_until(&r(4), -4096)
                    .as_bool()
                    == Some(true)
        }));
    }
}

#[test]
fn retained_circular_conics_share_the_native_corner_kernel() {
    let native_arc = CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap();
    let conic = native_arc
        .rational_bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .spans()[0]
        .curve()
        .clone();
    let elevated = RationalBezier2::from(conic.clone())
        .elevated_to_degree(5)
        .unwrap();
    let carriers = [
        (CurveFamily2::RationalQuadraticBezier, Curve2::from(conic)),
        (CurveFamily2::RationalBezier, Curve2::from(elevated)),
    ];

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (family, carrier) in &carriers {
            let path = CurvePath2::try_new(vec![
                Curve2::from(LineSeg2::try_new(p(-2, 0), p(0, 0)).unwrap()),
                carrier.clone(),
            ])
            .unwrap();

            let CurveCornerSolutions2::Unique(chamfer) = path
                .chamfer_vertex_by_setbacks(
                    1,
                    q(1, 2),
                    q(1, 2),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap()
                .into_value()
            else {
                panic!("the retained circular conic must have one exact chamfer");
            };
            assert_eq!(chamfer.curves()[2].family(), *family);
            assert_eq!(chamfer.curves()[0].end(), chamfer.curves()[1].start());
            assert_eq!(chamfer.curves()[1].end(), chamfer.curves()[2].start());

            let CurveCornerSolutions2::Unique(fillet) = path
                .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)
                .unwrap()
                .into_value()
            else {
                panic!("the retained circular conic must have one exact fillet");
            };
            assert_eq!(fillet.curves()[2].family(), *family);
            assert_eq!(fillet.curves()[0].end(), fillet.curves()[1].start());
            assert_eq!(fillet.curves()[1].end(), fillet.curves()[2].start());
            let CurveGeometry2::CircularArc(inserted) = fillet.curves()[1].geometry() else {
                panic!("the inserted fillet must remain circular");
            };
            assert!(matches!(
                inserted
                    .radius_squared()
                    .certified_eq_until(&q(1, 4), -4096),
                CertifiedRealEquality::Equal { .. }
            ));

            let blocked = path
                .chamfer_vertex_by_setbacks(
                    1,
                    q(1, 2),
                    q(1, 2),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .map(|_| ());
            assert!(matches!(
                blocked,
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.operation() == CurveOperation2::Chamfer
                        && blocker.family() == *family
                        && blocker.reason() == UncertaintyReason::Unsupported
            ));

            let extended = path
                .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOrExtend, &policy)
                .expect("the retained circular conic shares full-circle fillet support")
                .into_value();
            assert_eq!(extended.candidate_count(), 3);

            assert_eq!(
                path.chamfer_vertex_by_setbacks(
                    1,
                    Real::zero(),
                    Real::zero(),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap()
                .into_value(),
                CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ZeroDesignValue)
            );
            assert_eq!(
                path.fillet_vertex_by_radius(
                    1,
                    Real::zero(),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap()
                .into_value(),
                CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ZeroDesignValue)
            );
        }
    }
}

#[test]
fn retained_circular_conic_pairs_extend_on_native_supports() {
    let previous = CircularArc2::try_from_center(p(-1, -1), p(0, 0), p(0, -1), true).unwrap();
    let next = CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap();
    let retained = |arc: &CircularArc2, elevated: bool| {
        let conic = arc
            .rational_bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value()
            .spans()[0]
            .curve()
            .clone();
        if elevated {
            Curve2::from(RationalBezier2::from(conic).elevated_to_degree(5).unwrap())
        } else {
            Curve2::from(conic)
        }
    };

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for elevated in [false, true] {
            for reversed in [false, true] {
                let path = CurvePath2::try_new(vec![
                    retained(&previous, elevated),
                    retained(&next, elevated),
                ])
                .unwrap();
                let path = if reversed {
                    path.reversed(&policy).unwrap().into_value()
                } else {
                    path
                };
                let trim = path
                    .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)
                    .unwrap()
                    .into_value();
                assert_eq!(trim.candidate_count(), 1);
                let extended = path
                    .fillet_vertex_by_radius(
                        1,
                        q(1, 2),
                        CurveCornerMode2::TrimOrExtend,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "retained circular supports must extend: policy={policy:?}, elevated={elevated}, reversed={reversed}, error={error:?}"
                        )
                    })
                    .into_value();
                assert_eq!(extended.candidate_count(), 2);

                let boundary_path = CurvePath2::try_new(vec![
                    retained(&previous, elevated),
                    retained(&next, elevated),
                    Curve2::from(LineSeg2::try_new(p(1, 1), p(-2, 1)).unwrap()),
                    Curve2::from(LineSeg2::try_new(p(-2, 1), p(-2, -1)).unwrap()),
                    Curve2::from(LineSeg2::try_new(p(-2, -1), p(-1, -1)).unwrap()),
                ])
                .unwrap();
                let boundary_path = if reversed {
                    boundary_path.reversed(&policy).unwrap().into_value()
                } else {
                    boundary_path
                };
                let region = CurveRegion2::try_from_boundary_paths(&[boundary_path], &policy)
                    .unwrap()
                    .into_value();
                let fragments = region.boundary_loops()[0].fragments();
                let corner = (0..fragments.len())
                    .find(|index| {
                        let is_rational_arc = |fragment: &BezierSplitFragment2| {
                            matches!(
                                fragment,
                                BezierSplitFragment2::Materialized {
                                    curve: BezierSubcurve2::RationalQuadratic(_)
                                        | BezierSubcurve2::Rational(_),
                                    ..
                                }
                            )
                        };
                        is_rational_arc(&fragments[(index + fragments.len() - 1) % fragments.len()])
                            && is_rational_arc(&fragments[*index])
                    })
                    .expect("the retained circular pair stays adjacent in CurveRegion2");
                let trim = region
                    .fillet_loop_vertex_by_radius(
                        0,
                        corner,
                        q(1, 2),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap();
                let extended = region
                    .fillet_loop_vertex_by_radius(
                        0,
                        corner,
                        q(1, 2),
                        CurveCornerMode2::TrimOrExtend,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "CurveRegion2 retained circular supports must extend: policy={policy:?}, elevated={elevated}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert!(extended.value.candidate_count() > trim.value.candidate_count());
            }
        }
    }
}

#[test]
fn retained_circular_corner_recognition_uses_the_shared_approximate_terminal() {
    let native_arc = CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap();
    let conic = native_arc
        .rational_bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .spans()[0]
        .curve();
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let control = Point2::new(
        conic.control().x() + undecidable_zero,
        conic.control().y().clone(),
    );
    let conic = RationalQuadraticBezier2::try_new(
        conic.start().clone(),
        control,
        conic.end().clone(),
        conic.start_weight().clone(),
        conic.control_weight().clone(),
        conic.end_weight().clone(),
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-2, 0), p(0, 0)).unwrap()),
        Curve2::from(conic),
    ])
    .unwrap();

    assert!(matches!(
        path.fillet_vertex_by_radius(
            1,
            q(1, 2),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Fillet
                && blocker.family() == CurveFamily2::RationalQuadraticBezier
    ));
    let approximate = path
        .fillet_vertex_by_radius(
            1,
            q(1, 2),
            CurveCornerMode2::TrimOnly,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(matches!(
        approximate.value,
        CurveCornerSolutions2::Unique(_)
    ));
}

#[test]
fn exact_native_arc_fillet_solver_classifies_collapsed_and_coincident_offsets() {
    let line_arc = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-2, 0), p(0, 0)).unwrap()),
        Curve2::from(CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap()),
    ])
    .unwrap();
    let same_circle = CurvePath2::try_new(vec![
        Curve2::from(CircularArc2::try_from_center(p(0, -1), p(1, 0), p(0, 0), false).unwrap()),
        Curve2::from(CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap()),
    ])
    .unwrap();
    let half_root_three = r(3).sqrt().unwrap() / r(2);
    let shared_corner = Point2::new(q(1, 2), half_root_three.unwrap());
    let disjoint_offsets = CurvePath2::try_new(vec![
        Curve2::from(
            CircularArc2::try_from_center(p(-1, 0), shared_corner.clone(), p(0, 0), true).unwrap(),
        ),
        Curve2::from(
            CircularArc2::try_from_center(shared_corner, p(2, 0), p(1, 0), false).unwrap(),
        ),
    ])
    .unwrap();
    let major_arcs = CurvePath2::try_new(vec![
        Curve2::from(CircularArc2::try_from_center(p(1, -1), p(0, 0), p(0, -1), true).unwrap()),
        Curve2::from(CircularArc2::try_from_center(p(0, 0), p(1, -1), p(1, 0), true).unwrap()),
    ])
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            line_arc
                .fillet_vertex_by_radius(1, Real::one(), CurveCornerMode2::TrimOnly, &policy)
                .unwrap()
                .into_value(),
            CurveCornerSolutions2::Unique(_)
        ));
        assert_eq!(
            same_circle
                .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)
                .unwrap()
                .into_value(),
            CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::DegenerateCandidate)
        );
        assert_eq!(
            disjoint_offsets
                .fillet_vertex_by_radius(1, q(3, 4), CurveCornerMode2::TrimOrExtend, &policy)
                .unwrap()
                .into_value(),
            CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::NoTangentCircle)
        );
        let CurveCornerSolutions2::Multiple(fillets) = major_arcs
            .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)
            .unwrap()
            .into_value()
        else {
            panic!("both exact offset-circle intersections must survive major sweeps");
        };
        assert_eq!(fillets.len(), 2);
        assert_ne!(
            fillets[0].curves()[1].geometry(),
            fillets[1].curves()[1].geometry()
        );
    }
}

#[test]
fn exact_native_arc_fillet_uses_only_the_shared_approximate_terminal() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-2, 0), p(0, 0)).unwrap()),
        Curve2::from(CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap()),
    ])
    .unwrap();
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let radius = Real::one() + undecidable_zero;
    assert!(matches!(
        path.fillet_vertex_by_radius(
            1,
            radius.clone(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == UncertaintyReason::RealSign
    ));
    let approximate = path
        .fillet_vertex_by_radius(
            1,
            radius,
            CurveCornerMode2::TrimOnly,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(matches!(
        approximate.value,
        CurveCornerSolutions2::Unique(_)
    ));
}

#[test]
fn radical_line_image_fillets_preserve_retained_families() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(p(-1, -1), p(0, 0)).unwrap(),
        )),
        Curve2::from(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(p(0, 0), p(-1, 1)).unwrap(),
        )),
    ])
    .unwrap();
    let CurveCornerSolutions2::Unique(fillet) = path
        .fillet_vertex_by_radius(
            1,
            q(1, 2),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    else {
        panic!("the radical line-image corner must have one exact fillet");
    };
    assert_eq!(fillet.curves()[0].family(), CurveFamily2::QuadraticBezier);
    assert_eq!(fillet.curves()[2].family(), CurveFamily2::QuadraticBezier);
    for distance_squared in [
        fillet.curves()[0].end().distance_squared(&p(0, 0)),
        fillet.curves()[2].start().distance_squared(&p(0, 0)),
    ] {
        assert!(matches!(
            distance_squared.certified_eq_until(&q(1, 4), -4096),
            CertifiedRealEquality::Equal { .. }
        ));
    }
    let CurveGeometry2::CircularArc(arc) = fillet.curves()[1].geometry() else {
        panic!("the radical line-image fillet must remain circular");
    };
    assert!(matches!(
        arc.radius_squared().certified_eq_until(&q(1, 4), -4096),
        CertifiedRealEquality::Equal { .. }
    ));
}

#[test]
fn line_corner_solvers_enumerate_extensions_deterministically() {
    let path = right_angle_line_path(4);
    let CurveCornerSolutions2::Multiple(chamfers) = path
        .chamfer_vertex_by_setbacks(
            1,
            r(1),
            r(1),
            CurveCornerMode2::TrimOrExtend,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    else {
        panic!("trim-or-extend setbacks must expose all four line-support choices");
    };
    assert_eq!(chamfers.len(), 4);
    assert_eq!(chamfers[0].curves()[0].end(), &p(-1, 0));
    assert_eq!(chamfers[0].curves()[2].start(), &p(0, 1));
    assert_eq!(chamfers[1].curves()[0].end(), &p(-1, 0));
    assert_eq!(chamfers[1].curves()[2].start(), &p(0, -1));
    assert_eq!(chamfers[2].curves()[0].end(), &p(1, 0));
    assert_eq!(chamfers[2].curves()[2].start(), &p(0, 1));
    assert_eq!(chamfers[3].curves()[0].end(), &p(1, 0));
    assert_eq!(chamfers[3].curves()[2].start(), &p(0, -1));

    let CurveCornerSolutions2::Multiple(fillets) = path
        .fillet_vertex_by_radius(
            1,
            r(1),
            CurveCornerMode2::TrimOrExtend,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    else {
        panic!("trim-or-extend must expose both oriented line fillets");
    };
    assert_eq!(fillets.len(), 2);
    let centers_and_orientations = fillets
        .iter()
        .map(|fillet| {
            let CurveGeometry2::CircularArc(arc) = fillet.curves()[1].geometry() else {
                panic!("every line fillet candidate must contain one circular arc");
            };
            (arc.center().clone(), arc.is_clockwise())
        })
        .collect::<Vec<_>>();
    assert_eq!(
        centers_and_orientations,
        vec![(p(-1, 1), false), (p(1, -1), true)]
    );
}

#[test]
fn polynomial_chamfer_materializes_represented_incident_extension() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let path = CurvePath2::try_new(vec![
            Curve2::from(QuadraticBezier2::new(
                p(-1, 1),
                Point2::new(-q(1, 2), Real::zero()),
                p(0, 0),
            )),
            Curve2::from(LineSeg2::try_new(p(0, 0), p(0, 3)).unwrap()),
        ])
        .unwrap();
        for reversed in [false, true] {
            let path = if reversed {
                path.clone().reversed(&policy).unwrap().into_value()
            } else {
                path.clone()
            };
            let setback = Real::from(2_i8).sqrt().unwrap();
            let (previous_setback, next_setback) = if reversed {
                (Real::zero(), setback)
            } else {
                (setback, Real::zero())
            };
            let result = path
                .chamfer_vertex_by_setbacks(
                    1,
                    previous_setback,
                    next_setback,
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .expect("the represented polynomial extension must materialize");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(edited) = result.into_value() else {
                panic!("the represented polynomial extension must be unique");
            };
            let quadratic = edited
                .curves()
                .iter()
                .find(|curve| curve.family() == CurveFamily2::QuadraticBezier)
                .expect("the extended source family must be retained");
            assert!(quadratic.start() == &p(1, 1) || quadratic.end() == &p(1, 1));
        }
    }
}

#[test]
fn rational_chamfer_materializes_the_incident_projective_cell() {
    let half = q(1, 2);
    let quarter = q(1, 4);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let path = CurvePath2::try_new(vec![
            Curve2::from(
                RationalBezier2::try_new(
                    vec![p(0, 0), Point2::new(half.clone(), Real::zero()), p(1, 1)],
                    vec![Real::one(), half.clone(), quarter.clone()],
                )
                .unwrap(),
            ),
            Curve2::from(LineSeg2::try_new(p(1, 1), p(1, 12)).unwrap()),
        ])
        .unwrap();
        for reversed in [false, true] {
            let path = if reversed {
                path.clone().reversed(&policy).unwrap().into_value()
            } else {
                path.clone()
            };
            let setback = Real::from(68_i8).sqrt().unwrap();
            let (previous_setback, next_setback) = if reversed {
                (Real::zero(), setback)
            } else {
                (setback, Real::zero())
            };
            let result = path
                .chamfer_vertex_by_setbacks(
                    1,
                    previous_setback,
                    next_setback,
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .expect("the incident rational cell must extend before its pole");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(edited) = result.into_value() else {
                panic!("the pole-partitioned rational extension must be unique");
            };
            let rational = edited
                .curves()
                .iter()
                .find(|curve| curve.family() == CurveFamily2::RationalBezier)
                .expect("the exact rational carrier must be retained");
            assert!(rational.start() == &p(3, 9) || rational.end() == &p(3, 9));
        }
    }
}

#[test]
fn line_parabola_mixed_exact_algebraic_fillet_requires_retained_region() {
    let half = q(1, 2);
    let radius = q(299, 125);
    let line_direction = Point2::new(q(38280, 91901), q(83549, 91901));
    let corner = p(1, 1);
    let line_end = Point2::new(
        corner.x() + line_direction.x(),
        corner.y() + line_direction.y(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let path = CurvePath2::try_new(vec![
            Curve2::from(QuadraticBezier2::new(
                p(0, 0),
                Point2::new(half.clone(), Real::zero()),
                corner.clone(),
            )),
            Curve2::from(LineSeg2::try_new(corner.clone(), line_end.clone()).unwrap()),
        ])
        .unwrap();
        for reversed in [false, true] {
            let path = if reversed {
                path.clone().reversed(&policy).unwrap().into_value()
            } else {
                path.clone()
            };
            let result = path.fillet_vertex_by_radius(
                1,
                radius.clone(),
                CurveCornerMode2::TrimOrExtend,
                &policy,
            );
            assert!(
                matches!(
                    &result,
                    Err(ExactCurveError::Blocked(blocker))
                        if blocker.operation() == CurveOperation2::Fillet
                            && matches!(
                                blocker.family(),
                                CurveFamily2::QuadraticBezier | CurveFamily2::Line
                            )
                            && blocker.reason() == UncertaintyReason::Unsupported
                ),
                "policy={policy:?}, reversed={reversed}, result={result:?}"
            );
        }
    }
}

#[test]
fn line_corner_solvers_report_exact_no_solution_and_invalid_options() {
    let path = right_angle_line_path(4);
    assert_eq!(
        path.fillet_vertex_by_radius(1, r(5), CurveCornerMode2::TrimOnly, &CurveContext::STRICT,)
            .unwrap()
            .into_value(),
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::OutsideTrimDomain)
    );
    assert_eq!(
        path.chamfer_vertex_by_setbacks(
            1,
            Real::zero(),
            Real::zero(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value(),
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ZeroDesignValue)
    );
    assert!(matches!(
        path.fillet_vertex_by_radius(
            1,
            -Real::one(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Invalid {
            operation: CurveOperation2::Fillet,
            cause: CurveError::InvalidCornerOptions,
            ..
        })
    ));
    assert!(matches!(
        path.chamfer_vertex_by_setbacks(
            1,
            -Real::one(),
            Real::one(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Invalid {
            operation: CurveOperation2::Chamfer,
            cause: CurveError::InvalidCornerOptions,
            ..
        })
    ));
    assert_eq!(
        path.chamfer_vertex_by_setbacks(
            1,
            r(4),
            Real::one(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value(),
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::OutsideTrimDomain)
    );

    let tangent_path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
    ])
    .unwrap();
    assert_eq!(
        tangent_path
            .fillet_vertex_by_radius(
                1,
                Real::one(),
                CurveCornerMode2::TrimOrExtend,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ParallelTangents)
    );

    let backtracking = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-2, 0), p(0, 0)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, 0), p(-2, 0)).unwrap()),
    ])
    .unwrap();
    assert_eq!(
        backtracking
            .chamfer_vertex_by_setbacks(
                1,
                Real::one(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::DegenerateCandidate)
    );
}

#[test]
fn automatic_corner_solver_keeps_unsupported_pairs_explicit() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            Point2::new(-q(4, 5), q(3, 5)),
            Point2::new(-q(4, 5), q(1, 10)),
            p(0, 0),
        )),
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(2, 1), p(0, 2))),
    ])
    .unwrap();
    assert!(matches!(
        path.fillet_vertex_by_radius(
            1,
            Real::one(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Fillet
                && blocker.family() == CurveFamily2::QuadraticBezier
                && blocker.reason() == UncertaintyReason::Unsupported
    ));

    let spline = CurvePath2::try_new(vec![
        Curve2::try_polynomial_bspline(
            2,
            vec![p(-4, 0), p(-2, 1), p(0, 0)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value(),
        Curve2::from(LineSeg2::try_new(p(0, 0), p(0, 4)).unwrap()),
    ])
    .unwrap();
    assert!(matches!(
        spline.fillet_vertex_by_radius(
            1,
            Real::one(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Fillet
                && blocker.family() == CurveFamily2::PolynomialBSpline
                && blocker.reason() == UncertaintyReason::Unsupported
    ));
    assert_eq!(
        spline
            .fillet_vertex_by_radius(
                1,
                Real::zero(),
                CurveCornerMode2::TrimOrExtend,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ZeroDesignValue)
    );
    assert_eq!(
        spline
            .chamfer_vertex_by_setbacks(
                1,
                Real::zero(),
                Real::zero(),
                CurveCornerMode2::TrimOrExtend,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ZeroDesignValue)
    );

    let algebraic_cut = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
    ])
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            algebraic_cut.chamfer_vertex_by_setbacks(
                1,
                Real::one(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            ),
            Err(ExactCurveError::Blocked(blocker))
                if blocker.operation() == CurveOperation2::Chamfer
                    && blocker.family() == CurveFamily2::QuadraticBezier
                    && blocker.reason() == UncertaintyReason::Unsupported
        ));
    }
}

#[test]
fn represented_line_bezier_corners_use_the_general_incidence_kernel() {
    let conic = RationalQuadraticBezier2::try_new(
        p(0, 0),
        p(0, 1),
        p(1, 2),
        Real::one(),
        Real::one(),
        Real::one(),
    )
    .unwrap();
    let carriers = [
        (
            CurveFamily2::QuadraticBezier,
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
        ),
        (
            CurveFamily2::CubicBezier,
            Curve2::from(CubicBezier2::new(
                p(0, 0),
                Point2::new(Real::zero(), q(2, 3)),
                Point2::new(q(1, 3), q(4, 3)),
                p(1, 2),
            )),
        ),
        (
            CurveFamily2::RationalQuadraticBezier,
            Curve2::from(conic.clone()),
        ),
        (
            CurveFamily2::RationalBezier,
            Curve2::from(RationalBezier2::from(conic).elevated_to_degree(5).unwrap()),
        ),
    ];
    let expected_center = Point2::new(-q(39, 16), q(15, 4));
    let next_setback = (r(657).sqrt().unwrap() / r(16)).unwrap();
    for (family, carrier) in carriers {
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
            carrier,
        ])
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let solutions = path
                .fillet_vertex_by_radius(1, q(15, 4), CurveCornerMode2::TrimOnly, &policy)
                .unwrap()
                .into_value();
            let has_expected = |candidate: &CurvePath2| {
                let CurveGeometry2::CircularArc(fillet) = candidate.curves()[1].geometry() else {
                    return false;
                };
                candidate.curves()[2].family() == family
                    && fillet
                        .center()
                        .distance_squared(&expected_center)
                        .certified_eq_until(&Real::zero(), -4096)
                        .as_bool()
                        == Some(true)
            };
            match &solutions {
                CurveCornerSolutions2::Unique(candidate) => assert!(has_expected(candidate)),
                CurveCornerSolutions2::Multiple(candidates) => {
                    assert!(candidates.iter().any(has_expected));
                }
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the represented line/quadratic contact was lost: {reason:?}")
                }
            }

            let CurveCornerSolutions2::Unique(chamfered) = path
                .chamfer_vertex_by_setbacks(
                    1,
                    Real::one(),
                    next_setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap()
                .into_value()
            else {
                panic!("the represented quadratic circle contact must define one chamfer");
            };
            assert_eq!(chamfered.curves()[0].end(), &p(-1, 0));
            assert_eq!(
                chamfered.curves()[2].start(),
                &Point2::new(q(9, 16), q(3, 2))
            );
            assert_eq!(chamfered.curves()[2].family(), family);

            let reversed = path.clone().reversed(&policy).unwrap().into_value();
            let reversed_solutions = reversed
                .fillet_vertex_by_radius(1, q(15, 4), CurveCornerMode2::TrimOnly, &policy)
                .unwrap()
                .into_value();
            let reversed_has_expected = |candidate: &CurvePath2| {
                let CurveGeometry2::CircularArc(fillet) = candidate.curves()[1].geometry() else {
                    return false;
                };
                candidate.curves()[0].family() == family
                    && candidate.curves()[2].family() == CurveFamily2::Line
                    && fillet
                        .center()
                        .distance_squared(&expected_center)
                        .certified_eq_until(&Real::zero(), -4096)
                        .as_bool()
                        == Some(true)
            };
            match &reversed_solutions {
                CurveCornerSolutions2::Unique(candidate) => {
                    assert!(reversed_has_expected(candidate));
                }
                CurveCornerSolutions2::Multiple(candidates) => {
                    assert!(candidates.iter().any(reversed_has_expected));
                }
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the reversed represented curve/line contact was lost: {reason:?}")
                }
            }

            let CurveCornerSolutions2::Unique(reversed_chamfered) = reversed
                .chamfer_vertex_by_setbacks(
                    1,
                    next_setback.clone(),
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap()
                .into_value()
            else {
                panic!("the reversed represented curve/line chamfer must be unique");
            };
            assert_eq!(reversed_chamfered.curves()[0].family(), family);
            assert_eq!(
                reversed_chamfered.curves()[0].end(),
                &Point2::new(q(9, 16), q(3, 2))
            );
            assert_eq!(reversed_chamfered.curves()[2].start(), &p(-1, 0));
        }
    }
}

#[test]
fn spline_incident_spans_reuse_represented_bezier_corner_incidence() {
    let controls = vec![p(0, 0), p(0, 1), p(1, 2), p(2, 1), p(4, 2)];
    let knots = vec![r(2), r(2), r(2), r(5), r(5), r(9), r(9), r(9)];
    let carriers = [
        (
            CurveFamily2::PolynomialBSpline,
            Curve2::try_polynomial_bspline(
                2,
                controls.clone(),
                knots.clone(),
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
        ),
        (
            CurveFamily2::Nurbs,
            Curve2::try_nurbs(
                2,
                controls,
                vec![Real::one(); 5],
                knots,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
        ),
    ];
    let expected_cut = Point2::new(q(9, 16), q(3, 2));
    let expected_public_parameter = q(17, 4);
    let expected_reversed_parameter = q(27, 4);
    let next_setback = (r(657).sqrt().unwrap() / r(16)).unwrap();
    let expected_center = Point2::new(-q(39, 16), q(15, 4));

    for (family, carrier) in carriers {
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
            carrier,
        ])
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let CurveCornerSolutions2::Unique(chamfered) = path
                .chamfer_vertex_by_setbacks(
                    1,
                    Real::one(),
                    next_setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap()
                .into_value()
            else {
                panic!("the incident {family:?} span must define one exact chamfer");
            };
            let trimmed = &chamfered.curves()[2];
            assert_eq!(trimmed.family(), family);
            assert_eq!(trimmed.start(), &expected_cut);
            assert_eq!(
                trimmed.parameter_domain().start(),
                &expected_public_parameter
            );
            assert_eq!(trimmed.parameter_domain().end(), &r(9));

            let fillets = path
                .fillet_vertex_by_radius(1, q(15, 4), CurveCornerMode2::TrimOnly, &policy)
                .unwrap()
                .into_value();
            let has_expected = |candidate: &CurvePath2| {
                let CurveGeometry2::CircularArc(fillet) = candidate.curves()[1].geometry() else {
                    return false;
                };
                candidate.curves()[2].family() == family
                    && candidate.curves()[2].start() == &expected_cut
                    && fillet
                        .center()
                        .distance_squared(&expected_center)
                        .certified_eq_until(&Real::zero(), -4096)
                        .as_bool()
                        == Some(true)
            };
            match &fillets {
                CurveCornerSolutions2::Unique(candidate) => assert!(has_expected(candidate)),
                CurveCornerSolutions2::Multiple(candidates) => {
                    assert!(candidates.iter().any(has_expected));
                }
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the incident {family:?} span lost its exact fillet: {reason:?}")
                }
            }

            let reversed = path.clone().reversed(&policy).unwrap().into_value();
            let CurveCornerSolutions2::Unique(reversed_chamfered) = reversed
                .chamfer_vertex_by_setbacks(
                    1,
                    next_setback.clone(),
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap()
                .into_value()
            else {
                panic!("the reversed incident {family:?} span must remain exact");
            };
            let reversed_trimmed = &reversed_chamfered.curves()[0];
            assert_eq!(reversed_trimmed.family(), family);
            assert_eq!(reversed_trimmed.end(), &expected_cut);
            assert_eq!(reversed_trimmed.parameter_domain().start(), &r(2));
            assert_eq!(
                reversed_trimmed.parameter_domain().end(),
                &expected_reversed_parameter
            );
        }
    }
}

#[test]
fn spline_incident_span_pairs_reuse_exact_ph_fillet_fast_path() {
    let make_spline =
        |family: CurveFamily2, controls: Vec<Point2>, start: i32, end: i32| match family {
            CurveFamily2::PolynomialBSpline => Curve2::try_polynomial_bspline(
                3,
                controls,
                vec![r(start); 4]
                    .into_iter()
                    .chain(vec![r(end); 4])
                    .collect(),
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
            CurveFamily2::Nurbs => Curve2::try_nurbs(
                3,
                controls,
                vec![Real::one(); 4],
                vec![r(start); 4]
                    .into_iter()
                    .chain(vec![r(end); 4])
                    .collect(),
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
            _ => unreachable!(),
        };
    let previous_controls = vec![
        p(-4, 0),
        Point2::new(-q(8, 3), Real::zero()),
        Point2::new(-q(4, 3), Real::zero()),
        p(0, 0),
    ];
    let next_controls = vec![
        p(0, 0),
        Point2::new(Real::zero(), q(4, 3)),
        Point2::new(Real::zero(), q(8, 3)),
        p(0, 4),
    ];

    for previous_family in [CurveFamily2::PolynomialBSpline, CurveFamily2::Nurbs] {
        for next_family in [CurveFamily2::PolynomialBSpline, CurveFamily2::Nurbs] {
            let path = CurvePath2::try_new(vec![
                make_spline(previous_family, previous_controls.clone(), 2, 5),
                make_spline(next_family, next_controls.clone(), 7, 11),
            ])
            .unwrap();
            for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                let CurveCornerSolutions2::Unique(filleted) = path
                    .fillet_vertex_by_radius(1, Real::one(), CurveCornerMode2::TrimOnly, &policy)
                    .unwrap()
                    .into_value()
                else {
                    panic!(
                        "the exact {previous_family:?}/{next_family:?} PH spans must define one fillet"
                    );
                };
                assert_eq!(filleted.curves()[0].family(), previous_family);
                assert_eq!(filleted.curves()[0].end(), &p(-1, 0));
                assert_eq!(filleted.curves()[0].parameter_domain().end(), &q(17, 4));
                assert_eq!(filleted.curves()[2].family(), next_family);
                assert_eq!(filleted.curves()[2].start(), &p(0, 1));
                assert_eq!(filleted.curves()[2].parameter_domain().start(), &r(8));
                let CurveGeometry2::CircularArc(fillet) = filleted.curves()[1].geometry() else {
                    panic!("the exact PH-span fillet must remain circular");
                };
                assert_eq!(fillet.center(), &p(-1, 1));
            }
        }
    }
}

#[test]
fn represented_bezier_pairs_use_independent_chamfer_and_exact_ph_fillet_routes() {
    let next = QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2));
    let previous = QuadraticBezier2::new(p(-1, 2), p(0, 1), p(0, 0));
    let chamfer_path =
        CurvePath2::try_new(vec![Curve2::from(previous), Curve2::from(next)]).unwrap();
    let setback = (r(657).sqrt().unwrap() / r(16)).unwrap();

    let cubic_line_path = CurvePath2::try_new(vec![
        Curve2::from(CubicBezier2::new(
            p(-4, 0),
            Point2::new(-q(8, 3), Real::zero()),
            Point2::new(-q(4, 3), Real::zero()),
            p(0, 0),
        )),
        Curve2::from(CubicBezier2::new(
            p(0, 0),
            Point2::new(Real::zero(), q(4, 3)),
            Point2::new(Real::zero(), q(8, 3)),
            p(0, 4),
        )),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let CurveCornerSolutions2::Unique(chamfered) = chamfer_path
            .chamfer_vertex_by_setbacks(
                1,
                setback.clone(),
                setback.clone(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap()
            .into_value()
        else {
            panic!("the represented quadratic pair must define one chamfer");
        };
        assert_eq!(
            chamfered.curves()[0].end(),
            &Point2::new(-q(9, 16), q(3, 2))
        );
        assert_eq!(
            chamfered.curves()[2].start(),
            &Point2::new(q(9, 16), q(3, 2))
        );
        assert_eq!(
            chamfered.curves()[0].family(),
            CurveFamily2::QuadraticBezier
        );
        assert_eq!(
            chamfered.curves()[2].family(),
            CurveFamily2::QuadraticBezier
        );

        let CurveCornerSolutions2::Unique(filleted) = cubic_line_path
            .fillet_vertex_by_radius(1, Real::one(), CurveCornerMode2::TrimOnly, &policy)
            .unwrap()
            .into_value()
        else {
            panic!("the exact PH cubic pair must define one fillet");
        };
        assert_eq!(filleted.curves()[0].end(), &p(-1, 0));
        assert_eq!(filleted.curves()[2].start(), &p(0, 1));
        assert_eq!(filleted.curves()[0].family(), CurveFamily2::CubicBezier);
        assert_eq!(filleted.curves()[2].family(), CurveFamily2::CubicBezier);
        let CurveGeometry2::CircularArc(fillet) = filleted.curves()[1].geometry() else {
            panic!("the exact PH pair must insert a circular fillet");
        };
        assert_eq!(fillet.center(), &p(-1, 1));
    }
}

#[test]
fn direct_bezier_pair_fillet_materializes_both_incident_extensions() {
    // P(t) = (t, t^2) and
    // Q(s) = (1, 1) + (-31/65, -86/325)s + (-48/65, -43/325)s^2.
    // Their left parallels at distance 1/2 meet at the exact parameters
    // t = 6/5 and s = -1, outside both authored spans but inside their
    // endpoint-adjacent regular cells.
    let previous_cut = Point2::new(q(6, 5), q(36, 25));
    let next_cut = Point2::new(q(48, 65), q(368, 325));
    let expected_center = Point2::new(q(48, 65), q(1061, 650));
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            p(0, 0),
            Point2::new(q(1, 2), Real::zero()),
            p(1, 1),
        )),
        Curve2::from(QuadraticBezier2::new(
            p(1, 1),
            Point2::new(q(99, 130), q(282, 325)),
            Point2::new(-q(14, 65), q(196, 325)),
        )),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let path = if reversed {
                path.clone().reversed(&policy).unwrap().into_value()
            } else {
                path.clone()
            };
            let result = path
                .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOrExtend, &policy)
                .expect("both regular Bezier incident extensions must be solved exactly");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let has_expected = |candidate: &CurvePath2| {
                let CurveGeometry2::CircularArc(fillet) = candidate.curves()[1].geometry() else {
                    return false;
                };
                let (expected_previous, expected_next) = if reversed {
                    (&next_cut, &previous_cut)
                } else {
                    (&previous_cut, &next_cut)
                };
                candidate.curves()[0].end() == expected_previous
                    && candidate.curves()[2].start() == expected_next
                    && fillet.center() == &expected_center
                    && candidate.curves()[0].family() == CurveFamily2::QuadraticBezier
                    && candidate.curves()[2].family() == CurveFamily2::QuadraticBezier
            };
            match result.into_value() {
                CurveCornerSolutions2::Unique(candidate) => assert!(has_expected(&candidate)),
                CurveCornerSolutions2::Multiple(candidates) => {
                    assert!(candidates.iter().any(has_expected));
                }
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the exact projective Bezier fillet was lost: {reason:?}")
                }
            }
        }
    }
}

#[test]
fn independently_parameterized_bezier_continuation_is_an_incident_fillet_component() {
    // These are adjacent parameter spans of the same non-PH parabola:
    // P(t)=(t,t^2), Q(s)=(1+s,(1+s)^2). Their authored domains share only
    // the seam P(1)=Q(0), while their analytic continuations satisfy
    // s=t-1. Equal selected offsets therefore provide infinitely many
    // projective fillet centers and must be classified as a degeneracy rather
    // than disappearing when the squared pair component is saturated.
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            p(0, 0),
            Point2::new(q(1, 2), Real::zero()),
            p(1, 1),
        )),
        Curve2::from(QuadraticBezier2::new(
            p(1, 1),
            Point2::new(q(3, 2), r(2)),
            p(2, 4),
        )),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let path = if reversed {
                path.clone().reversed(&policy).unwrap().into_value()
            } else {
                path.clone()
            };
            let result = path
                .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOrExtend, &policy)
                .expect("the incident source component must be classified exactly");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert_eq!(
                result.into_value(),
                CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::DegenerateCandidate)
            );
        }
    }
}

#[test]
fn same_bezier_support_fillet_removes_the_projective_parameter_diagonal() {
    // This regular closed cubic has P(2) = (22/3, -4/3) with tangent
    // (13, 0), and P(-1) = (4/3, -22/3) with tangent (0, 13). Its right
    // parallel at distance 6 therefore has the off-diagonal self-contact
    // (2, -1) at (22/3, -22/3). The coincident t=s component must be divided
    // out before the two incident domains are projected.
    let previous_cut = Point2::new(q(22, 3), -q(4, 3));
    let next_cut = Point2::new(q(4, 3), -q(22, 3));
    let expected_center = Point2::new(q(22, 3), -q(22, 3));
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(-q(5, 9), q(8, 9)),
        Point2::new(-q(8, 9), q(5, 9)),
        p(0, 0),
    );
    let path =
        CurvePath2::try_new(vec![Curve2::from(source.clone()), Curve2::from(source)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let path = if reversed {
                path.clone().reversed(&policy).unwrap().into_value()
            } else {
                path.clone()
            };
            let result = path
                .fillet_vertex_by_radius(1, r(6), CurveCornerMode2::TrimOrExtend, &policy)
                .expect("the structural diagonal must leave complete off-diagonal contacts");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let has_expected = |candidate: &CurvePath2| {
                let CurveGeometry2::CircularArc(fillet) = candidate.curves()[1].geometry() else {
                    return false;
                };
                let (expected_previous, expected_next) = if reversed {
                    (&next_cut, &previous_cut)
                } else {
                    (&previous_cut, &next_cut)
                };
                candidate.curves()[0].end() == expected_previous
                    && candidate.curves()[2].start() == expected_next
                    && fillet.center() == &expected_center
            };
            match result.into_value() {
                CurveCornerSolutions2::Unique(candidate) => assert!(has_expected(&candidate)),
                CurveCornerSolutions2::Multiple(candidates) => {
                    assert!(candidates.iter().any(has_expected));
                }
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the off-diagonal projective fillet was lost: {reason:?}")
                }
            }
        }
    }
}

#[test]
fn same_ph_bezier_support_fillet_reuses_rational_projective_self_contact() {
    // With x=t-1/2 and b=sqrt(3)/6, this closed PH cubic is
    // P(x)=(x^3/3-x/12, b(x^2-1/4)); its hodograph is
    // (x^2-b^2, 2bx) and its everywhere-positive speed is x^2+b^2.
    // At signed left distance 13*sqrt(3)/48, the exact rational parallel has
    // the off-diagonal contact x=(1,-1), hence t=(3/2,-1/2), at
    // (0,17*sqrt(3)/48). The finite PH injectivity fast path must not discard
    // this exterior pair.
    let sqrt_three = r(3).sqrt().unwrap();
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 18), -(&sqrt_three / r(18)).unwrap()),
        Point2::new(-q(1, 18), -(&sqrt_three / r(18)).unwrap()),
        p(0, 0),
    );
    let radius = (&r(13) * &sqrt_three / r(48)).unwrap();
    let previous_cut = Point2::new(q(1, 4), (&sqrt_three / r(8)).unwrap());
    let next_cut = Point2::new(-q(1, 4), (&sqrt_three / r(8)).unwrap());
    let expected_center = Point2::new(Real::zero(), (&r(17) * &sqrt_three / r(48)).unwrap());
    assert!(matches!(
        source
            .parallel_left(radius.clone())
            .unwrap()
            .exact_pythagorean_hodograph_offset(&CurveContext::STRICT)
            .unwrap(),
        Classification::Decided(Some(_))
    ));
    let path =
        CurvePath2::try_new(vec![Curve2::from(source.clone()), Curve2::from(source)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let path = if reversed {
                path.clone().reversed(&policy).unwrap().into_value()
            } else {
                path.clone()
            };
            let result = path
                .fillet_vertex_by_radius(1, radius.clone(), CurveCornerMode2::TrimOrExtend, &policy)
                .expect("the exact PH parallel must retain its exterior self-contact");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let has_expected = |candidate: &CurvePath2| {
                let CurveGeometry2::CircularArc(fillet) = candidate.curves()[1].geometry() else {
                    return false;
                };
                let (expected_previous, expected_next) = if reversed {
                    (&next_cut, &previous_cut)
                } else {
                    (&previous_cut, &next_cut)
                };
                candidate.curves()[0].end() == expected_previous
                    && candidate.curves()[2].start() == expected_next
                    && fillet.center() == &expected_center
            };
            match result.into_value() {
                CurveCornerSolutions2::Unique(candidate) => assert!(has_expected(&candidate)),
                CurveCornerSolutions2::Multiple(candidates) => {
                    assert!(candidates.iter().any(has_expected));
                }
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the PH projective fillet was lost: {reason:?}")
                }
            }
        }
    }
}

#[test]
fn represented_arc_bezier_fillets_use_circle_incidence() {
    let source_center = Point2::new(-q(7, 16), q(207, 512));
    let source_start = Point2::new(-q(7, 16), -q(49, 256));
    let previous =
        CircularArc2::try_from_center(source_start, p(0, 0), source_center, true).unwrap();
    let conic = RationalQuadraticBezier2::try_new(
        p(0, 0),
        p(0, 1),
        p(1, 2),
        Real::one(),
        Real::one(),
        Real::one(),
    )
    .unwrap();
    let carriers = [
        (
            CurveFamily2::QuadraticBezier,
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
        ),
        (
            CurveFamily2::CubicBezier,
            Curve2::from(CubicBezier2::new(
                p(0, 0),
                Point2::new(Real::zero(), q(2, 3)),
                Point2::new(q(1, 3), q(4, 3)),
                p(1, 2),
            )),
        ),
        (
            CurveFamily2::RationalQuadraticBezier,
            Curve2::from(conic.clone()),
        ),
        (
            CurveFamily2::RationalBezier,
            Curve2::from(RationalBezier2::from(conic).elevated_to_degree(5).unwrap()),
        ),
    ];
    let expected_center = Point2::new(-q(7, 16), q(9, 4));

    for (family, carrier) in carriers {
        let path = CurvePath2::try_new(vec![Curve2::from(previous.clone()), carrier]).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let solutions = path
                .fillet_vertex_by_radius(1, q(5, 4), CurveCornerMode2::TrimOnly, &policy)
                .unwrap()
                .into_value();
            let has_expected = |candidate: &CurvePath2| {
                let CurveGeometry2::CircularArc(fillet) = candidate.curves()[1].geometry() else {
                    return false;
                };
                fillet
                    .center()
                    .distance_squared(&expected_center)
                    .certified_eq_until(&Real::zero(), -4096)
                    .as_bool()
                    == Some(true)
                    && candidate.curves()[0].family() == CurveFamily2::CircularArc
                    && candidate.curves()[2].family() == family
            };
            match &solutions {
                CurveCornerSolutions2::Unique(candidate) => assert!(has_expected(candidate)),
                CurveCornerSolutions2::Multiple(candidates) => {
                    assert!(candidates.iter().any(has_expected));
                }
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the represented arc/quadratic contact was lost: {reason:?}")
                }
            }

            let reversed = path.clone().reversed(&policy).unwrap().into_value();
            let reversed_solutions = reversed
                .fillet_vertex_by_radius(1, q(5, 4), CurveCornerMode2::TrimOnly, &policy)
                .unwrap()
                .into_value();
            let reversed_has_expected = |candidate: &CurvePath2| {
                let CurveGeometry2::CircularArc(fillet) = candidate.curves()[1].geometry() else {
                    return false;
                };
                candidate.curves()[0].family() == family
                    && candidate.curves()[2].family() == CurveFamily2::CircularArc
                    && fillet
                        .center()
                        .distance_squared(&expected_center)
                        .certified_eq_until(&Real::zero(), -4096)
                        .as_bool()
                        == Some(true)
            };
            match &reversed_solutions {
                CurveCornerSolutions2::Unique(candidate) => {
                    assert!(reversed_has_expected(candidate));
                }
                CurveCornerSolutions2::Multiple(candidates) => {
                    assert!(candidates.iter().any(reversed_has_expected));
                }
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the reversed represented curve/arc contact was lost: {reason:?}")
                }
            }
        }
    }
}

#[test]
fn represented_bezier_chamfer_retains_more_than_two_exact_cuts() {
    // This regular cubic meets the unit circle about its start at t = 1/4,
    // 1/2, and 3/4. The remaining cubic factor of the circle equation has no
    // root in [0, 1], so these are the complete cuts rather than samples.
    let cubic = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(266, 195), q(200, 117)),
        Point2::new(q(28, 65), q(272, 585)),
        Point2::new(-q(30, 13), q(56, 65)),
    );
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-1, 0), p(0, 0)).unwrap()),
        Curve2::from(cubic),
    ])
    .unwrap();
    let expected = [
        Point2::new(q(3, 5), q(4, 5)),
        Point2::new(q(5, 13), q(12, 13)),
        Point2::new(-q(3, 5), q(4, 5)),
    ];

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let CurveCornerSolutions2::Multiple(candidates) = path
            .chamfer_vertex_by_setbacks(
                1,
                Real::zero(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap()
            .into_value()
        else {
            panic!("all three represented cubic circle contacts must be retained");
        };
        assert_eq!(candidates.len(), expected.len());
        for (candidate, expected) in candidates.iter().zip(&expected) {
            assert_eq!(candidate.curves()[2].family(), CurveFamily2::CubicBezier);
            assert_eq!(candidate.curves()[2].start(), expected);
        }
    }
}

#[test]
fn represented_bezier_corner_incidence_uses_the_shared_approximate_terminal() {
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(
            p(0, 0),
            Point2::new(undecidable_zero, Real::one()),
            p(1, 2),
        )),
    ])
    .unwrap();

    assert!(matches!(
        path.fillet_vertex_by_radius(
            1,
            q(15, 4),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Fillet
                && blocker.family() == CurveFamily2::QuadraticBezier
    ));
    let approximate = path
        .fillet_vertex_by_radius(
            1,
            q(15, 4),
            CurveCornerMode2::TrimOnly,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_ne!(approximate.value.candidate_count(), 0);
}

#[test]
fn spline_corner_incidence_uses_the_shared_approximate_terminal() {
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    for family in [CurveFamily2::PolynomialBSpline, CurveFamily2::Nurbs] {
        let controls = vec![
            p(0, 0),
            Point2::new(undecidable_zero.clone(), Real::one()),
            p(1, 2),
        ];
        let knots = vec![r(2), r(2), r(2), r(5), r(5), r(5)];
        let carrier = match family {
            CurveFamily2::PolynomialBSpline => {
                Curve2::try_polynomial_bspline(2, controls, knots, &CurveContext::STRICT)
            }
            CurveFamily2::Nurbs => Curve2::try_nurbs(
                2,
                controls,
                vec![Real::one(); 3],
                knots,
                &CurveContext::STRICT,
            ),
            _ => unreachable!(),
        }
        .unwrap()
        .into_value();
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
            carrier,
        ])
        .unwrap();

        assert!(matches!(
            path.fillet_vertex_by_radius(
                1,
                q(15, 4),
                CurveCornerMode2::TrimOnly,
                &CurveContext::STRICT,
            ),
            Err(ExactCurveError::Blocked(blocker))
                if blocker.operation() == CurveOperation2::Fillet
                    && blocker.family() == family
        ));
        let approximate = path
            .fillet_vertex_by_radius(
                1,
                q(15, 4),
                CurveCornerMode2::TrimOnly,
                &CurveContext::APPROXIMATE_512,
            )
            .unwrap();
        assert_eq!(
            approximate.certainty,
            CurveCertainty::Approximate512Consumed
        );
        assert_ne!(approximate.value.candidate_count(), 0);
    }
}

#[test]
fn automatic_corner_solver_obeys_strict_and_approximate_512_once() {
    let path = right_angle_line_path(4);
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    assert!(matches!(
        path.fillet_vertex_by_radius(
            1,
            undecidable_zero.clone(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Fillet
                && blocker.reason() == UncertaintyReason::RealSign
    ));
    let approximate = path
        .fillet_vertex_by_radius(
            1,
            undecidable_zero,
            CurveCornerMode2::TrimOnly,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        approximate.value,
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ZeroDesignValue)
    );

    let undecidable_parallel = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let near_tangent = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-1, 0), p(0, 0)).unwrap()),
        Curve2::from(
            LineSeg2::try_new(p(0, 0), Point2::new(Real::one(), undecidable_parallel)).unwrap(),
        ),
    ])
    .unwrap();
    assert!(matches!(
        near_tangent.fillet_vertex_by_radius(
            1,
            Real::one(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == UncertaintyReason::RealSign
    ));
    let approximate = near_tangent
        .fillet_vertex_by_radius(
            1,
            Real::one(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        approximate.value,
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ParallelTangents)
    );
}

proptest! {
    #[test]
    fn exact_line_corner_edits_hold_for_rational_trim_parameters(
        radius in 1_i32..32,
        previous_remainder in 1_i32..32,
        next_remainder in 1_i32..32,
    ) {
        let previous_length = radius + previous_remainder;
        let next_length = radius + next_remainder;
        let path = CurvePath2::try_new(vec![
            Curve2::from(
                LineSeg2::try_new(p(-previous_length, 0), p(0, 0)).unwrap()
            ),
            Curve2::from(
                LineSeg2::try_new(p(0, 0), p(0, next_length)).unwrap()
            ),
        ])
        .unwrap();
        let previous_parameter = q(previous_remainder, previous_length);
        let next_parameter = q(radius, next_length);

        let chamfered = path
            .chamfer_vertex_by_parameters(
                1,
                previous_parameter.clone(),
                next_parameter.clone(),
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value();
        prop_assert_eq!(chamfered.curves()[0].end(), &p(-radius, 0));
        prop_assert_eq!(chamfered.curves()[1].end(), &p(0, radius));
        let CurveCornerSolutions2::Unique(solved_chamfer) = path
            .chamfer_vertex_by_setbacks(
                1,
                r(radius),
                r(radius),
                CurveCornerMode2::TrimOnly,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value()
        else {
            return Err(TestCaseError::fail("line setbacks must solve uniquely"));
        };
        prop_assert_eq!(&solved_chamfer, &chamfered);

        let filleted = path
            .fillet_vertex_by_parameters(
                1,
                previous_parameter,
                next_parameter,
                &p(-radius, radius),
                false,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value();
        prop_assert_eq!(filleted.curves()[0].end(), &p(-radius, 0));
        prop_assert_eq!(filleted.curves()[1].family(), CurveFamily2::CircularArc);
        prop_assert_eq!(filleted.curves()[1].end(), &p(0, radius));
        prop_assert_eq!(filleted.curves()[2].start(), &p(0, radius));
        let CurveCornerSolutions2::Unique(solved_fillet) = path
            .fillet_vertex_by_radius(
                1,
                r(radius),
                CurveCornerMode2::TrimOnly,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value()
        else {
            return Err(TestCaseError::fail("line radius must solve uniquely"));
        };
        prop_assert_eq!(solved_fillet, filleted);
    }

    #[test]
    fn exact_line_arc_fillets_retain_rational_radius_across_source_scales(
        source_radius in 1_i32..16,
        fillet_numerator in 1_i32..24,
        fillet_denominator in 1_i32..8,
    ) {
        let line_length = 4 * (source_radius + fillet_numerator + fillet_denominator);
        let path = CurvePath2::try_new(vec![
            Curve2::from(
                LineSeg2::try_new(p(-line_length, 0), p(0, 0)).unwrap()
            ),
            Curve2::from(
                CircularArc2::try_from_center(
                    p(0, 0),
                    p(source_radius, source_radius),
                    p(source_radius, 0),
                    true,
                )
                .unwrap()
            ),
        ])
        .unwrap();
        let radius = q(fillet_numerator, fillet_denominator);
        let CurveCornerSolutions2::Unique(filleted) = path
            .fillet_vertex_by_radius(
                1,
                radius.clone(),
                CurveCornerMode2::TrimOnly,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value()
        else {
            return Err(TestCaseError::fail("line/arc radius must solve uniquely"));
        };
        let CurveGeometry2::CircularArc(inserted) = filleted.curves()[1].geometry() else {
            return Err(TestCaseError::fail("the solved fillet must remain circular"));
        };
        prop_assert_eq!(filleted.curves()[0].end(), inserted.start());
        prop_assert_eq!(filleted.curves()[2].start(), inserted.end());
        prop_assert_eq!(
            inserted
                .radius_squared()
                .certified_eq_until(&(&radius * &radius), -4096)
                .as_bool(),
            Some(true)
        );
        let CurveGeometry2::CircularArc(retained) = filleted.curves()[2].geometry() else {
            return Err(TestCaseError::fail("the source arc must be retained"));
        };
        prop_assert_eq!(retained.center(), &p(source_radius, 0));
        prop_assert_eq!(retained.end(), &p(source_radius, source_radius));
    }
}
