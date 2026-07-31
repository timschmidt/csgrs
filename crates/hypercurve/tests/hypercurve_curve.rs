use hypercurve::{
    CircularArc2, Classification, CubicBezier2, Curve2, CurveContext, CurveError, CurveFamily2,
    CurveGeometry2, CurveOperation2, CurvePath2, CurveRegion2, ExactCurveError, LineSeg2, Point2,
    QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2, Real, RegionPointLocation,
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
        CurveFamily2::PolynomialBSpline => {
            Curve2::try_polynomial_bspline(1, vec![start, end], vec![r(0), r(0), r(1), r(1)])
                .unwrap()
        }
        CurveFamily2::Nurbs => Curve2::try_nurbs(
            1,
            vec![start, end],
            vec![r(1), r(1)],
            vec![r(0), r(0), r(1), r(1)],
        )
        .unwrap(),
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
        )
        .unwrap(),
        Curve2::try_nurbs(
            2,
            vec![p(14, 0), p(15, 2), p(16, 0)],
            vec![r(1), r(2), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        )
        .unwrap(),
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
    let region =
        CurveRegion2::try_from_boundary_paths(&[every_family_closed_path()], &CurveContext::STRICT)
            .unwrap();
    let clone = region.clone();
    let signed_area = region
        .signed_area()
        .unwrap()
        .expect("degree-two rational boundary contributions are exact");
    assert_eq!(clone.signed_area(), Ok(Some(signed_area)));
    assert_eq!(
        region.classify_point(&p(8, -1), &CurveContext::STRICT),
        Ok(Classification::Decided(RegionPointLocation::Inside))
    );
    assert_eq!(
        clone.classify_point(&p(8, -4), &CurveContext::STRICT),
        Ok(Classification::Decided(RegionPointLocation::Outside))
    );
    assert_eq!(
        clone.classify_point(&p(0, 0), &CurveContext::STRICT),
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
    let bounded = CurveRegion2::try_from_boundary_paths(&[square], &CurveContext::STRICT).unwrap();
    let bounded_clone = bounded.clone();
    assert_eq!(
        bounded.classify_point(&p(1, 1), &CurveContext::STRICT),
        Ok(Classification::Decided(RegionPointLocation::Inside))
    );
    assert_eq!(
        bounded_clone.classify_point(&p(1, 1), &CurveContext::STRICT),
        Ok(Classification::Decided(RegionPointLocation::Inside))
    );
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
    )
    .unwrap();

    assert_eq!(line.point_at(&half).unwrap(), p(1, 0));
    assert_eq!(
        (
            line.parameter_domain().start(),
            line.parameter_domain().end()
        ),
        (&r(0), &r(1))
    );
    assert_eq!(quadratic.as_view().point_at(&half).unwrap(), p(1, 1));
    assert_eq!(spline.point_at(&r(1)).unwrap(), p(1, 1));
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
            curve.point_at(curve.parameter_domain().start()).unwrap(),
            curve.start().clone()
        );
        assert_eq!(
            curve.point_at(curve.parameter_domain().end()).unwrap(),
            curve.end().clone()
        );
    }

    let rational =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 2), p(2, 0)], vec![r(1), r(2), r(3)]).unwrap();
    let top_level = Curve2::from(rational.clone());
    assert_eq!(top_level.point_at(&r(0)).unwrap(), p(0, 0));
    assert_eq!(top_level.point_at(&r(1)).unwrap(), p(2, 0));
    assert_eq!(
        top_level.point_at(&(r(1) / r(2)).unwrap()).unwrap(),
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
    let line_derivative = line.as_view().derivative_at(&half).unwrap();
    assert_eq!(line_derivative.dx(), &r(2));
    assert_eq!(line_derivative.dy(), &r(0));
    assert_eq!(line_clone.derivative_at(&half).unwrap(), line_derivative);

    let quadratic = Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0)));
    let quadratic_derivative = quadratic.derivative_at(&half).unwrap();
    assert_eq!(quadratic_derivative.dx(), &r(2));
    assert_eq!(quadratic_derivative.dy(), &r(0));

    let spline = Curve2::try_polynomial_bspline(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
    )
    .unwrap();
    let CurveGeometry2::PolynomialBSpline(retained_spline) = spline.geometry() else {
        panic!("top-level polynomial constructor returned another family");
    };
    let spline_derivative = spline.derivative_at(&r(1)).unwrap();
    assert_eq!(spline_derivative.dx(), &r(1));
    assert_eq!(spline_derivative.dy(), &r(0));
    assert_eq!(
        retained_spline.derivative_at(&r(1)).unwrap(),
        spline_derivative
    );
    assert_eq!(spline.derivative_at(&r(1)).unwrap(), spline_derivative);
}

#[test]
fn top_level_curve_and_view_expose_exact_higher_derivatives() {
    let curve = Curve2::from(
        hypercurve::RationalBezier2::try_new(vec![p(0, 0), p(4, 0)], vec![r(1), r(3)]).unwrap(),
    );
    let half = (r(1) / r(2)).unwrap();

    let derivatives = curve.as_view().derivatives_at(&half, 3).unwrap();

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
            .chamfer_vertex_by_parameters(vertex_index, q(1, 2), q(1, 2))
            .unwrap();
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
                .fillet_vertex_by_parameters(1, q(1, 2), q(1, 2), &p(-1, 1), false)
                .unwrap_or_else(|error| {
                    panic!("{previous_family:?}/{next_family:?} fillet failed: {error}")
                });

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
        .fillet_vertex_by_parameters(1, q(3, 5), next_parameter, &p(3, 1), false)
        .unwrap();

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
        .fillet_vertex_by_parameters(1, previous_parameter, q(2, 5), &p(3, 1), true)
        .unwrap();
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
        .chamfer_vertex_by_parameters(0, q(1, 2), q(1, 2))
        .unwrap();
    assert_eq!(chamfered.start(), &p(0, 1));
    assert_eq!(chamfered.end(), chamfered.start());
    assert_eq!(chamfered.curves()[0].end(), &p(1, 0));

    let filleted = path
        .fillet_vertex_by_parameters(0, q(1, 2), q(1, 2), &p(1, 1), false)
        .unwrap();
    assert_eq!(filleted.start(), &p(0, 1));
    assert_eq!(filleted.end(), filleted.start());
    assert_eq!(filleted.curves()[0].family(), CurveFamily2::CircularArc);
    assert_eq!(filleted.curves()[0].end(), &p(1, 0));
    CurveRegion2::try_from_boundary_paths(&[filleted], &CurveContext::STRICT).unwrap();
}
#[test]
fn mixed_curve_path_corner_edits_reject_invalid_parameters_and_tangency() {
    let path = CurvePath2::try_new(vec![
        linear_family_curve(CurveFamily2::Line, false),
        linear_family_curve(CurveFamily2::Line, true),
    ])
    .unwrap();

    let boundary = path
        .chamfer_vertex_by_parameters(1, r(1), q(1, 2))
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
        .fillet_vertex_by_parameters(1, q(1, 2), q(1, 2), &p(0, 0), false)
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
        .chamfer_vertex_by_parameters(0, q(1, 2), q(1, 2))
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
            )
            .unwrap();
        prop_assert_eq!(chamfered.curves()[0].end(), &p(-radius, 0));
        prop_assert_eq!(chamfered.curves()[1].end(), &p(0, radius));

        let filleted = path
            .fillet_vertex_by_parameters(
                1,
                previous_parameter,
                next_parameter,
                &p(-radius, radius),
                false,
            )
            .unwrap();
        prop_assert_eq!(filleted.curves()[0].end(), &p(-radius, 0));
        prop_assert_eq!(filleted.curves()[1].family(), CurveFamily2::CircularArc);
        prop_assert_eq!(filleted.curves()[1].end(), &p(0, radius));
        prop_assert_eq!(filleted.curves()[2].start(), &p(0, radius));
    }
}
