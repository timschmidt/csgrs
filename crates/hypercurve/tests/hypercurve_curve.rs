use hypercurve::{
    BezierSubcurve2, BooleanOp, CircularArc2, Classification, CubicBezier2, Curve2,
    CurveBoundaryInteriorSide2, CurveError, CurveFamily2, CurveGeometry2, CurveOperation2,
    CurvePath2, CurvePolicy, CurveRegion2, CurveSource2, ExactCurveError, LineSeg2, Point2,
    QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2, Real, RegionPointLocation,
    Similarity2,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
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
            Some(CurveSource2::new(70)),
        )
        .unwrap(),
        Curve2::try_nurbs(
            2,
            vec![p(14, 0), p(15, 2), p(16, 0)],
            vec![r(1), r(2), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            Some(CurveSource2::new(71)),
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
fn top_level_curve_region_accepts_every_public_family_with_provenance() {
    let path = every_family_closed_path();

    let region = CurveRegion2::try_from_boundary_paths(&[path]).unwrap();

    assert_eq!(region.boundary_loops().len(), 1);
    let provenance = region
        .fragment_provenance()
        .expect("direct curved region retains authored lineage");
    for family in [
        CurveFamily2::Line,
        CurveFamily2::CircularArc,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
    ] {
        assert!(provenance.iter().any(|source| source.family() == family));
    }
    assert!(provenance.iter().all(|source| {
        source.operand().is_none() && source.source_path_index() == 0 && !source.reversed()
    }));
}

#[test]
fn top_level_curve_region_classifies_points_and_shares_native_boundary_cache() {
    let region = CurveRegion2::try_from_boundary_paths(&[every_family_closed_path()]).unwrap();
    let clone = region.clone();
    assert!(!region.is_native_boundary_cache_cached());
    assert!(!region.is_signed_area_cached());
    assert_eq!(region.signed_area(), Ok(None));
    assert!(region.is_signed_area_cached());
    assert!(clone.is_signed_area_cached());
    assert_eq!(
        region.classify_point(&p(8, -1), &CurvePolicy::certified()),
        Ok(Classification::Decided(RegionPointLocation::Inside))
    );
    assert!(region.is_native_boundary_cache_cached());
    assert!(clone.is_native_boundary_cache_cached());
    assert_eq!(
        clone.classify_point(&p(8, -4), &CurvePolicy::certified()),
        Ok(Classification::Decided(RegionPointLocation::Outside))
    );
    assert_eq!(
        clone.classify_point(&p(0, 0), &CurvePolicy::certified()),
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
    let bounded = CurveRegion2::try_from_boundary_paths(&[square]).unwrap();
    let bounded_clone = bounded.clone();
    assert!(!bounded.is_native_boundary_bounds_cache_cached());
    assert_eq!(
        bounded.classify_point(&p(1, 1), &CurvePolicy::certified()),
        Ok(Classification::Decided(RegionPointLocation::Inside))
    );
    assert!(bounded.is_native_boundary_bounds_cache_cached());
    assert!(bounded_clone.is_native_boundary_bounds_cache_cached());
}

#[test]
fn top_level_curve_region_rejects_open_boundary_paths_with_context() {
    let path = CurvePath2::try_new(vec![Curve2::from(
        LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap(),
    )])
    .unwrap();

    let error = CurveRegion2::try_from_boundary_paths(&[path]).unwrap_err();

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
fn identical_all_family_paths_boolean_without_losing_family_provenance() {
    let path = every_family_closed_path();
    let policy = CurvePolicy::certified();
    let prepared = path.try_prepare_intersection(&path, &policy).unwrap();
    let report = prepared.report_view().unwrap();
    assert!(report.is_complete(), "{:#?}", report.blockers());

    let selection = prepared
        .boolean_selection_view(
            BooleanOp::Union,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
        )
        .unwrap_or_else(|error| panic!("selection: {error:?}"));
    selection
        .arrangement_graph_view()
        .unwrap_or_else(|error| panic!("arrangement: {error:?}"));
    selection
        .traversal_view()
        .unwrap_or_else(|error| panic!("traversal: {error:?}"));
    let union = selection
        .region_view()
        .unwrap_or_else(|error| panic!("region: {error:?}"));
    let provenance = union.fragment_provenance().unwrap();
    for family in [
        CurveFamily2::Line,
        CurveFamily2::CircularArc,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
    ] {
        assert!(provenance.iter().any(|source| source.family() == family));
    }
    assert!(provenance.iter().all(|source| source.operand().is_some()));

    let difference = path
        .boolean_region(
            &path,
            BooleanOp::Difference,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        )
        .unwrap();
    assert!(difference.boundary_loops().is_empty());
    assert_eq!(difference.fragment_provenance(), Some([].as_slice()));
}

#[test]
fn top_level_spline_constructors_need_no_policy_and_retain_source() {
    let polynomial_source = CurveSource2::with_version(7, 2);
    let polynomial = Curve2::try_polynomial_bspline(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        Some(polynomial_source),
    )
    .unwrap();
    let nurbs_source = CurveSource2::with_version(8, 3);
    let nurbs = Curve2::try_nurbs(
        2,
        vec![p(2, 0), p(3, 2), p(4, 0)],
        vec![r(1), r(2), r(1)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        Some(nurbs_source),
    )
    .unwrap();

    assert_eq!(polynomial.family(), CurveFamily2::PolynomialBSpline);
    assert_eq!(polynomial.source(), Some(polynomial_source));
    assert_eq!(nurbs.family(), CurveFamily2::Nurbs);
    assert_eq!(nurbs.source(), Some(nurbs_source));
    assert_eq!(polynomial.end(), nurbs.start());
}

#[test]
fn mixed_curve_path_is_borrowed_and_preserves_per_curve_provenance() {
    let first_source = CurveSource2::new(11);
    let first = Curve2::with_source(
        CurveGeometry2::Line(LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap()),
        first_source,
    )
    .unwrap();
    let second_source = CurveSource2::new(12);
    let second = Curve2::with_source(
        CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(p(1, 0), p(2, 1), p(3, 0))),
        second_source,
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![first, second]).unwrap();
    let view = path.as_view();
    let viewed = view.iter().collect::<Vec<_>>();

    assert_eq!(view.start(), &p(0, 0));
    assert_eq!(view.end(), &p(3, 0));
    assert_eq!(viewed[0].source(), Some(first_source));
    assert_eq!(viewed[1].source(), Some(second_source));
    assert!(std::ptr::eq(viewed[0].curve(), &path.curves()[0]));
}

#[test]
fn reversed_curve_path_preserves_connectivity_and_per_curve_provenance() {
    let first_source = CurveSource2::new(13);
    let second_source = CurveSource2::new(14);
    let path = CurvePath2::try_new(vec![
        Curve2::with_source(
            CurveGeometry2::Line(LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap()),
            first_source,
        )
        .unwrap(),
        Curve2::with_source(
            CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(p(1, 0), p(2, 1), p(3, 0))),
            second_source,
        )
        .unwrap(),
    ])
    .unwrap();

    let reversed = path.as_view().reversed().unwrap();

    assert_eq!(reversed.start(), path.end());
    assert_eq!(reversed.end(), path.start());
    assert_eq!(reversed.curves()[0].source(), Some(second_source));
    assert_eq!(reversed.curves()[1].source(), Some(first_source));
    assert_eq!(reversed.reversed().unwrap(), path);
}

#[test]
fn disconnected_curve_path_names_failing_family_and_source() {
    let source = CurveSource2::with_version(22, 4);
    let first = Curve2::from(LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap());
    let second = Curve2::with_source(
        CurveGeometry2::CubicBezier(CubicBezier2::new(p(2, 0), p(3, 1), p(4, 1), p(5, 0))),
        source,
    )
    .unwrap();

    let error = CurvePath2::try_new(vec![first, second]).unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::Construction);
    assert_eq!(error.family(), CurveFamily2::CubicBezier);
    assert_eq!(error.source(), Some(source));
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: CurveError::DisconnectedCurvePath,
            ..
        }
    ));
}

#[test]
fn top_level_curve_rejects_conflicting_nested_nurbs_source() {
    let retained = hypercurve::NurbsCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(1, 1), p(2, 0)],
        vec![r(1), r(1), r(1)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        CurveSource2::new(1),
    )
    .unwrap();

    let error =
        Curve2::with_source(CurveGeometry2::Nurbs(retained), CurveSource2::new(2)).unwrap_err();

    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: CurveError::ConflictingCurveSource,
            ..
        }
    ));
}

#[test]
fn top_level_curve_rejects_conflicting_nested_polynomial_source() {
    let retained = hypercurve::PolynomialSplineCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(1, 1), p(2, 0)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        CurveSource2::new(3),
    )
    .unwrap();

    let error = Curve2::with_source(
        CurveGeometry2::PolynomialBSpline(retained),
        CurveSource2::new(4),
    )
    .unwrap_err();

    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: CurveError::ConflictingCurveSource,
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
        None,
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
fn top_level_curve_derivatives_preserve_parameter_domains_and_share_evaluators() {
    let half = (r(1) / r(2)).unwrap();
    let line = Curve2::from(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap());
    let line_clone = line.clone();
    assert!(!line.is_rational_evaluator_cache_cached());
    let line_derivative = line.as_view().derivative_at(&half).unwrap();
    assert_eq!(line_derivative.dx(), &r(2));
    assert_eq!(line_derivative.dy(), &r(0));
    assert!(line_clone.is_rational_evaluator_cache_cached());

    let quadratic = Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0)));
    let quadratic_derivative = quadratic.derivative_at(&half).unwrap();
    assert_eq!(quadratic_derivative.dx(), &r(2));
    assert_eq!(quadratic_derivative.dy(), &r(0));

    let spline = Curve2::try_polynomial_bspline(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        None,
    )
    .unwrap();
    assert!(!spline.is_rational_evaluator_cache_cached());
    let CurveGeometry2::PolynomialBSpline(retained_spline) = spline.geometry() else {
        panic!("top-level polynomial constructor returned another family");
    };
    assert!(!retained_spline.is_rational_span_cache_cached());
    let spline_derivative = spline.derivative_at(&r(1)).unwrap();
    assert_eq!(spline_derivative.dx(), &r(1));
    assert_eq!(spline_derivative.dy(), &r(0));
    assert!(retained_spline.is_rational_span_cache_cached());
    assert!(!spline.is_rational_evaluator_cache_cached());
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
fn top_level_curve_evaluation_reports_domain_and_capability_context() {
    let line_source = CurveSource2::new(51);
    let line = Curve2::with_source(
        CurveGeometry2::Line(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap()),
        line_source,
    )
    .unwrap();
    let domain_error = line.point_at(&r(2)).unwrap_err();
    assert_eq!(domain_error.operation(), CurveOperation2::Evaluation);
    assert_eq!(domain_error.family(), CurveFamily2::Line);
    assert_eq!(domain_error.source(), Some(line_source));
    assert!(matches!(
        domain_error,
        ExactCurveError::Invalid {
            cause: CurveError::InvalidCurveParameter,
            ..
        }
    ));

    let arc_source = CurveSource2::new(52);
    let arc = Curve2::with_source(
        CurveGeometry2::CircularArc(CircularArc2::from_bulge(p(0, 0), p(2, 0), r(1)).unwrap()),
        arc_source,
    )
    .unwrap();
    assert_eq!(arc.point_at(&(r(1) / r(2)).unwrap()).unwrap(), p(1, -1));
}

#[test]
fn top_level_reversal_preserves_source_and_negates_parameter_direction() {
    let source = CurveSource2::with_version(53, 2);
    let curve = Curve2::with_source(
        CurveGeometry2::Line(LineSeg2::try_new(p(0, 0), p(4, 2)).unwrap()),
        source,
    )
    .unwrap();
    let reversed = curve.as_view().reversed().unwrap();

    assert_eq!(reversed.source(), Some(source));
    assert_eq!(reversed.start(), curve.end());
    assert_eq!(reversed.end(), curve.start());
    let derivative = reversed.derivative_at(&(r(1) / r(2)).unwrap()).unwrap();
    assert_eq!((derivative.dx(), derivative.dy()), (&r(-4), &r(-2)));
    assert_eq!(
        reversed.native_bezier_fragments().unwrap()[0]
            .provenance()
            .source_parameter_range(),
        (&r(1), &r(0))
    );
    assert_eq!(reversed.reversed().unwrap().geometry(), curve.geometry());
}

#[test]
fn top_level_split_preserves_native_families_sources_and_exact_join_points() {
    let source = CurveSource2::with_version(53, 4);
    let curves = vec![
        CurveGeometry2::Line(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
        CurveGeometry2::CircularArc(CircularArc2::from_bulge(p(0, 0), p(2, 0), r(1)).unwrap()),
        CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0))),
        CurveGeometry2::CubicBezier(CubicBezier2::new(p(0, 0), p(1, 3), p(3, 3), p(4, 0))),
        CurveGeometry2::RationalQuadraticBezier(
            RationalQuadraticBezier2::try_new(p(0, 0), p(2, 4), p(4, 0), r(1), r(2), r(1)).unwrap(),
        ),
        CurveGeometry2::RationalBezier(
            RationalBezier2::try_new(
                vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
                vec![r(1), r(2), r(3), r(4)],
            )
            .unwrap(),
        ),
    ];
    let half = (r(1) / r(2)).unwrap();

    for geometry in curves {
        let curve = Curve2::with_source(geometry, source).unwrap();
        let join = curve.point_at(&half).unwrap();
        let (left, right) = curve.as_view().split_at(half.clone()).unwrap();

        assert_eq!(left.family(), curve.family());
        assert_eq!(right.family(), curve.family());
        assert_eq!(left.source(), Some(source));
        assert_eq!(right.source(), Some(source));
        assert_eq!(left.start(), curve.start());
        assert_eq!(left.end(), &join);
        assert_eq!(right.start(), &join);
        assert_eq!(right.end(), curve.end());
        let left_fragments = left.native_bezier_fragments().unwrap();
        let right_fragments = right.native_bezier_fragments().unwrap();
        assert_eq!(
            left_fragments
                .first()
                .unwrap()
                .provenance()
                .source_parameter_range()
                .0,
            &r(0)
        );
        assert_eq!(
            left_fragments
                .last()
                .unwrap()
                .provenance()
                .source_parameter_range()
                .1,
            &half
        );
        assert_eq!(
            right_fragments
                .first()
                .unwrap()
                .provenance()
                .source_parameter_range()
                .0,
            &half
        );
        assert_eq!(
            right_fragments
                .last()
                .unwrap()
                .provenance()
                .source_parameter_range()
                .1,
            &r(1)
        );
    }
}

#[test]
fn top_level_spline_split_retains_nonclamped_authored_domains() {
    let polynomial_source = CurveSource2::new(57);
    let polynomial = Curve2::try_polynomial_bspline(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(0), r(1), r(2), r(3), r(4), r(5), r(6)],
        Some(polynomial_source),
    )
    .unwrap();
    let nurbs_source = CurveSource2::new(58);
    let nurbs = Curve2::try_nurbs(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(2), r(3), r(4)],
        vec![r(0), r(1), r(2), r(3), r(4), r(5), r(6)],
        Some(nurbs_source),
    )
    .unwrap();

    for (curve, source) in [(polynomial, polynomial_source), (nurbs, nurbs_source)] {
        let join = curve.point_at(&r(3)).unwrap();
        let (left, right) = curve.split_at(r(3)).unwrap();

        assert_eq!(left.source(), Some(source));
        assert_eq!(right.source(), Some(source));
        assert_eq!(left.parameter_domain().start(), &r(2));
        assert_eq!(left.parameter_domain().end(), &r(3));
        assert_eq!(right.parameter_domain().start(), &r(3));
        assert_eq!(right.parameter_domain().end(), &r(4));
        assert_eq!(left.end(), &join);
        assert_eq!(right.start(), &join);
    }
}

#[test]
fn top_level_subcurve_preserves_arc_support_and_full_domain_caches() {
    let source = CurveSource2::new(59);
    let curve = Curve2::with_source(
        CurveGeometry2::CircularArc(CircularArc2::from_bulge(p(0, 0), p(2, 0), r(1)).unwrap()),
        source,
    )
    .unwrap();
    let original_fragments = curve.native_bezier_fragments().unwrap();
    let full = curve.subcurve(r(0), r(1)).unwrap();
    assert!(std::ptr::eq(
        original_fragments.as_ptr(),
        full.native_bezier_fragments().unwrap().as_ptr()
    ));

    let quarter = (r(1) / r(4)).unwrap();
    let three_quarters = (r(3) / r(4)).unwrap();
    let trimmed = curve
        .subcurve(quarter.clone(), three_quarters.clone())
        .unwrap();
    let CurveGeometry2::CircularArc(trimmed_arc) = trimmed.geometry() else {
        panic!("trimmed circular arc changed family");
    };
    let CurveGeometry2::CircularArc(original_arc) = curve.geometry() else {
        unreachable!();
    };
    assert_eq!(trimmed.source(), Some(source));
    assert_eq!(trimmed_arc.center(), original_arc.center());
    assert_eq!(
        trimmed_arc.radius_squared_ref(),
        original_arc.radius_squared_ref()
    );
    assert_eq!(trimmed.start(), &curve.point_at(&quarter).unwrap());
    assert_eq!(trimmed.end(), &curve.point_at(&three_quarters).unwrap());
    let trimmed_fragments = trimmed.native_bezier_fragments().unwrap();
    assert_eq!(
        trimmed_fragments
            .first()
            .unwrap()
            .provenance()
            .source_parameter_range()
            .0,
        &quarter
    );
    assert_eq!(
        trimmed_fragments
            .last()
            .unwrap()
            .provenance()
            .source_parameter_range()
            .1,
        &three_quarters
    );
}

#[test]
fn nested_top_level_trims_retain_root_parameter_provenance() {
    let source = CurveSource2::new(60);
    let curve = Curve2::with_source(
        CurveGeometry2::CubicBezier(CubicBezier2::new(p(0, 0), p(1, 3), p(3, 3), p(4, 0))),
        source,
    )
    .unwrap();
    let quarter = (r(1) / r(4)).unwrap();
    let three_quarters = (r(3) / r(4)).unwrap();
    let trimmed = curve
        .subcurve(quarter.clone(), three_quarters.clone())
        .unwrap();
    let trimmed_fragments = trimmed.native_bezier_fragments().unwrap();
    assert_eq!(
        trimmed_fragments[0].provenance().source_parameter_range(),
        (&quarter, &three_quarters)
    );
    assert_eq!(
        trimmed_fragments[0].provenance().parameter_range(),
        (&r(0), &r(1))
    );
    let nested = trimmed.subcurve((r(1) / r(2)).unwrap(), r(1)).unwrap();
    let nested_fragments = nested.native_bezier_fragments().unwrap();
    assert_eq!(
        nested_fragments
            .first()
            .unwrap()
            .provenance()
            .source_parameter_range()
            .0,
        &(r(1) / r(2)).unwrap()
    );
    assert_eq!(
        nested_fragments
            .last()
            .unwrap()
            .provenance()
            .source_parameter_range()
            .1,
        &three_quarters
    );
    let reversed = nested.reversed().unwrap();
    assert_eq!(
        reversed.native_bezier_fragments().unwrap()[0]
            .provenance()
            .source_parameter_range(),
        (&three_quarters, &(r(1) / r(2)).unwrap())
    );
}

#[test]
fn top_level_subdivision_rejects_invalid_ranges_with_context() {
    let source = CurveSource2::new(61);
    let curve = Curve2::with_source(
        CurveGeometry2::Line(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
        source,
    )
    .unwrap();

    for error in [
        curve.split_at(r(0)).unwrap_err(),
        curve.subcurve(r(1), r(0)).unwrap_err(),
        curve.subcurve(r(-1), r(1)).unwrap_err(),
    ] {
        assert_eq!(error.operation(), CurveOperation2::Subdivision);
        assert_eq!(error.family(), CurveFamily2::Line);
        assert_eq!(error.source(), Some(source));
        assert!(matches!(
            error,
            ExactCurveError::Invalid {
                cause: CurveError::InvalidCurveParameter,
                ..
            }
        ));
    }
}

#[test]
fn top_level_similarity_transform_preserves_all_curve_families_and_domains() {
    let transform =
        Similarity2::try_from_real_affine(r(0), r(-1), r(1), r(0), r(10), r(20)).unwrap();
    let source = CurveSource2::with_version(54, 3);
    let curves = vec![
        Curve2::from(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap()),
        Curve2::from(CircularArc2::from_bulge(p(0, 0), p(2, 0), r(1)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0))),
        Curve2::from(CubicBezier2::new(p(0, 0), p(1, 2), p(2, 2), p(3, 0))),
        Curve2::from(
            RationalQuadraticBezier2::try_new(p(0, 0), p(1, 2), p(2, 0), r(1), r(2), r(1)).unwrap(),
        ),
        Curve2::from(
            RationalBezier2::try_new(
                vec![p(0, 0), p(1, 2), p(2, 2), p(3, 0)],
                vec![r(1), r(2), r(3), r(4)],
            )
            .unwrap(),
        ),
        Curve2::try_polynomial_bspline(
            2,
            vec![p(0, 0), p(1, 2), p(2, 0)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            None,
        )
        .unwrap(),
        Curve2::try_nurbs(
            2,
            vec![p(0, 0), p(1, 2), p(2, 0)],
            vec![r(1), r(2), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            Some(source),
        )
        .unwrap(),
    ];
    let half = (r(1) / r(2)).unwrap();

    for curve in curves {
        let transformed = curve.transform_similarity(&transform).unwrap();
        assert_eq!(transformed.family(), curve.family());
        assert_eq!(transformed.source(), curve.source());
        assert_eq!(transformed.parameter_domain(), curve.parameter_domain());
        assert_eq!(
            transformed.start(),
            &transform.transform_point(curve.start())
        );
        assert_eq!(transformed.end(), &transform.transform_point(curve.end()));
        assert_eq!(
            transformed.point_at(&half).unwrap(),
            transform.transform_point(&curve.point_at(&half).unwrap())
        );
    }
}

#[test]
fn top_level_periodic_splines_wrap_transform_and_trim_without_losing_provenance() {
    let transform = Similarity2::try_from_real_affine(r(0), r(-1), r(1), r(0), r(5), r(7)).unwrap();
    let polynomial_source = CurveSource2::with_version(60, 1);
    let nurbs_source = CurveSource2::with_version(61, 2);
    let controls = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)];
    let breaks = (0..=4).map(r).collect::<Vec<_>>();
    let curves = [
        Curve2::try_periodic_polynomial_bspline(
            2,
            controls.clone(),
            breaks.clone(),
            Some(polynomial_source),
        )
        .unwrap(),
        Curve2::try_periodic_nurbs(
            2,
            controls,
            vec![r(1), r(2), r(3), r(4)],
            breaks,
            Some(nurbs_source),
        )
        .unwrap(),
    ];

    for curve in curves {
        assert!(curve.as_view().is_periodic());
        assert_eq!(curve.as_view().period(), Some(&r(4)));
        assert_eq!(curve.start(), curve.end());
        assert_eq!(
            curve.as_view().point_at_wrapped(&r(5)).unwrap(),
            curve.point_at(&r(1)).unwrap()
        );

        let transformed = curve.transform_similarity(&transform).unwrap();
        assert_eq!(transformed.period(), curve.period());
        assert_eq!(transformed.source(), curve.source());
        assert_eq!(
            transformed.point_at_wrapped(&r(5)).unwrap(),
            transform.transform_point(&curve.point_at(&r(1)).unwrap())
        );

        let full = curve.subcurve(r(0), r(4)).unwrap();
        assert!(full.is_periodic());
        let partial = curve.subcurve(r(1), r(3)).unwrap();
        assert!(!partial.is_periodic());
        assert_eq!(partial.source(), curve.source());
        assert_eq!(partial.start(), &curve.point_at(&r(1)).unwrap());
        assert_eq!(partial.end(), &curve.point_at(&r(3)).unwrap());
    }
}

#[test]
fn path_similarity_transform_preserves_connectivity_and_sources() {
    let transform = Similarity2::try_from_real_affine(r(0), r(-1), r(1), r(0), r(5), r(7)).unwrap();
    let first_source = CurveSource2::new(55);
    let second_source = CurveSource2::new(56);
    let path = CurvePath2::try_new(vec![
        Curve2::with_source(
            CurveGeometry2::Line(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap()),
            first_source,
        )
        .unwrap(),
        Curve2::with_source(
            CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(p(2, 0), p(3, 1), p(4, 0))),
            second_source,
        )
        .unwrap(),
    ])
    .unwrap();

    let transformed = path.as_view().transform_similarity(&transform).unwrap();

    assert_eq!(
        transformed.start(),
        &transform.transform_point(path.start())
    );
    assert_eq!(transformed.end(), &transform.transform_point(path.end()));
    assert_eq!(transformed.curves()[0].source(), Some(first_source));
    assert_eq!(transformed.curves()[1].source(), Some(second_source));
    assert_eq!(
        transformed.curves()[0].end(),
        transformed.curves()[1].start()
    );
}

#[test]
fn native_bezier_promotion_is_cached_and_preserves_path_provenance() {
    let first_source = CurveSource2::new(61);
    let first = Curve2::with_source(
        CurveGeometry2::Line(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap()),
        first_source,
    )
    .unwrap();
    let first_clone = first.clone();
    let first_fragments = first.native_bezier_fragments().unwrap();
    assert!(std::ptr::eq(
        first_fragments,
        first_clone.native_bezier_fragments().unwrap()
    ));
    assert!(matches!(
        first_fragments[0].curve(),
        hypercurve::BezierSubcurve2::Quadratic(_)
    ));
    assert_eq!(first_fragments[0].provenance().source(), Some(first_source));
    assert_eq!(
        first_fragments[0].provenance().parameter_range(),
        (&r(0), &r(1))
    );

    let second_source = CurveSource2::new(62);
    let second = Curve2::with_source(
        CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(p(2, 0), p(2, 2), p(0, 2))),
        second_source,
    )
    .unwrap();
    let third_source = CurveSource2::new(63);
    let third = Curve2::with_source(
        CurveGeometry2::Line(LineSeg2::try_new(p(0, 2), p(0, 0)).unwrap()),
        third_source,
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![first, second, third]).unwrap();
    let path_clone = path.clone();
    assert!(!path.is_native_bezier_fragments_cached());
    assert!(!path.is_bezier_boundary_loop_cached());
    let promoted = path.bezier_boundary_loop().unwrap();

    assert!(path.is_native_bezier_fragments_cached());
    assert!(path_clone.is_native_bezier_fragments_cached());
    assert!(path.is_bezier_boundary_loop_cached());
    assert!(path_clone.is_bezier_boundary_loop_cached());
    assert!(std::ptr::eq(
        path.native_bezier_fragments().unwrap(),
        path_clone.native_bezier_fragments().unwrap()
    ));
    assert!(std::ptr::eq(
        promoted,
        path_clone.bezier_boundary_loop().unwrap()
    ));
    assert_eq!(promoted.boundary_loop().fragments().len(), 3);
    assert_eq!(promoted.fragments().len(), 3);
    assert_eq!(
        promoted
            .fragments()
            .iter()
            .map(|fragment| fragment.provenance().source())
            .collect::<Vec<_>>(),
        vec![Some(first_source), Some(second_source), Some(third_source)]
    );
}

#[test]
fn spline_promotion_retains_source_span_ranges_for_all_native_families() {
    let source = CurveSource2::with_version(64, 5);
    let polynomial = Curve2::try_polynomial_bspline(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        Some(source),
    )
    .unwrap();
    let fragments = polynomial.native_bezier_fragments().unwrap();
    assert_eq!(fragments.len(), 2);
    assert_eq!(fragments[0].provenance().source(), Some(source));
    assert_eq!(fragments[0].provenance().source_span_index(), Some(0));
    assert_eq!(fragments[1].provenance().source_span_index(), Some(1));
    assert_eq!(fragments[1].provenance().parameter_range(), (&r(1), &r(2)));

    let nurbs_source = CurveSource2::new(65);
    let nurbs = Curve2::try_nurbs(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(1), r(2), r(4), r(8), r(16)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        Some(nurbs_source),
    )
    .unwrap();
    let fragments = nurbs.native_bezier_fragments().unwrap();
    assert_eq!(fragments.len(), 2);
    assert!(
        fragments
            .iter()
            .all(|fragment| matches!(fragment.curve(), BezierSubcurve2::Rational(_)))
    );
    assert_eq!(fragments[0].provenance().source(), Some(nurbs_source));
    assert_eq!(fragments[0].provenance().source_span_index(), Some(0));
    assert_eq!(fragments[0].provenance().parameter_range(), (&r(0), &r(1)));
    assert_eq!(fragments[1].provenance().source_span_index(), Some(1));
    assert_eq!(fragments[1].provenance().parameter_range(), (&r(1), &r(2)));
}
