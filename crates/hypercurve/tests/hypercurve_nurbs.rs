use hypercurve::{
    BezierSubcurve2, Curve2, CurveError, CurveFamily2, CurveOperation2, CurveParameterSide2,
    CurvePolicy, CurveSource2, ExactCurveError, NurbsCurve2, Point2, Real, SplinePeriodicity2,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn quadratic_nurbs(source: CurveSource2) -> NurbsCurve2 {
    NurbsCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(2), r(4), r(1)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        source,
    )
    .unwrap()
}

#[test]
fn linear_nurbs_evaluates_and_promotes_with_source_provenance() {
    let source = CurveSource2::with_version(40, 2);
    let curve = NurbsCurve2::try_new_with_source(
        1,
        vec![p(0, 0), p(4, 0)],
        vec![r(1), r(3)],
        vec![r(0), r(0), r(1), r(1)],
        source,
    )
    .unwrap();
    let half = (r(1) / r(2)).unwrap();

    assert_eq!(curve.degree(), 1);
    assert_eq!(curve.parameter_domain(), (&r(0), &r(1)));
    assert!(!curve.is_rational_span_cache_cached());
    assert_eq!(curve.point_at(&half).unwrap(), p(3, 0));
    assert!(curve.is_rational_span_cache_cached());
    let derivative = curve.derivative_at(&half).unwrap();
    assert_eq!(derivative.dx(), &r(3));
    assert_eq!(derivative.dy(), &r(0));
    assert!(curve.is_rational_span_cache_cached());
    let spans = curve.native_spans().unwrap().collect::<Vec<_>>();
    assert_eq!(spans.len(), 1);
    assert_eq!(spans[0].source_span().source(), Some(source));
    assert_eq!(spans[0].source_span().knot_interval(), (&r(0), &r(1)));
    assert!(matches!(
        spans[0].curve(),
        BezierSubcurve2::RationalQuadratic(_)
    ));

    let top_level = Curve2::from(curve);
    let fragments = top_level.native_bezier_fragments().unwrap();
    assert_eq!(fragments.len(), 1);
    assert_eq!(fragments[0].provenance().source(), Some(source));
    assert_eq!(fragments[0].provenance().source_span_index(), Some(0));
    assert_eq!(fragments[0].provenance().parameter_range(), (&r(0), &r(1)));
}

#[test]
fn nurbs_derivative_uses_authored_knot_parameter_and_shared_span_cache() {
    let curve = NurbsCurve2::try_new_with_source(
        1,
        vec![p(0, 0), p(4, 8)],
        vec![r(1), r(1)],
        vec![r(2), r(2), r(6), r(6)],
        CurveSource2::new(42),
    )
    .unwrap();
    let clone = curve.clone();

    assert_eq!(curve.parameter_domain(), (&r(2), &r(6)));
    let derivative = curve.derivative_at(&r(3)).unwrap();
    assert_eq!(derivative.dx(), &r(1));
    assert_eq!(derivative.dy(), &r(2));
    assert!(clone.is_rational_span_cache_cached());
    assert_eq!(clone.derivative_at(&r(5)).unwrap(), derivative);
}

#[test]
fn nurbs_higher_derivatives_use_each_authored_parameter_chain_power() {
    let curve = NurbsCurve2::try_new(
        1,
        vec![p(0, 0), p(4, 0)],
        vec![r(1), r(3)],
        vec![r(2), r(2), r(6), r(6)],
    )
    .unwrap();

    let derivatives = curve.derivatives_at(&r(4), 3).unwrap();

    assert_eq!(derivatives.len(), 3);
    assert_eq!(
        (derivatives[0].dx(), derivatives[0].dy()),
        (&q(3, 4), &r(0))
    );
    assert_eq!(
        (derivatives[1].dx(), derivatives[1].dy()),
        (&q(-3, 8), &r(0))
    );
    assert_eq!(
        (derivatives[2].dx(), derivatives[2].dy()),
        (&q(9, 32), &r(0))
    );
}

#[test]
fn nurbs_internal_corner_requires_explicit_derivative_side() {
    let source = CurveSource2::new(43);
    let curve = NurbsCurve2::try_new_with_source(
        1,
        vec![p(0, 0), p(1, 0), p(1, 1)],
        vec![r(1), r(1), r(1)],
        vec![r(0), r(0), r(1), r(2), r(2)],
        source,
    )
    .unwrap();

    let error = curve.derivative_at(&r(1)).unwrap_err();
    assert!(matches!(
        error,
        ExactCurveError::Blocked(blocker)
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
    ));
    let left = curve
        .derivative_at_side(&r(1), CurveParameterSide2::Left)
        .unwrap();
    let right = curve
        .derivative_at_side(&r(1), CurveParameterSide2::Right)
        .unwrap();
    assert_eq!((left.dx(), left.dy()), (&r(1), &r(0)));
    assert_eq!((right.dx(), right.dy()), (&r(0), &r(1)));

    let top_level = Curve2::from(curve);
    assert!(top_level.derivative_at(&r(1)).is_err());
    assert_eq!(
        top_level
            .derivative_at_side(&r(1), CurveParameterSide2::Right)
            .unwrap(),
        right
    );
}

#[test]
fn discontinuous_nurbs_knot_requires_explicit_point_side() {
    let source = CurveSource2::new(49);
    let curve = NurbsCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(1, 1), p(2, 0), p(10, 0), p(11, 1), p(12, 0)],
        vec![r(1); 6],
        vec![r(0), r(0), r(0), r(1), r(1), r(1), r(2), r(2), r(2)],
        source,
    )
    .unwrap();

    assert!(matches!(
        curve.point_at(&r(1)),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
                && blocker.source() == Some(source)
    ));
    assert_eq!(
        curve
            .point_at_side(&r(1), CurveParameterSide2::Left)
            .unwrap(),
        p(2, 0)
    );
    assert_eq!(
        curve
            .point_at_side(&r(1), CurveParameterSide2::Right)
            .unwrap(),
        p(10, 0)
    );

    let (left, right) = curve.split_at(r(1)).unwrap();
    assert_eq!(left.parameter_domain(), (&r(0), &r(1)));
    assert_eq!(right.parameter_domain(), (&r(1), &r(2)));
    assert_eq!(left.end(), &p(2, 0));
    assert_eq!(right.start(), &p(10, 0));

    let top_level = Curve2::from(curve);
    assert_eq!(
        top_level
            .as_view()
            .point_at_side(&r(1), CurveParameterSide2::Right)
            .unwrap(),
        p(10, 0)
    );
}

#[test]
fn nurbs_knot_insertion_preserves_exact_image_source_and_full_multiplicity_cache() {
    let source = CurveSource2::with_version(44, 6);
    let curve = NurbsCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(2, 4), p(4, 0)],
        vec![r(1), r(2), r(1)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        source,
    )
    .unwrap();
    let samples = [r(0), (r(1) / r(2)).unwrap(), r(1), r(2)];
    let expected = samples
        .iter()
        .map(|parameter| curve.point_at(parameter).unwrap())
        .collect::<Vec<_>>();

    let once = curve.insert_knot(r(1)).unwrap();
    let twice = once.insert_knot(r(1)).unwrap();
    assert_eq!(once.source(), Some(source));
    assert_eq!(
        once.control_points().len(),
        curve.control_points().len() + 1
    );
    assert_eq!(
        twice.control_points().len(),
        curve.control_points().len() + 2
    );
    assert_eq!(
        twice.knots().iter().filter(|knot| **knot == r(1)).count(),
        2
    );
    assert_eq!(
        samples
            .iter()
            .map(|parameter| twice.point_at(parameter).unwrap())
            .collect::<Vec<_>>(),
        expected
    );

    let cached = twice.bezier_decomposition().unwrap();
    let no_op = twice.insert_knot(r(1)).unwrap();
    assert!(std::ptr::eq(cached, no_op.bezier_decomposition().unwrap()));
    assert_eq!(no_op, twice);
}

#[test]
fn nurbs_batch_knot_refinement_projects_once_and_reuses_clone_shared_result() {
    let source = CurveSource2::with_version(144, 3);
    let curve = NurbsCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(2, 4), p(4, 0)],
        vec![r(1), r(2), r(1)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        source,
    )
    .unwrap();
    let clone = curve.clone();
    let request = vec![r(1), r(1)];
    assert!(!curve.is_knot_refinement_cached(&request));

    let batch = curve.insert_knots(request.clone()).unwrap();
    let sequential = curve.insert_knot(r(1)).unwrap().insert_knot(r(1)).unwrap();
    assert_eq!(batch, sequential);
    assert_eq!(batch.source(), Some(source));
    assert!(clone.is_knot_refinement_cached(&request));
    for parameter in [r(0), q(1, 2), r(1), q(3, 2), r(2)] {
        assert_eq!(batch.point_at(&parameter), curve.point_at(&parameter));
    }

    let retained = batch.bezier_decomposition().unwrap();
    let replay = clone.insert_knots(request).unwrap();
    assert!(std::ptr::eq(
        retained,
        replay.bezier_decomposition().unwrap()
    ));
}

#[test]
fn nurbs_batch_knot_refinement_retains_contextual_failure_without_mutating_source() {
    let source = CurveSource2::new(145);
    let curve = quadratic_nurbs(source);
    let source_control_count = curve.control_points().len();
    let request = vec![r(1), r(3)];

    let first = curve.insert_knots(request.clone()).unwrap_err();
    assert_eq!(first.operation(), CurveOperation2::KnotInsertion);
    assert_eq!(first.family(), CurveFamily2::Nurbs);
    assert_eq!(first.source(), Some(source));
    assert!(curve.is_knot_refinement_cached(&request));
    assert_eq!(curve.insert_knots(request).unwrap_err(), first);
    assert_eq!(curve.control_points().len(), source_control_count);
}

#[test]
fn nurbs_knot_removal_exactly_inverts_insertion_and_reuses_clone_shared_proof() {
    let source = CurveSource2::with_version(148, 5);
    let curve = NurbsCurve2::try_new_with_source(
        3,
        vec![p(0, 0), p(1, 4), p(4, 3), p(6, 0)],
        vec![r(1), r(2), r(5), r(3)],
        vec![r(0), r(0), r(0), r(0), r(2), r(2), r(2), r(2)],
        source,
    )
    .unwrap();
    let knot = q(3, 4);
    let refined = curve.insert_knot(knot.clone()).unwrap();
    let clone = refined.clone();
    assert!(!refined.is_knot_removal_cached(&knot));

    let removed = refined.remove_knot(knot.clone()).unwrap().unwrap();
    assert_eq!(removed.source(), Some(source));
    assert_eq!(removed.degree(), curve.degree());
    assert_eq!(removed.knots(), curve.knots());
    assert_eq!(removed.control_points(), curve.control_points());
    assert_eq!(removed.weights(), curve.weights());
    for parameter in [r(0), q(1, 4), q(3, 4), q(3, 2), r(2)] {
        assert_eq!(removed.point_at(&parameter), curve.point_at(&parameter));
    }

    assert!(clone.is_knot_removal_cached(&knot));
    let retained = removed.bezier_decomposition().unwrap();
    let replay = clone.remove_knot(knot).unwrap().unwrap();
    assert!(std::ptr::eq(
        retained,
        replay.bezier_decomposition().unwrap()
    ));
}

#[test]
fn nurbs_knot_removal_retains_exact_negative_result_and_contextual_domain_errors() {
    let source = CurveSource2::new(149);
    let curve = quadratic_nurbs(source);
    let clone = curve.clone();
    assert!(!curve.is_knot_removal_cached(&r(1)));
    assert!(curve.remove_knot(r(1)).unwrap().is_none());
    assert!(clone.is_knot_removal_cached(&r(1)));
    assert!(clone.remove_knot(r(1)).unwrap().is_none());

    for knot in [r(-1), r(0), r(2), r(3)] {
        let error = curve.remove_knot(knot).unwrap_err();
        assert_eq!(error.operation(), CurveOperation2::KnotRemoval);
        assert_eq!(error.family(), CurveFamily2::Nurbs);
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
fn periodic_nurbs_knot_removal_preserves_period_and_wrapped_image() {
    let source = CurveSource2::with_version(150, 2);
    let curve = NurbsCurve2::try_new_periodic_with_source(
        2,
        vec![p(0, 0), p(3, 0), p(4, 2), p(2, 5), p(-1, 2)],
        vec![r(1), r(2), r(5), r(3), r(4)],
        vec![r(0), r(1), r(1), r(3), r(5), r(8)],
        source,
    )
    .unwrap();
    let knot = q(5, 2);
    let refined = curve.insert_knot(knot.clone()).unwrap();
    let removed = refined.remove_knot(knot).unwrap().unwrap();

    assert_eq!(removed.period(), curve.period());
    assert_eq!(removed.source(), Some(source));
    assert_eq!(removed.start(), removed.end());
    for parameter in [r(-3), r(0), q(5, 2), r(7), r(13)] {
        assert_eq!(
            removed.point_at_wrapped(&parameter),
            curve.point_at_wrapped(&parameter)
        );
    }
}

#[test]
fn nurbs_degree_elevation_retains_exact_span_image_intervals_source_and_cache() {
    let source = CurveSource2::with_version(146, 8);
    let curve = NurbsCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        source,
    )
    .unwrap();
    let clone = curve.clone();
    assert!(!curve.is_degree_elevation_cached(4));

    let elevation = curve.degree_elevation(4).unwrap();
    assert_eq!(elevation.source_degree(), 2);
    assert_eq!(elevation.target_degree(), 4);
    assert_eq!(elevation.source(), Some(source));
    assert_eq!(elevation.spans().len(), 2);
    for span in elevation.spans() {
        assert_eq!(span.curve().degree(), 4);
        let (start, end) = span.parameter_interval();
        for local in [r(0), q(1, 2), r(1)] {
            let source_parameter = start + &local * (end - start);
            assert_eq!(
                span.curve()
                    .point_at(&local, &CurvePolicy::certified())
                    .unwrap(),
                curve
                    .point_at_side(
                        &source_parameter,
                        if local == r(0) {
                            CurveParameterSide2::Right
                        } else {
                            CurveParameterSide2::Left
                        },
                    )
                    .unwrap()
            );
        }
    }
    assert!(clone.is_degree_elevation_cached(4));
    let replay = clone.degree_elevation(4).unwrap();
    assert!(std::ptr::eq(
        elevation.spans().as_ptr(),
        replay.spans().as_ptr()
    ));
}

#[test]
fn nurbs_elevated_carrier_preserves_image_source_and_source_continuity() {
    let source = CurveSource2::with_version(151, 9);
    let curve = NurbsCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        source,
    )
    .unwrap();
    let clone = curve.clone();
    assert!(!curve.is_elevated_curve_cached(4));

    let elevated = curve.elevated_to_degree(4).unwrap();
    assert_eq!(elevated.degree(), 4);
    assert_eq!(elevated.source(), Some(source));
    assert_eq!(elevated.parameter_domain(), curve.parameter_domain());
    assert_eq!(
        elevated
            .knots()
            .iter()
            .filter(|knot| **knot == r(1))
            .count(),
        3
    );
    for parameter in [r(0), q(1, 4), q(3, 4), r(1), q(3, 2), r(2)] {
        assert_eq!(elevated.point_at(&parameter), curve.point_at(&parameter));
    }
    for parameter in [q(1, 2), r(1), q(3, 2)] {
        assert_eq!(
            elevated.derivative_at(&parameter),
            curve.derivative_at(&parameter)
        );
    }

    assert!(clone.is_elevated_curve_cached(4));
    let retained = elevated.bezier_decomposition().unwrap();
    let replay = clone.elevated_to_degree(4).unwrap();
    assert!(std::ptr::eq(
        retained,
        replay.bezier_decomposition().unwrap()
    ));
}

#[test]
fn nurbs_elevated_carrier_preserves_discontinuous_knot_sides() {
    let source = CurveSource2::new(152);
    let curve = NurbsCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(1, 1), p(2, 0), p(10, 0), p(11, 1), p(12, 0)],
        vec![r(1); 6],
        vec![r(0), r(0), r(0), r(1), r(1), r(1), r(2), r(2), r(2)],
        source,
    )
    .unwrap();
    let elevated = curve.elevated_to_degree(4).unwrap();

    assert_eq!(elevated.degree(), 4);
    assert_eq!(
        elevated
            .knots()
            .iter()
            .filter(|knot| **knot == r(1))
            .count(),
        5
    );
    assert_eq!(
        elevated.point_at_side(&r(1), CurveParameterSide2::Left),
        curve.point_at_side(&r(1), CurveParameterSide2::Left)
    );
    assert_eq!(
        elevated.point_at_side(&r(1), CurveParameterSide2::Right),
        curve.point_at_side(&r(1), CurveParameterSide2::Right)
    );
    assert!(matches!(
        elevated.point_at(&r(1)),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
    ));
}

#[test]
fn periodic_nurbs_elevated_carrier_preserves_wrapped_points_and_derivatives() {
    let source = CurveSource2::with_version(153, 4);
    let curve = NurbsCurve2::try_new_periodic_with_source(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        vec![r(1), r(2), r(3), r(4)],
        (0..=4).map(r).collect(),
        source,
    )
    .unwrap();
    let elevated = curve.elevated_to_degree(3).unwrap();

    assert_eq!(elevated.degree(), 3);
    assert_eq!(elevated.period(), curve.period());
    assert_eq!(elevated.source(), Some(source));
    assert_eq!(elevated.start(), elevated.end());
    for parameter in [r(-3), q(1, 2), q(7, 2), r(4), q(17, 2)] {
        assert_eq!(
            elevated.point_at_wrapped(&parameter),
            curve.point_at_wrapped(&parameter)
        );
        assert_eq!(
            elevated.derivative_at_wrapped(&parameter),
            curve.derivative_at_wrapped(&parameter)
        );
    }
}

#[test]
fn nurbs_degree_elevation_retains_contextual_invalid_target_and_projective_blocker() {
    let source = CurveSource2::new(147);
    let curve = quadratic_nurbs(source);
    let invalid = curve.degree_elevation(1).unwrap_err();
    assert_eq!(invalid.operation(), CurveOperation2::DegreeElevation);
    assert_eq!(invalid.family(), CurveFamily2::Nurbs);
    assert_eq!(invalid.source(), Some(source));

    let singular = NurbsCurve2::try_new_with_source(
        1,
        vec![p(0, 0), p(2, 0)],
        vec![r(1), r(-1)],
        vec![r(0), r(0), r(1), r(1)],
        source,
    )
    .unwrap();
    let blocked = singular.degree_elevation(2).unwrap_err();
    assert_eq!(blocked.operation(), CurveOperation2::DegreeElevation);
    assert_eq!(blocked.family(), CurveFamily2::Nurbs);
    assert_eq!(blocked.source(), Some(source));
    assert!(singular.is_degree_elevation_cached(2));
    assert_eq!(singular.degree_elevation(2).unwrap_err(), blocked);
}

#[test]
fn out_of_domain_nurbs_knot_insertion_has_contextual_error() {
    let source = CurveSource2::new(45);
    let curve = quadratic_nurbs(source);
    let error = curve.insert_knot(r(3)).unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::KnotInsertion);
    assert_eq!(error.family(), CurveFamily2::Nurbs);
    assert_eq!(error.source(), Some(source));
}

#[test]
fn nurbs_split_and_subcurve_preserve_authored_parameters_and_exact_image() {
    let source = CurveSource2::with_version(46, 2);
    let curve = NurbsCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(2, 4), p(4, 0)],
        vec![r(1), r(2), r(1)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        source,
    )
    .unwrap();

    let (left, right) = curve.split_at(r(1)).unwrap();
    assert_eq!(left.parameter_domain(), (&r(0), &r(1)));
    assert_eq!(right.parameter_domain(), (&r(1), &r(2)));
    assert_eq!(left.source(), Some(source));
    assert_eq!(right.source(), Some(source));
    assert_eq!(left.end(), right.start());
    assert_eq!(left.end(), &curve.point_at(&r(1)).unwrap());
    assert_eq!(
        left.point_at(&q(1, 2)).unwrap(),
        curve.point_at(&q(1, 2)).unwrap()
    );
    assert_eq!(
        right.point_at(&q(3, 2)).unwrap(),
        curve.point_at(&q(3, 2)).unwrap()
    );

    let middle = curve.subcurve(q(1, 2), q(3, 2)).unwrap();
    assert_eq!(middle.parameter_domain(), (&q(1, 2), &q(3, 2)));
    assert_eq!(middle.start(), &curve.point_at(&q(1, 2)).unwrap());
    assert_eq!(middle.end(), &curve.point_at(&q(3, 2)).unwrap());
    assert_eq!(
        middle.point_at(&r(1)).unwrap(),
        curve.point_at(&r(1)).unwrap()
    );
}

#[test]
fn nurbs_reversal_preserves_domain_source_and_exact_parameter_mapping() {
    let source = CurveSource2::with_version(48, 3);
    let curve = quadratic_nurbs(source);
    let reversed = curve.reversed().unwrap();

    assert_eq!(reversed.parameter_domain(), curve.parameter_domain());
    assert_eq!(reversed.source(), Some(source));
    assert_eq!(reversed.start(), curve.end());
    assert_eq!(reversed.end(), curve.start());
    assert_eq!(
        reversed.point_at(&q(1, 2)).unwrap(),
        curve.point_at(&q(3, 2)).unwrap()
    );
    let forward_derivative = curve.derivative_at(&q(3, 2)).unwrap();
    let reverse_derivative = reversed.derivative_at(&q(1, 2)).unwrap();
    assert_eq!(reverse_derivative.dx(), &(-forward_derivative.dx()));
    assert_eq!(reverse_derivative.dy(), &(-forward_derivative.dy()));
    assert_eq!(reversed.reversed().unwrap(), curve);
}

#[test]
fn invalid_nurbs_split_and_trim_ranges_report_subdivision_context() {
    let source = CurveSource2::new(47);
    let curve = quadratic_nurbs(source);
    for error in [
        curve.split_at(r(0)).unwrap_err(),
        curve.subcurve(r(1), r(1)).unwrap_err(),
        curve.subcurve(r(-1), r(1)).unwrap_err(),
    ] {
        assert_eq!(error.operation(), CurveOperation2::Subdivision);
        assert_eq!(error.family(), CurveFamily2::Nurbs);
        assert_eq!(error.source(), Some(source));
    }
}

#[test]
fn top_level_nurbs_retains_source_and_exact_geometry_without_policy() {
    let source = CurveSource2::with_version(41, 7);
    let curve = quadratic_nurbs(source);

    assert_eq!(curve.source(), Some(source));
    assert_eq!(curve.degree(), 2);
    assert_eq!(curve.start(), &p(0, 0));
    assert_eq!(curve.end(), &p(6, 0));
    assert_eq!(curve.control_points().len(), 4);
    assert_eq!(curve.weights(), &[r(1), r(2), r(4), r(1)]);
}

#[test]
fn nurbs_clones_share_one_retained_bezier_decomposition() {
    let curve = quadratic_nurbs(CurveSource2::new(3));
    let clone = curve.clone();
    assert!(!curve.is_bezier_decomposition_cached());

    let first = curve.bezier_decomposition().unwrap();
    let second = clone.bezier_decomposition().unwrap();

    assert!(curve.is_bezier_decomposition_cached());
    assert!(std::ptr::eq(first, second));
    assert_eq!(first.spans().len(), 2);
    assert_eq!(first.inserted_knot_count(), 1);
}

#[test]
fn native_nurbs_spans_are_cached_and_borrowed() {
    let source = CurveSource2::new(5);
    let curve = quadratic_nurbs(source);

    let first = curve.native_subcurves().unwrap();
    let first_ptr = first.as_ptr();
    let second = curve.native_subcurves().unwrap();

    assert_eq!(first_ptr, second.as_ptr());
    assert_eq!(first.len(), 2);
    assert!(
        first
            .iter()
            .all(|span| matches!(span, BezierSubcurve2::RationalQuadratic(_)))
    );

    let retained = curve.bezier_spans().unwrap().collect::<Vec<_>>();
    let promoted = curve.native_spans().unwrap().collect::<Vec<_>>();
    assert_eq!(retained.len(), 2);
    assert_eq!(retained[0].span_index(), 0);
    assert_eq!(retained[1].span_index(), 1);
    assert_eq!(retained[0].source(), Some(source));
    assert_eq!(retained[0].knot_interval(), (&r(0), &r(1)));
    assert_eq!(retained[1].knot_interval(), (&r(1), &r(2)));
    assert!(std::ptr::eq(
        retained[0].retained_span(),
        curve
            .bezier_decomposition()
            .unwrap()
            .spans()
            .first()
            .unwrap()
    ));
    assert_eq!(promoted[1].source_span().span_index(), 1);
    assert_eq!(promoted[1].source_span().source(), Some(source));
    assert!(std::ptr::eq(promoted[0].curve(), &first[0]));
}

#[test]
fn nurbs_evaluation_reuses_decomposition_and_preserves_exact_coordinates() {
    let curve = quadratic_nurbs(CurveSource2::new(8));

    assert_eq!(curve.point_at(&r(0)).unwrap(), p(0, 0));
    let join = curve.point_at(&r(1)).unwrap();
    assert_eq!(join.x(), &(Real::from(10) / Real::from(3)).unwrap());
    assert_eq!(join.y(), &r(4));
    assert_eq!(curve.point_at(&r(2)).unwrap(), p(6, 0));
    assert!(curve.is_bezier_decomposition_cached());
}

#[test]
fn out_of_domain_nurbs_evaluation_has_contextual_error() {
    let source = CurveSource2::with_version(13, 2);
    let curve = quadratic_nurbs(source);

    let error = curve.point_at(&r(3)).unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::Evaluation);
    assert_eq!(error.family(), CurveFamily2::Nurbs);
    assert_eq!(error.source(), Some(source));
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: hypercurve::CurveError::InvalidCurveParameter,
            ..
        }
    ));
}

#[test]
fn unequal_weight_cubic_nurbs_promotes_once_with_provenance() {
    let source = CurveSource2::with_version(99, 12);
    let curve = NurbsCurve2::try_new_with_source(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(1), r(2), r(4), r(8), r(16)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        source,
    )
    .unwrap();

    let first = curve.native_subcurves().unwrap();
    let first_pointer = first.as_ptr();
    let second = curve.native_subcurves().unwrap();
    assert_eq!(first_pointer, second.as_ptr());
    assert_eq!(first.len(), 2);
    assert!(
        first
            .iter()
            .all(|span| matches!(span, BezierSubcurve2::Rational(_)))
    );

    let spans = curve.native_spans().unwrap().collect::<Vec<_>>();
    assert_eq!(spans.len(), 2);
    assert!(
        spans
            .iter()
            .all(|span| span.source_span().source() == Some(source))
    );
}

#[test]
fn higher_degree_nurbs_promotes_evaluates_and_splits_exactly() {
    let source = CurveSource2::with_version(100, 4);
    let curve = NurbsCurve2::try_new_with_source(
        4,
        vec![p(0, 0), p(1, 4), p(2, 0), p(3, 4), p(4, 0)],
        vec![r(1); 5],
        [vec![r(0); 5], vec![r(1); 5]].concat(),
        source,
    )
    .unwrap();

    assert_eq!(curve.degree(), 4);
    assert_eq!(curve.point_at(&q(1, 2)).unwrap(), p(2, 2));
    let spans = curve.native_spans().unwrap().collect::<Vec<_>>();
    assert_eq!(spans.len(), 1);
    assert_eq!(spans[0].source_span().source(), Some(source));
    assert_eq!(spans[0].source_span().degree(), 4);
    assert!(matches!(spans[0].curve(), BezierSubcurve2::Rational(_)));

    let (left, right) = curve.split_at(q(1, 2)).unwrap();
    assert_eq!(left.end(), &p(2, 2));
    assert_eq!(right.start(), &p(2, 2));
    assert_eq!(left.source(), Some(source));
    assert_eq!(right.source(), Some(source));
}

#[test]
fn unclamped_nurbs_retains_active_endpoints_and_exact_editing() {
    let source = CurveSource2::with_version(102, 5);
    let curve = NurbsCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1); 4],
        (0..=6).map(r).collect(),
        source,
    )
    .unwrap();

    assert_eq!(curve.parameter_domain(), (&r(2), &r(4)));
    assert_eq!(curve.start(), &Point2::new(r(1), r(2)));
    assert_eq!(curve.end(), &Point2::new(r(5), r(2)));
    assert_eq!(curve.point_at(&r(3)).unwrap(), p(3, 4));
    assert!(curve.is_bezier_decomposition_cached());

    let inserted = curve.insert_knot(r(3)).unwrap();
    assert_eq!(inserted.start(), curve.start());
    assert_eq!(inserted.end(), curve.end());
    assert!(!inserted.is_bezier_decomposition_cached());
    assert_eq!(inserted.point_at(&r(3)).unwrap(), p(3, 4));

    let (left, right) = curve.split_at(r(3)).unwrap();
    assert_eq!(left.parameter_domain(), (&r(2), &r(3)));
    assert_eq!(right.parameter_domain(), (&r(3), &r(4)));
    assert_eq!(left.start(), curve.start());
    assert_eq!(left.end(), &p(3, 4));
    assert_eq!(right.start(), &p(3, 4));
    assert_eq!(right.end(), curve.end());

    let reversed = curve.reversed().unwrap();
    assert_eq!(reversed.start(), curve.end());
    assert_eq!(reversed.end(), curve.start());
    assert_eq!(
        reversed.point_at(&r(3)).unwrap(),
        curve.point_at(&r(3)).unwrap()
    );
}

#[test]
fn unclamped_weighted_nurbs_projects_homogeneous_endpoint_evidence() {
    let curve = NurbsCurve2::try_new(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(2), r(3), r(4)],
        (0..=6).map(r).collect(),
    )
    .unwrap();

    assert_eq!(curve.start(), &Point2::new(q(4, 3), q(8, 3)));
    assert_eq!(curve.end(), &Point2::new(q(36, 7), q(12, 7)));
    assert_eq!(curve.point_at(&r(2)).unwrap(), curve.start().clone());
    assert_eq!(curve.point_at(&r(4)).unwrap(), curve.end().clone());
    assert!(
        curve
            .bezier_decomposition()
            .unwrap()
            .refined_weights()
            .iter()
            .all(|weight| weight.zero_status() == hyperreal::ZeroKnowledge::NonZero)
    );
}

#[test]
fn invalid_nurbs_construction_returns_contextual_error() {
    let source = CurveSource2::new(101);
    let error = NurbsCurve2::try_new_with_source(
        1,
        vec![p(0, 0), p(1, 1)],
        vec![r(1), r(1)],
        vec![r(0), r(0), r(1)],
        source,
    )
    .unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::Construction);
    assert_eq!(error.family(), CurveFamily2::Nurbs);
    assert_eq!(error.source(), Some(source));
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: hypercurve::CurveError::InvalidBSpline,
            ..
        }
    ));
}

#[test]
fn periodic_nurbs_wraps_exact_points_derivatives_and_retains_source() {
    let source = CurveSource2::with_version(111, 4);
    let curve = NurbsCurve2::try_new_periodic_with_source(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        vec![r(1), r(1), r(1), r(1)],
        (0..=4).map(r).collect(),
        source,
    )
    .unwrap();

    assert_eq!(curve.period(), Some(&r(4)));
    assert!(matches!(
        curve.periodicity(),
        SplinePeriodicity2::Periodic { .. }
    ));
    assert_eq!(curve.source(), Some(source));
    assert_eq!(curve.parameter_domain(), (&r(0), &r(4)));
    assert_eq!(curve.control_points().len(), 6);
    assert_eq!(curve.knots().len(), 9);
    assert_eq!(curve.start(), curve.end());
    assert_eq!(curve.point_at(&r(0)).unwrap(), p(1, 0));
    assert_eq!(curve.point_at_wrapped(&r(-1)).unwrap(), p(0, 1));
    assert_eq!(curve.point_at_wrapped(&r(5)).unwrap(), p(2, 1));
    assert_eq!(curve.point_at_wrapped(&r(9)).unwrap(), p(2, 1));
    assert_eq!(
        curve.derivatives_at_wrapped(&q(11, 2), 3).unwrap(),
        curve.derivatives_at(&q(3, 2), 3).unwrap()
    );
    assert_eq!(
        curve.derivatives_at_wrapped(&r(4), 1).unwrap(),
        curve.derivatives_at(&r(0), 1).unwrap()
    );
}

#[test]
fn periodic_nurbs_editing_preserves_period_only_for_whole_curve_operations() {
    let curve = NurbsCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        vec![r(1), r(2), r(3), r(4)],
        (0..=4).map(r).collect(),
    )
    .unwrap();

    let inserted = curve.insert_knots(vec![q(1, 2), q(3, 2)]).unwrap();
    assert_eq!(inserted.period(), curve.period());
    assert_eq!(
        inserted.point_at_wrapped(&r(5)).unwrap(),
        curve.point_at_wrapped(&r(5)).unwrap()
    );

    let elevated = curve.degree_elevation(3).unwrap();
    assert_eq!(elevated.source_degree(), 2);
    assert_eq!(elevated.target_degree(), 3);
    assert_eq!(
        elevated.spans().first().unwrap().parameter_interval().0,
        &r(0)
    );
    assert_eq!(
        elevated.spans().last().unwrap().parameter_interval().1,
        &r(4)
    );
    assert_eq!(
        elevated
            .spans()
            .first()
            .unwrap()
            .curve()
            .point_at(&Real::zero(), &CurvePolicy::certified())
            .unwrap(),
        curve.start().clone()
    );
    assert_eq!(
        elevated
            .spans()
            .last()
            .unwrap()
            .curve()
            .point_at(&Real::one(), &CurvePolicy::certified())
            .unwrap(),
        curve.end().clone()
    );

    let reversed = curve.reversed().unwrap();
    assert_eq!(reversed.period(), curve.period());
    assert_eq!(reversed.start(), reversed.end());
    assert_eq!(
        reversed.point_at_wrapped(&r(1)).unwrap(),
        curve.point_at_wrapped(&r(3)).unwrap()
    );

    let (left, right) = curve.split_at(r(2)).unwrap();
    assert_eq!(left.period(), None);
    assert_eq!(right.period(), None);
    assert_ne!(left.start(), left.end());
    assert_ne!(right.start(), right.end());
}

#[test]
fn nonuniform_weighted_periodic_nurbs_supports_repeated_interior_knots() {
    let curve = NurbsCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(3, 0), p(4, 2), p(2, 5), p(-1, 2)],
        vec![r(1), r(2), r(5), r(3), r(4)],
        vec![r(0), r(1), r(1), r(3), r(5), r(8)],
    )
    .unwrap();
    let parameter = q(5, 2);
    let shifted = q(21, 2);

    assert_eq!(curve.period(), Some(&r(8)));
    assert_eq!(curve.start(), curve.end());
    assert_eq!(
        curve.point_at_wrapped(&shifted).unwrap(),
        curve.point_at(&parameter).unwrap()
    );
    assert_eq!(
        curve.derivative_at_wrapped(&shifted).unwrap(),
        curve.derivative_at(&parameter).unwrap()
    );
    assert!(curve.bezier_spans().unwrap().len() >= 4);
}

#[test]
fn periodic_nurbs_rejects_invalid_layout_and_nonperiodic_wrapping() {
    let invalid = NurbsCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(1, 0), p(1, 1)],
        vec![r(1), r(1)],
        (0..=3).map(r).collect(),
    )
    .unwrap_err();
    assert!(matches!(
        invalid,
        ExactCurveError::Invalid {
            cause: CurveError::InvalidPeriodicSpline,
            ..
        }
    ));

    let open = quadratic_nurbs(CurveSource2::new(112));
    let error = open.point_at_wrapped(&r(3)).unwrap_err();
    assert_eq!(error.operation(), CurveOperation2::Evaluation);
    assert_eq!(error.source(), Some(CurveSource2::new(112)));
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: CurveError::CurveIsNotPeriodic,
            ..
        }
    ));
}
