use hypercurve::{
    BezierParameter2, BezierParameterRange2, Classification, CubicBezier2, Curve2, CurveCertainty,
    CurveContext, CurveFamily2, CurveParameterRange2, CurvePath2, CurveRegion2,
    CurveRegionLoopRole, FillRule, FiniteProjectionOptions, LineSeg2, OffsetCornerStyle2, Point2,
    QuadraticBezier2, Real, RegionPointLocation,
};
use hypercurve::{
    CurveCornerMode2, CurveCornerNoSolution2, CurveCornerSolutions2, RationalQuadraticBezier2,
};

/// Contacts of one unit-domain analytic parallel with a line segment.
fn line_contacts(
    parallel: hypercurve::BezierParallel2,
    line: LineSeg2,
    policy: &CurveContext,
) -> Vec<hypercurve::CurveIntersectionContact2> {
    let Classification::Decided(range) = BezierParameterRange2::try_new(
        BezierParameter2::Exact(Real::zero()),
        BezierParameter2::Exact(Real::one()),
        policy,
    )
    .unwrap() else {
        panic!("the unit range is exact");
    };
    let Classification::Decided(parallel) =
        Curve2::try_analytic_parallel(parallel, range, policy).unwrap()
    else {
        panic!("the regular parallel is admitted");
    };
    let evidence = parallel
        .intersect_curve(&Curve2::from(line), policy)
        .unwrap()
        .value;
    assert!(evidence.is_complete());
    evidence.contacts().to_vec()
}

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn range(start: i64, end: i64, policy: &CurveContext) -> BezierParameterRange2 {
    match BezierParameterRange2::try_new(
        BezierParameter2::Exact(Real::from(start)),
        BezierParameter2::Exact(Real::from(end)),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(range) => range,
        Classification::Uncertain(reason) => panic!("unexpected range uncertainty: {reason:?}"),
    }
}

fn line_parallel_fragment(
    start: Point2,
    midpoint: Point2,
    end: Point2,
    distance: i64,
    start_parameter: i64,
    end_parameter: i64,
    policy: &CurveContext,
) -> Curve2 {
    let parallel = QuadraticBezier2::new(start, midpoint, end)
        .parallel_left(Real::from(distance))
        .unwrap();
    match Curve2::try_analytic_parallel(
        parallel,
        range(start_parameter, end_parameter, policy),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(fragment) => fragment,
        Classification::Uncertain(reason) => {
            panic!("unexpected analytic-parallel uncertainty: {reason:?}")
        }
    }
}

fn assert_real_equal(left: &Real, right: &Real) {
    assert_eq!(left.partial_cmp(right), Some(std::cmp::Ordering::Equal));
}

fn loop_vertex_at(region: &CurveRegion2, point: Point2, policy: &CurveContext) -> usize {
    let Classification::Decided(paths) = region.boundary_paths(policy).unwrap().value else {
        panic!("normalized exact boundary paths");
    };
    assert_eq!(paths.len(), 1);
    let point = point.into();
    paths[0]
        .curves()
        .iter()
        .position(|curve| {
            curve.start().coincides_with(&point, policy).value == Classification::Decided(true)
        })
        .expect("the authored corner survives normalization")
}

fn analytic_square(min_x: i64, max_x: i64, policy: &CurveContext) -> CurveRegion2 {
    let midpoint_x = (min_x + max_x) / 2;
    let edges = [
        (point(min_x, 0), point(midpoint_x, 0), point(max_x, 0)),
        (point(max_x, 0), point(max_x, 2), point(max_x, 4)),
        (point(max_x, 4), point(midpoint_x, 4), point(min_x, 4)),
        (point(min_x, 4), point(min_x, 2), point(min_x, 0)),
    ];
    let fragments = edges
        .into_iter()
        .map(|(start, midpoint, end)| {
            Curve2::from(line_parallel_fragment(
                start, midpoint, end, 0, 0, 1, policy,
            ))
        })
        .collect();
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[CurvePath2::try_new_with_policy(fragments, policy)
            .unwrap()
            .into_value()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        policy,
    )
    .unwrap()
    .into_value()
}

#[test]
fn boundary_curves_reenter_boolean_without_native_conversion() {
    let policy = CurveContext::STRICT;
    let region = analytic_square(0, 4, &policy);
    let curves = region.boundary_loops()[0].curves();
    assert!(curves.iter().any(|curve| {
        curve.family() == CurveFamily2::AnalyticParallel && curve.geometry().is_none()
    }));
    let path = CurvePath2::try_new_with_policy(curves.to_vec(), &policy)
        .expect("generated analytic boundary curves remain one exact path")
        .into_value();
    let replay =
        CurveRegion2::try_from_boundary_paths(&[path], hypercurve::FillRule::EvenOdd, &policy)
            .expect("the exact boundary path re-enters region construction")
            .into_value();
    let disjoint = analytic_square(10, 14, &policy);
    let batch = replay
        .boolean_regions(&disjoint, &policy)
        .expect("an analytic boundary re-enters Boolean operations");
    assert_eq!(batch.certainty, CurveCertainty::Certified);
    assert!(batch.value.intersection().is_empty());
    assert_eq!(batch.value.union().boundary_loops().len(), 2);
    assert_eq!(
        replay
            .classify_point(&point(2, 2).into(), &policy)
            .unwrap()
            .value,
        Classification::Decided(RegionPointLocation::Inside)
    );
}

fn quadratic_line(start: Point2, end: Point2) -> Curve2 {
    let midpoint = start.lerp(&end, (Real::one() / Real::from(2_u8)).unwrap());
    QuadraticBezier2::new(start, midpoint, end).into()
}

fn curved_parallel_cap(policy: &CurveContext) -> CurveRegion2 {
    let parallel = QuadraticBezier2::new(point(0, 0), point(2, 2), point(4, 0))
        .parallel_left(Real::one())
        .unwrap();
    let right = match parallel.point_at(&Real::one(), policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("right cap endpoint: {reason:?}"),
    };
    let left = match parallel.point_at(&Real::zero(), policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("left cap endpoint: {reason:?}"),
    };
    let analytic =
        match Curve2::try_analytic_parallel(parallel, range(1, 0, policy), policy).unwrap() {
            Classification::Decided(fragment) => Curve2::from(fragment),
            Classification::Uncertain(reason) => panic!("curved parallel cap: {reason:?}"),
        };
    let lower_left = Point2::new(left.x().clone(), Real::from(-2));
    let lower_right = Point2::new(right.x().clone(), Real::from(-2));
    let boundary = CurvePath2::try_new_with_policy(
        vec![
            analytic,
            quadratic_line(left, lower_left.clone()),
            quadratic_line(lower_left, lower_right.clone()),
            quadratic_line(lower_right, right),
        ],
        policy,
    )
    .unwrap();
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[boundary.into_value()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        policy,
    )
    .unwrap()
    .into_value()
}

fn analytic_rational_arc_corner_region(
    unit_end_weights: bool,
    reversed: bool,
    policy: &CurveContext,
) -> (CurveRegion2, usize) {
    let analytic = QuadraticBezier2::new(point(0, 0), point(1, 0), point(1, 1))
        .parallel_left(Real::zero())
        .unwrap();
    let analytic =
        match Curve2::try_analytic_parallel(analytic, range(0, 1, policy), policy).unwrap() {
            Classification::Decided(fragment) => Curve2::from(fragment),
            Classification::Uncertain(reason) => panic!("analytic arc fixture: {reason:?}"),
        };
    let arc = if unit_end_weights {
        let half_sqrt_two = (Real::from(2_i8).sqrt().unwrap() / Real::from(2_i8)).unwrap();
        RationalQuadraticBezier2::try_unit_end_weights(
            point(1, 1),
            point(2, 1),
            point(2, 2),
            half_sqrt_two,
        )
        .unwrap()
    } else {
        RationalQuadraticBezier2::try_new(
            point(1, 1),
            point(2, 1),
            point(2, 2),
            Real::one(),
            Real::one(),
            Real::from(2_i8),
        )
        .unwrap()
    };
    let arc = Curve2::from(arc);
    let lower_right = point(2, -1);
    let mut fragments = vec![
        analytic,
        arc,
        quadratic_line(point(2, 2), lower_right.clone()),
        quadratic_line(lower_right, point(0, 0)),
    ];
    if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|curve| curve.reversed(policy).unwrap().into_value())
            .collect();
    }
    let boundary = CurvePath2::try_new_with_policy(fragments, policy).unwrap();
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[boundary.into_value()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        policy,
    )
    .unwrap()
    .into_value();
    let vertex = loop_vertex_at(&region, point(1, 1), policy);
    (region, vertex)
}

#[test]
fn retained_rational_arc_and_analytic_parallel_fillet_exactly() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for unit_end_weights in [false, true] {
            for reversed in [false, true] {
                let (source, vertex_index) =
                    analytic_rational_arc_corner_region(unit_end_weights, reversed, &policy);
                let solved = source
                    .fillet_loop_vertex(
                        0,
                        vertex_index,
                        &hypercurve::CurveFillet2::new((Real::one() / Real::from(4_i8)).unwrap()),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!("retained rational-arc/analytic fillet must decide: {error:?}")
                    });
                let candidates = {
                    let solutions = solved.value;
                    let candidates = solutions.into_solutions();
                    assert!(
                        !candidates.is_empty(),
                        "expected at least one isolated fillet"
                    );
                    candidates
                };
                assert!(!candidates.is_empty());
                for candidate in candidates {
                    assert!(
                        candidate.boundary_loops()[0]
                            .curves()
                            .iter()
                            .any(|fragment| fragment.family() == CurveFamily2::CircularArc)
                    );
                    let disjoint = analytic_square(5, 6, &policy);
                    let replay = candidate
                        .boolean_regions(&disjoint, &policy)
                        .expect("the retained mixed fillet must re-enter the Boolean kernel")
                        .into_value();
                    assert!(replay.intersection().is_empty());
                    assert_eq!(replay.union().boundary_loops().len(), 2);
                }
            }
        }
    }
}

#[test]
fn retained_rational_arc_and_analytic_parallel_fillet_extends_exactly() {
    let radius = (Real::one() / Real::from(4_i8)).unwrap();
    let count =
        |solutions: hypercurve::CurveCornerSolutions2<CurveRegion2>| solutions.solutions().len();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for unit_end_weights in [false, true] {
            for reversed in [false, true] {
                let (source, vertex_index) =
                    analytic_rational_arc_corner_region(unit_end_weights, reversed, &policy);
                let trim_count = count(
                    source
                        .fillet_loop_vertex(
                            0,
                            vertex_index,
                            &hypercurve::CurveFillet2::new(radius.clone()),
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap_or_else(|error| {
                            panic!("retained trim-only fillet must decide: {error:?}")
                        })
                        .value,
                );
                let extended = source
                    .fillet_loop_vertex(
                        0,
                        vertex_index,
                        &hypercurve::CurveFillet2::new(radius.clone()),
                        CurveCornerMode2::TrimOrExtend,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!("retained trim-or-extend fillet must decide: {error:?}")
                    })
                    .value;
                let extended_count = count(extended);
                assert!(
                    extended_count > trim_count,
                    "the incident analytic ray must contribute an exterior fillet center"
                );
            }
        }
    }
}

#[test]
fn retained_arc_fillet_preserves_past_center_tangent_orientation() {
    let radius = (Real::from(5_i8) / Real::from(4_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for unit_end_weights in [false, true] {
            for reversed in [false, true] {
                let (source, vertex_index) =
                    analytic_rational_arc_corner_region(unit_end_weights, reversed, &policy);
                let solved = source
                    .fillet_loop_vertex(
                        0,
                        vertex_index,
                        &hypercurve::CurveFillet2::new(radius.clone()),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!("past-center arc fillet must decide: {error:?}")
                    });
                let candidates = {
                    let solutions = solved.value;
                    let candidates = solutions.into_solutions();
                    assert!(
                        !candidates.is_empty(),
                        "expected at least one isolated fillet"
                    );
                    candidates
                };
                assert!(candidates.iter().all(|candidate| {
                    candidate.boundary_loops()[0]
                        .curves()
                        .iter()
                        .any(|fragment| fragment.family() == CurveFamily2::CircularArc)
                }));
            }
        }
    }
}

fn rational_endpoint_curved_parallel_cap(policy: &CurveContext) -> CurveRegion2 {
    let parallel = QuadraticBezier2::new(point(0, 0), point(0, 2), point(4, 2))
        .parallel_left(Real::one())
        .unwrap();
    let right = match parallel.point_at(&Real::one(), policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("right cap endpoint: {reason:?}"),
    };
    let left = match parallel.point_at(&Real::zero(), policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("left cap endpoint: {reason:?}"),
    };
    let analytic =
        match Curve2::try_analytic_parallel(parallel, range(1, 0, policy), policy).unwrap() {
            Classification::Decided(fragment) => Curve2::from(fragment),
            Classification::Uncertain(reason) => panic!("curved parallel cap: {reason:?}"),
        };
    let lower_left = Point2::new(left.x().clone(), Real::from(-2));
    let lower_right = Point2::new(right.x().clone(), Real::from(-2));
    let boundary = CurvePath2::try_new_with_policy(
        vec![
            analytic,
            quadratic_line(left, lower_left.clone()),
            quadratic_line(lower_left, lower_right.clone()),
            quadratic_line(lower_right, right),
        ],
        policy,
    )
    .unwrap();
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[boundary.into_value()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        policy,
    )
    .unwrap()
    .into_value()
}

fn check_policy(policy: CurveContext) {
    let fragment = line_parallel_fragment(point(0, 0), point(2, 0), point(4, 0), 1, 1, 0, &policy);
    // The parallel y = 1 of the degenerate quadratic is traversed backwards.
    for (actual, expected) in [
        (fragment.start(), point(4, 1)),
        (fragment.end(), point(0, 1)),
    ] {
        assert_eq!(
            actual.coincides_with(&expected.into(), &policy).value,
            Classification::Decided(true)
        );
    }

    let region = analytic_square(0, 4, &policy);
    let boundary = &region.boundary_loops()[0];

    let projected = region
        .project_to_finite_profiles(&FiniteProjectionOptions::try_new(1.0e-2).unwrap(), &policy)
        .expect("analytic-parallel loops must cross the explicit finite-output boundary")
        .into_value();
    let Classification::Decided(projected) = projected else {
        panic!("analytic-parallel finite projection must retain decided loop ownership");
    };
    assert_eq!(projected.len(), 1);
    assert_eq!(projected[0].material().points().len(), 5);

    assert_eq!(
        region.filled_side_is_left(&policy).unwrap().value,
        Classification::Decided(&[true][..]),
    );
    assert_eq!(boundary.curves().len(), 4);
    let envelope = match region.bounds(&policy).unwrap().value {
        Classification::Decided(envelope) => envelope,
        Classification::Uncertain(reason) => panic!("unexpected envelope uncertainty: {reason:?}"),
    };
    assert_real_equal(envelope.min_x(), &Real::zero());
    assert_real_equal(envelope.min_y(), &Real::zero());
    assert_real_equal(envelope.max_x(), &Real::from(4));
    assert_real_equal(envelope.max_y(), &Real::from(4));

    for (point, expected) in [
        (point(2, 2), RegionPointLocation::Inside),
        (point(5, 2), RegionPointLocation::Outside),
        (point(0, 2), RegionPointLocation::Boundary),
    ] {
        match region
            .classify_point(&point.clone().into(), &policy)
            .unwrap()
            .value
        {
            Classification::Decided(location) => assert_eq!(location, expected),
            Classification::Uncertain(reason) => {
                panic!("unexpected analytic-region point uncertainty: {reason:?}")
            }
        }
    }

    let crossing_parallel = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0))
        .parallel_left(Real::one())
        .unwrap();
    let vertical = LineSeg2::try_new(point(2, 0), point(2, 2)).unwrap();
    let contacts = line_contacts(crossing_parallel, vertical, &policy);
    assert_eq!(contacts.len(), 1);
    assert!(contacts[0].is_certified_transverse());
    assert_eq!(
        contacts[0].tangent_cross_sign(),
        Some(hyperreal::RealSign::Positive)
    );

    let endpoint_tangent = QuadraticBezier2::new(point(0, 0), point(1, 0), point(2, 1))
        .parallel_left(Real::zero())
        .unwrap();
    let horizontal = LineSeg2::try_new(point(-1, 0), point(3, 0)).unwrap();
    let contacts = line_contacts(endpoint_tangent, horizontal, &policy);
    assert_eq!(contacts.len(), 1);
    assert!(!contacts[0].is_certified_transverse());

    let shifted = analytic_square(2, 6, &policy);
    let evidence = region.intersect_region(&shifted, &policy).unwrap().value;
    assert!(evidence.is_complete(), "{:#?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 2);
    let intersection = region
        .boolean_region(&shifted, hypercurve::BooleanOp::Intersection, &policy)
        .unwrap()
        .value;
    for (point, expected) in [
        (point(3, 2), RegionPointLocation::Inside),
        (point(1, 2), RegionPointLocation::Outside),
        (point(2, 2), RegionPointLocation::Boundary),
    ] {
        match intersection
            .classify_point(&point.clone().into(), &policy)
            .unwrap()
            .value
        {
            Classification::Decided(location) => assert_eq!(location, expected),
            Classification::Uncertain(reason) => {
                panic!("unexpected analytic Boolean point uncertainty: {reason:?}")
            }
        }
    }

    let curved = curved_parallel_cap(&policy);
    let cutter = analytic_square(1, 5, &policy);
    let evidence = curved.intersect_region(&cutter, &policy).unwrap().value;
    assert!(evidence.is_complete(), "{:#?}", evidence.blockers());
    assert!(!evidence.contacts().is_empty());
    assert!(evidence.contacts().iter().any(|contact| {
        contact.first_parameter().scalar().is_none()
            || contact.second_parameter().scalar().is_none()
    }));
    let clipped = curved
        .boolean_region(&cutter, hypercurve::BooleanOp::Intersection, &policy)
        .unwrap()
        .value;
    for (point, expected) in [
        (point(2, 1), RegionPointLocation::Inside),
        (point(0, 1), RegionPointLocation::Outside),
        (point(1, 1), RegionPointLocation::Boundary),
    ] {
        match clipped
            .classify_point(&point.clone().into(), &policy)
            .unwrap()
            .value
        {
            Classification::Decided(location) => assert_eq!(location, expected),
            Classification::Uncertain(reason) => {
                panic!("unexpected curved-parallel Boolean point uncertainty: {reason:?}")
            }
        }
    }
}

fn radical_cusp_split_parallel_region(policy: &CurveContext) -> CurveRegion2 {
    let half = (Real::one() / Real::from(2_u8)).unwrap();
    let parallel = QuadraticBezier2::new(point(0, 0), Point2::new(half, Real::zero()), point(1, 1))
        .parallel_left(Real::one())
        .unwrap();
    let analysis = match parallel
        .singularity_analysis(&CurveParameterRange2::unit(), policy)
        .unwrap()
    {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => panic!("cusp analysis: {reason:?}"),
    };
    let [cusp] = analysis.parallel_cusps() else {
        panic!("expected one radical parallel cusp");
    };
    assert!(cusp.scalar().is_some());

    let zero = BezierParameter2::Exact(Real::from(0));
    let one = BezierParameter2::Exact(Real::from(1));
    let make_range =
        |start: BezierParameter2, end: BezierParameter2| match BezierParameterRange2::try_new(
            start, end, policy,
        )
        .unwrap()
        {
            Classification::Decided(range) => range,
            Classification::Uncertain(reason) => panic!("cusp range: {reason:?}"),
        };
    let first = match Curve2::try_analytic_parallel(
        parallel.clone(),
        make_range(zero, cusp.clone()),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(fragment) => fragment,
        Classification::Uncertain(reason) => panic!("first cusp span: {reason:?}"),
    };
    let second = match Curve2::try_analytic_parallel(
        parallel.clone(),
        make_range(cusp.clone(), one),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(fragment) => fragment,
        Classification::Uncertain(reason) => panic!("second cusp span: {reason:?}"),
    };
    let start = match parallel.point_at(&Real::zero(), policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("parallel start: {reason:?}"),
    };
    let end = match parallel.point_at(&Real::one(), policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("parallel end: {reason:?}"),
    };

    let boundary = CurvePath2::try_new_with_policy(
        vec![
            Curve2::from(first),
            Curve2::from(second),
            quadratic_line(end, start),
        ],
        policy,
    )
    .expect("the shared analytic carrier and cusp parameter certify connectivity");
    assert_eq!(boundary.value.curves().len(), 3);
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[boundary.into_value()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        policy,
    )
    .expect("the radical cusp cap has exact regularized topology")
    .into_value()
}

fn self_crossing_cusp_split_parallel_region(policy: &CurveContext) -> CurveRegion2 {
    let source = CubicBezier2::new(point(0, 0), point(0, 4), point(4, -4), point(4, 0));
    let parallel = source
        .parallel_left((Real::one() / Real::from(2_u8)).unwrap())
        .unwrap();
    let analysis = match parallel
        .singularity_analysis(&CurveParameterRange2::unit(), policy)
        .unwrap()
    {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => panic!("cusp analysis: {reason:?}"),
    };
    let [first_cusp, second_cusp] = analysis.parallel_cusps() else {
        panic!("expected two algebraic parallel cusps");
    };
    let boundaries = [
        BezierParameter2::Exact(Real::from(0)),
        first_cusp.clone(),
        second_cusp.clone(),
        BezierParameter2::Exact(Real::from(1)),
    ];
    let mut fragments = boundaries
        .windows(2)
        .map(|window| {
            let range =
                match BezierParameterRange2::try_new(window[0].clone(), window[1].clone(), policy)
                    .unwrap()
                {
                    Classification::Decided(range) => range,
                    Classification::Uncertain(reason) => panic!("parallel span: {reason:?}"),
                };
            match Curve2::try_analytic_parallel(parallel.clone(), range, policy).unwrap() {
                Classification::Decided(fragment) => Curve2::from(fragment),
                Classification::Uncertain(reason) => {
                    panic!("analytic parallel span: {reason:?}")
                }
            }
        })
        .collect::<Vec<_>>();
    let start = match parallel.point_at(&Real::zero(), policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("parallel start: {reason:?}"),
    };
    let end = match parallel.point_at(&Real::one(), policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("parallel end: {reason:?}"),
    };
    fragments.push(quadratic_line(end, start));
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[CurvePath2::try_new_with_policy(fragments, policy)
            .unwrap()
            .into_value()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        policy,
    )
    .unwrap()
    .into_value()
}

#[test]
fn analytic_parallel_fragments_retain_exact_region_evidence_under_both_policies() {
    check_policy(CurveContext::STRICT);
    check_policy(CurveContext::APPROXIMATE_512);
}

#[test]
fn analytic_parallel_chamfers_retain_normalized_cut_points() {
    let setback = (Real::one() / Real::from(4_u8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (corner, analytic_next) in [(point(4, 3), true), (point(-1, 0), false)] {
            let source = rational_endpoint_curved_parallel_cap(&policy);
            let vertex = loop_vertex_at(&source, corner, &policy);
            let outcome = source
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertex,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("a represented-endpoint analytic parallel chamfer must complete");
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            let region = match outcome.value {
                CurveCornerSolutions2::Unique(region) => region,
                other => {
                    panic!("the analytic-parallel corner must have one finite chamfer: {other:?}")
                }
            };
            let fragments = region.boundary_loops()[0].curves();
            assert_eq!(fragments.len(), 5);
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| { fragment.family() == CurveFamily2::AnalyticParallel })
                    .count(),
                1
            );
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| { fragment.family() == CurveFamily2::Line })
                    .count(),
                1
            );
            let chord = fragments
                .iter()
                .find(|fragment| fragment.family() == CurveFamily2::Line)
                .expect("the chamfer is retained as one authoritative exact chord");
            assert_eq!(
                [chord.start(), chord.end()]
                    .into_iter()
                    .filter(|point| point.coordinates().is_some())
                    .count(),
                1,
                "one chamfer endpoint retains selected evidence and the other has coordinates",
            );

            for (sample, expected) in [
                (point(2, 0), RegionPointLocation::Inside),
                (point(6, 0), RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    region
                        .classify_point(&sample.clone().into(), &policy)
                        .unwrap()
                        .value,
                    Classification::Decided(expected)
                );
            }

            let union = region
                .boolean_region(
                    &analytic_square(10, 14, &policy),
                    hypercurve::BooleanOp::Union,
                    &policy,
                )
                .expect("a later Boolean must consume the retained chamfer evidence");
            assert_eq!(union.certainty, CurveCertainty::Certified);
            for (sample, expected) in [
                (point(2, 0), RegionPointLocation::Inside),
                (point(12, 2), RegionPointLocation::Inside),
                (point(7, 0), RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    union
                        .value
                        .classify_point(&sample.clone().into(), &policy)
                        .unwrap()
                        .value,
                    Classification::Decided(expected)
                );
            }

            let (previous_setback, next_setback) = if analytic_next {
                (setback.clone(), Real::zero())
            } else {
                (Real::zero(), setback.clone())
            };
            let one_sided = source
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertex,
                    previous_setback,
                    next_setback,
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("a zero analytic-side setback must retain the exact corner");
            assert_eq!(one_sided.certainty, CurveCertainty::Certified);
            assert!(matches!(one_sided.value, CurveCornerSolutions2::Unique(_)));

            let (previous_setback, next_setback) = if analytic_next {
                (setback.clone(), Real::from(100_u8))
            } else {
                (Real::from(100_u8), setback.clone())
            };
            let over_setback = source
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertex,
                    previous_setback,
                    next_setback,
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("an over-setback must terminate as an exact no-solution");
            assert_eq!(over_setback.certainty, CurveCertainty::Certified);
            assert!(matches!(
                over_setback.value,
                CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::OutsideTrimDomain)
            ));
        }
    }
}

#[test]
fn algebraic_endpoint_analytic_parallel_chamfers_replay_selected_distance() {
    let first_setback = (Real::one() / Real::from(4_u8)).unwrap();
    let second_setback = (Real::one() / Real::from(16_u8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for corner in [point(4, 3), point(-1, 0)] {
            let source = rational_endpoint_curved_parallel_cap(&policy);
            let first_vertex = loop_vertex_at(&source, corner, &policy);
            let first = source
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    first_vertex,
                    first_setback.clone(),
                    first_setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the first analytic-parallel chamfer must complete");
            assert_eq!(first.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(first) = first.value else {
                panic!("the first analytic-parallel chamfer must be unique");
            };

            // The first cut is an algebraic parameter retained jointly by the
            // analytic fragment and its chord. Chamfer that new junction in
            // both carrier orientations without materializing its point.
            let Classification::Decided(paths) = first.boundary_paths(&policy).unwrap().value
            else {
                panic!("exact edited boundary");
            };
            let curves = paths[0].curves();
            let generated_chord = |curve: &Curve2| {
                curve.family() == hypercurve::CurveFamily2::Line && curve.geometry().is_none()
            };
            let second_vertex = (0..curves.len())
                .find(|&index| {
                    let previous = &curves[(index + curves.len() - 1) % curves.len()];
                    let next = &curves[index];
                    (previous.family() == hypercurve::CurveFamily2::AnalyticParallel
                        && generated_chord(next))
                        || (generated_chord(previous)
                            && next.family() == hypercurve::CurveFamily2::AnalyticParallel)
                })
                .expect("the selected analytic/chord junction is retained");
            let second = first
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    second_vertex,
                    second_setback.clone(),
                    second_setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("an algebraic-endpoint analytic chamfer must complete");
            assert_eq!(second.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(second) = second.value else {
                panic!("the algebraic-endpoint analytic chamfer must be unique");
            };
            let fragments = second.boundary_loops()[0].curves();
            assert_eq!(fragments.len(), 6);
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| { fragment.family() == CurveFamily2::AnalyticParallel })
                    .count(),
                1
            );
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| { fragment.family() == CurveFamily2::Line })
                    .count(),
                2
            );
            for (sample, expected) in [
                (point(2, 0), RegionPointLocation::Inside),
                (point(6, 0), RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    second
                        .classify_point(&sample.clone().into(), &policy)
                        .unwrap()
                        .value,
                    Classification::Decided(expected)
                );
            }
        }
    }
}

#[test]
fn curve_trim_intersects_analytic_parallel_region_boundaries() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = analytic_square(0, 4, &policy);
        let source = Curve2::from(LineSeg2::try_new(point(-1, 2), point(5, 2)).unwrap());
        let outcome = source.trim_inside_region(&region, &policy).unwrap();
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
        let [curve] = outcome.value.as_slice() else {
            panic!("analytic-square trim must retain one exact line");
        };
        assert_eq!(curve.start(), point(0, 2).into());
        assert_eq!(curve.end(), point(4, 2).into());
    }
}

#[test]
fn curve_trim_retains_an_analytic_parallel_boundary_overlap() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = analytic_square(0, 4, &policy);
        let source = Curve2::from(LineSeg2::try_new(point(-1, 0), point(5, 0)).unwrap());
        let outcome = source.trim_inside_region(&region, &policy).unwrap();
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
        let [curve] = outcome.value.as_slice() else {
            panic!("analytic boundary overlap must retain one exact line");
        };
        assert_eq!(curve.start(), point(0, 0).into());
        assert_eq!(curve.end(), point(4, 0).into());
    }
}

#[test]
fn radical_parallel_cusp_spans_connect_under_both_policies() {
    assert_eq!(
        radical_cusp_split_parallel_region(&CurveContext::STRICT).boundary_loops()[0].len(),
        3
    );
    assert_eq!(
        radical_cusp_split_parallel_region(&CurveContext::APPROXIMATE_512).boundary_loops()[0]
            .len(),
        3
    );
}

#[test]
fn radical_parallel_cusp_offsets_exactly_under_both_policies() {
    let distance = (Real::one() / Real::from(10_u8)).unwrap();
    let mut strict_signature = None;
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = radical_cusp_split_parallel_region(&policy);
        let offset = source
            .offset(distance.clone(), &OffsetCornerStyle2::Round, &policy)
            .expect("the represented radical cusp offset must complete");
        assert!(!offset.value.is_empty());
        let fragment_kinds = offset
            .value
            .boundary_loops()
            .iter()
            .map(|boundary| {
                boundary
                    .curves()
                    .iter()
                    .map(Curve2::family)
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        let roles = match offset.value.loop_roles(&policy).unwrap().value {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => panic!("offset loop roles: {reason:?}"),
        };
        let signature = (
            fragment_kinds,
            roles,
            offset.value.loop_fill_rules().map(<[_]>::to_vec),
        );
        if let Some(strict_signature) = &strict_signature {
            assert_eq!(&signature, strict_signature);
        } else {
            strict_signature = Some(signature);
        }
    }
}

#[test]
fn cusp_split_analytic_self_crossing_normalizes_at_admission() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = self_crossing_cusp_split_parallel_region(&policy);
        assert_eq!(region.boundary_loops().len(), 3);
        assert_eq!(
            region.filled_side_is_left(&policy).unwrap().value,
            Classification::Decided(&[true; 3][..])
        );
        assert_eq!(
            region.regularized_region(&policy).unwrap().into_value(),
            region
        );
    }
}

#[test]
fn general_boundary_paths_preserve_analytic_carriers_and_boolean_reentry() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = curved_parallel_cap(&policy);
        let exported = source.boundary_paths(&policy).unwrap();
        assert_eq!(exported.certainty, CurveCertainty::Certified);
        let Classification::Decided(paths) = exported.value else {
            panic!("exact boundary paths")
        };
        assert_eq!(paths.len(), 1);
        let analytic = &paths[0].curves()[0];
        assert_eq!(
            analytic.family(),
            hypercurve::CurveFamily2::AnalyticParallel
        );
        assert!(analytic.geometry().is_none());
        assert!(analytic.start().coordinates().is_none());
        let reversed = analytic.reversed(&policy).unwrap();
        assert_eq!(reversed.certainty, CurveCertainty::Certified);
        assert_eq!(
            analytic
                .start()
                .coincides_with(&reversed.value.end(), &policy)
                .value,
            Classification::Decided(true),
        );
        assert_eq!(
            analytic
                .end()
                .coincides_with(&reversed.value.start(), &policy)
                .value,
            Classification::Decided(true),
        );
        assert!(analytic.bounds().is_ok());
        let reversed_path = paths[0].reversed(&policy).unwrap();
        assert_eq!(reversed_path.certainty, CurveCertainty::Certified);
        let restored = CurveRegion2::try_from_boundary_paths(
            &[reversed_path.value],
            hypercurve::FillRule::EvenOdd,
            &policy,
        )
        .unwrap();
        assert_eq!(restored.certainty, CurveCertainty::Certified);
        let clipped = restored
            .value
            .boolean_region(
                &analytic_square(1, 3, &policy),
                hypercurve::BooleanOp::Intersection,
                &policy,
            )
            .unwrap();
        assert_eq!(clipped.certainty, CurveCertainty::Certified);
        for (query, expected) in [
            (point(2, 1), RegionPointLocation::Inside),
            (point(2, 3), RegionPointLocation::Outside),
            (point(-2, 0), RegionPointLocation::Outside),
        ] {
            let located = clipped
                .value
                .classify_point(&query.clone().into(), &policy)
                .unwrap();
            assert_eq!(located.certainty, CurveCertainty::Certified);
            assert_eq!(located.value, Classification::Decided(expected));
        }
    }
}
