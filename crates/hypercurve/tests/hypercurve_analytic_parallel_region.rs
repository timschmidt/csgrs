use hypercurve::{
    BezierLineContactKind, BezierLineContactRelation, BezierLineCrossingDirection,
    BezierParallelFragment2, BezierParameter2, BezierParameterRange2, BezierRetainedCurveEnvelope2,
    BezierRetainedEndpointEnvelope2, BezierSplitFragment2, BezierSubcurve2, Classification,
    CubicBezier2, Curve2, CurveBoundaryInteriorSide2, CurveCertainty, CurveContext, CurveRegion2,
    CurveRegionBoundaryLoop2, CurveRegionLoopRole, FillRule, FiniteProjectionOptions, LineSeg2,
    OffsetCornerStyle2, Point2, QuadraticBezier2, Real, RegionPointLocation,
};
use hypercurve::{
    CurveCornerMode2, CurveCornerNoSolution2, CurveCornerSolutions2, RationalBezier2,
    RationalBezierIntersectionPointEvidence2, RationalQuadraticBezier2, RealSign,
};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn exact_parameter(value: i64, policy: &CurveContext) -> BezierParameter2 {
    match BezierParameter2::exact(Real::from(value), policy).unwrap() {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => panic!("unexpected parameter uncertainty: {reason:?}"),
    }
}

fn range(start: i64, end: i64, policy: &CurveContext) -> BezierParameterRange2 {
    match BezierParameterRange2::try_new(
        exact_parameter(start, policy),
        exact_parameter(end, policy),
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
) -> BezierParallelFragment2 {
    let parallel = QuadraticBezier2::new(start, midpoint, end)
        .parallel_left(Real::from(distance))
        .unwrap();
    match BezierParallelFragment2::try_new(
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
            BezierSplitFragment2::AnalyticParallel(line_parallel_fragment(
                start, midpoint, end, 0, 0, 1, policy,
            ))
        })
        .collect();
    CurveRegion2::try_new_with_loop_topology(
        vec![CurveRegionBoundaryLoop2::new(fragments, policy).unwrap()],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .unwrap()
}

fn materialized_line(start: Point2, end: Point2, policy: &CurveContext) -> BezierSplitFragment2 {
    let midpoint = start.lerp(
        &end,
        (Real::one() / Real::from(2_u8)).expect("one half is represented"),
    );
    BezierSplitFragment2::Materialized {
        start: exact_parameter(0, policy),
        end: exact_parameter(1, policy),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(start, midpoint, end)),
    }
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
        match BezierParallelFragment2::try_new(parallel, range(1, 0, policy), policy).unwrap() {
            Classification::Decided(fragment) => BezierSplitFragment2::AnalyticParallel(fragment),
            Classification::Uncertain(reason) => panic!("curved parallel cap: {reason:?}"),
        };
    let lower_left = Point2::new(left.x().clone(), Real::from(-2));
    let lower_right = Point2::new(right.x().clone(), Real::from(-2));
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            analytic,
            materialized_line(left, lower_left.clone(), policy),
            materialized_line(lower_left, lower_right.clone(), policy),
            materialized_line(lower_right, right, policy),
        ],
        policy,
    )
    .unwrap();
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .unwrap()
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
        match BezierParallelFragment2::try_new(analytic, range(0, 1, policy), policy).unwrap() {
            Classification::Decided(fragment) => BezierSplitFragment2::AnalyticParallel(fragment),
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
    let arc = BezierSplitFragment2::Materialized {
        start: exact_parameter(0, policy),
        end: exact_parameter(1, policy),
        curve: BezierSubcurve2::RationalQuadratic(arc),
    };
    let lower_right = point(2, -1);
    let mut fragments = vec![
        analytic,
        arc,
        materialized_line(point(2, 2), lower_right.clone(), policy),
        materialized_line(lower_right, point(0, 0), policy),
    ];
    if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect();
    }
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy).unwrap();
    let region = CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![if reversed {
            CurveBoundaryInteriorSide2::Left
        } else {
            CurveBoundaryInteriorSide2::Right
        }],
    )
    .unwrap();
    (region, if reversed { 3 } else { 1 })
}

#[test]
fn retained_rational_arc_and_analytic_parallel_fillet_exactly() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for unit_end_weights in [false, true] {
            for reversed in [false, true] {
                let (source, vertex_index) =
                    analytic_rational_arc_corner_region(unit_end_weights, reversed, &policy);
                let solved = source
                    .fillet_loop_vertex_by_radius(
                        0,
                        vertex_index,
                        (Real::one() / Real::from(4_i8)).unwrap(),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!("retained rational-arc/analytic fillet must decide: {error:?}")
                    });
                let candidates = match solved.value {
                    CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                    CurveCornerSolutions2::Multiple(candidates) => candidates,
                    CurveCornerSolutions2::NoSolution(reason) => {
                        panic!("retained rational-arc/analytic fillet has no solution: {reason:?}")
                    }
                };
                assert!(!candidates.is_empty());
                for candidate in candidates {
                    assert!(candidate.boundary_loops()[0].fragments().iter().any(
                        |fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        )
                    ));
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
fn analytic_parallel_intersects_independently_parameterized_circles_exactly() {
    let center = point(1, 2);
    let source = QuadraticBezier2::new(point(0, 0), point(1, 0), point(1, 1));
    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    let half_sqrt_two = (Real::from(2_i8).sqrt().unwrap() / Real::from(2_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let mut contact_count = 0;
        for distance in [quarter.clone(), -quarter.clone()] {
            let parallel = source.parallel_left(distance.clone()).unwrap();
            let radius_scale = Real::one() - distance;
            let scaled = |point: Point2| {
                let radial = point.delta_from(&center);
                center.translated(&radial.0 * &radius_scale, &radial.1 * &radius_scale)
            };
            let circle: RationalBezier2 = RationalQuadraticBezier2::try_unit_end_weights(
                scaled(point(1, 1)),
                scaled(point(2, 1)),
                scaled(point(2, 2)),
                half_sqrt_two.clone(),
            )
            .unwrap()
            .into();
            let intersections = match parallel.intersections(&circle, &policy).unwrap() {
                Classification::Decided(intersections) => intersections,
                Classification::Uncertain(reason) => {
                    panic!("analytic/circle intersection remained uncertain: {reason:?}")
                }
            };
            assert!(intersections.is_complete());
            contact_count += intersections.contacts().len();
        }
        assert_eq!(contact_count, 1);
    }
}

#[test]
fn analytic_parallel_circle_tangency_retains_zero_cross_evidence() {
    let source = QuadraticBezier2::new(point(-2, 0), point(0, 0), point(2, 0));
    let half_sqrt_two = (Real::from(2_i8).sqrt().unwrap() / Real::from(2_i8)).unwrap();
    let circle: RationalBezier2 = RationalQuadraticBezier2::try_unit_end_weights(
        point(1, 0),
        point(1, 1),
        point(0, 1),
        half_sqrt_two,
    )
    .unwrap()
    .into();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parallel = source.parallel_left(Real::one()).unwrap();
        let intersections = match parallel.intersections(&circle, &policy).unwrap() {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                panic!("analytic/circle tangency remained uncertain: {reason:?}")
            }
        };
        assert!(intersections.is_complete());
        let [contact] = intersections.contacts() else {
            panic!("analytic/circle tangency must retain exactly one contact")
        };
        assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Zero));
        assert_eq!(contact.tangent_dot_sign(), Some(RealSign::Negative));
        assert!(!contact.is_certified_transverse());
    }
}

#[test]
fn analytic_parallel_circle_fast_path_excludes_other_support_contacts() {
    let source = QuadraticBezier2::new(point(-2, -1), point(0, -1), point(2, -1));
    let half_sqrt_two = (Real::from(2_i8).sqrt().unwrap() / Real::from(2_i8)).unwrap();
    let circle: RationalBezier2 = RationalQuadraticBezier2::try_unit_end_weights(
        point(1, 0),
        point(1, 1),
        point(0, 1),
        half_sqrt_two,
    )
    .unwrap()
    .into();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parallel = source.parallel_left(Real::one()).unwrap();
        let intersections = match parallel.intersections(&circle, &policy).unwrap() {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                panic!("analytic/circle span filtering remained uncertain: {reason:?}")
            }
        };
        assert!(intersections.is_complete());
        let [contact] = intersections.contacts() else {
            panic!("one of two supporting-circle contacts lies on the retained quarter")
        };
        assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Positive));
        assert!(contact.is_certified_transverse());
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
                    .fillet_loop_vertex_by_radius(
                        0,
                        vertex_index,
                        radius.clone(),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!("past-center arc fillet must decide: {error:?}")
                    });
                let candidates = match solved.value {
                    CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                    CurveCornerSolutions2::Multiple(candidates) => candidates,
                    CurveCornerSolutions2::NoSolution(reason) => {
                        panic!("past-center arc fillet has no solution: {reason:?}")
                    }
                };
                assert!(candidates.iter().all(|candidate| {
                    candidate.boundary_loops()[0]
                        .fragments()
                        .iter()
                        .any(|fragment| {
                            matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                        })
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
        match BezierParallelFragment2::try_new(parallel, range(1, 0, policy), policy).unwrap() {
            Classification::Decided(fragment) => BezierSplitFragment2::AnalyticParallel(fragment),
            Classification::Uncertain(reason) => panic!("curved parallel cap: {reason:?}"),
        };
    let lower_left = Point2::new(left.x().clone(), Real::from(-2));
    let lower_right = Point2::new(right.x().clone(), Real::from(-2));
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            analytic,
            materialized_line(left, lower_left.clone(), policy),
            materialized_line(lower_left, lower_right.clone(), policy),
            materialized_line(lower_right, right, policy),
        ],
        policy,
    )
    .unwrap();
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .unwrap()
}

fn check_policy(policy: CurveContext) {
    let fragment = line_parallel_fragment(point(0, 0), point(2, 0), point(4, 0), 1, 1, 0, &policy);
    assert!(fragment.is_reversed());
    assert_eq!(
        fragment.range().exact_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    let representative = match fragment.representative_point(&policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            panic!("unexpected representative-point uncertainty: {reason:?}")
        }
    };
    assert_real_equal(representative.x(), &Real::from(2));
    assert_real_equal(representative.y(), &Real::one());

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

    let endpoint_envelope = match BezierRetainedEndpointEnvelope2::from_loop(boundary, &policy) {
        Classification::Decided(envelope) => envelope,
        Classification::Uncertain(reason) => {
            panic!("unexpected endpoint-envelope uncertainty: {reason:?}")
        }
    };
    assert_eq!(endpoint_envelope.native_endpoint_count(), 8);

    assert!(region.has_algebraic_fragments());
    let curve_envelope = match BezierRetainedCurveEnvelope2::from_region(&region, &policy) {
        Classification::Decided(envelope) => envelope,
        Classification::Uncertain(reason) => {
            panic!("unexpected curve-envelope uncertainty: {reason:?}")
        }
    };
    assert_eq!(curve_envelope.exact_fragment_count(), 4);
    assert_real_equal(curve_envelope.envelope().min_x(), &Real::zero());
    assert_real_equal(curve_envelope.envelope().min_y(), &Real::zero());
    assert_real_equal(curve_envelope.envelope().max_x(), &Real::from(4));
    assert_real_equal(curve_envelope.envelope().max_y(), &Real::from(4));

    for (point, expected) in [
        (point(2, 2), RegionPointLocation::Inside),
        (point(5, 2), RegionPointLocation::Outside),
        (point(0, 2), RegionPointLocation::Boundary),
    ] {
        match region.classify_point(&point, &policy).unwrap().value {
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
    let relation = match crossing_parallel
        .relation_to_supporting_line_with_contacts(&vertical, &policy)
        .unwrap()
    {
        Classification::Decided(relation) => relation,
        Classification::Uncertain(reason) => {
            panic!("unexpected parallel-line uncertainty: {reason:?}")
        }
    };
    let BezierLineContactRelation::Contacts { contacts } = relation else {
        panic!("expected one exact parallel-line contact, got {relation:?}");
    };
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Crossing);
    assert_eq!(
        contacts[0].crossing_direction(),
        Some(BezierLineCrossingDirection::PositiveToNegative)
    );

    let endpoint_tangent = QuadraticBezier2::new(point(0, 0), point(1, 0), point(2, 1))
        .parallel_left(Real::zero())
        .unwrap();
    let horizontal = LineSeg2::try_new(point(-1, 0), point(3, 0)).unwrap();
    let relation = match endpoint_tangent
        .relation_to_supporting_line_with_contacts(&horizontal, &policy)
        .unwrap()
    {
        Classification::Decided(relation) => relation,
        Classification::Uncertain(reason) => {
            panic!("unexpected endpoint-tangent uncertainty: {reason:?}")
        }
    };
    let BezierLineContactRelation::Contacts { contacts } = relation else {
        panic!("expected an endpoint tangent, got {relation:?}");
    };
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Tangent);
    assert_eq!(contacts[0].crossing_direction(), None);

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
        match intersection.classify_point(&point, &policy).unwrap().value {
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
        !contact.first_parameter().is_exact() || !contact.second_parameter().is_exact()
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
        match clipped.classify_point(&point, &policy).unwrap().value {
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
    let analysis = match parallel.singularity_analysis(policy).unwrap() {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => panic!("cusp analysis: {reason:?}"),
    };
    let [cusp] = analysis.parallel_cusps() else {
        panic!("expected one radical parallel cusp");
    };
    assert!(cusp.is_exact());

    let zero = exact_parameter(0, policy);
    let one = exact_parameter(1, policy);
    let make_range =
        |start: BezierParameter2, end: BezierParameter2| match BezierParameterRange2::try_new(
            start, end, policy,
        )
        .unwrap()
        {
            Classification::Decided(range) => range,
            Classification::Uncertain(reason) => panic!("cusp range: {reason:?}"),
        };
    let first = match BezierParallelFragment2::try_new(
        parallel.clone(),
        make_range(zero, cusp.clone()),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(fragment) => fragment,
        Classification::Uncertain(reason) => panic!("first cusp span: {reason:?}"),
    };
    let second = match BezierParallelFragment2::try_new(
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

    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            BezierSplitFragment2::AnalyticParallel(first),
            BezierSplitFragment2::AnalyticParallel(second),
            materialized_line(end, start, policy),
        ],
        policy,
    )
    .expect("the shared analytic carrier and cusp parameter certify connectivity");
    assert_eq!(boundary.len(), 3);
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Right],
    )
    .expect("the radical cusp cap has exact authored topology")
}

fn self_crossing_cusp_split_parallel_region(policy: &CurveContext) -> CurveRegion2 {
    let source = CubicBezier2::new(point(0, 0), point(0, 4), point(4, -4), point(4, 0));
    let parallel = source
        .parallel_left((Real::one() / Real::from(2_u8)).unwrap())
        .unwrap();
    let analysis = match parallel.singularity_analysis(policy).unwrap() {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => panic!("cusp analysis: {reason:?}"),
    };
    let [first_cusp, second_cusp] = analysis.parallel_cusps() else {
        panic!("expected two algebraic parallel cusps");
    };
    let boundaries = [
        exact_parameter(0, policy),
        first_cusp.clone(),
        second_cusp.clone(),
        exact_parameter(1, policy),
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
            match BezierParallelFragment2::try_new(parallel.clone(), range, policy).unwrap() {
                Classification::Decided(fragment) => {
                    BezierSplitFragment2::AnalyticParallel(fragment)
                }
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
    fragments.push(materialized_line(end, start, policy));
    CurveRegion2::try_new_with_loop_topology(
        vec![CurveRegionBoundaryLoop2::new(fragments, policy).unwrap()],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .unwrap()
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
        for vertex in [0, 1] {
            let source = rational_endpoint_curved_parallel_cap(&policy);
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
            let fragments = region.boundary_loops()[0].fragments();
            assert_eq!(fragments.len(), 5);
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| {
                        matches!(fragment, BezierSplitFragment2::AnalyticParallel(_))
                    })
                    .count(),
                1
            );
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| {
                        matches!(fragment, BezierSplitFragment2::AlgebraicChord(_))
                    })
                    .count(),
                1
            );
            let chord = fragments
                .iter()
                .find_map(|fragment| match fragment {
                    BezierSplitFragment2::AlgebraicChord(chord) => Some(chord),
                    _ => None,
                })
                .expect("the chamfer is retained as one authoritative exact chord");
            let is_analytic = |endpoint: &RationalBezierIntersectionPointEvidence2| {
                matches!(
                    endpoint,
                    RationalBezierIntersectionPointEvidence2::AnalyticParallel(_)
                )
            };
            let is_exact = |endpoint: &RationalBezierIntersectionPointEvidence2| {
                matches!(endpoint, RationalBezierIntersectionPointEvidence2::Exact(_))
            };
            assert_eq!(
                usize::from(is_analytic(chord.start())) + usize::from(is_analytic(chord.end())),
                1
            );
            assert_eq!(
                usize::from(is_exact(chord.start())) + usize::from(is_exact(chord.end())),
                1
            );

            for (sample, expected) in [
                (point(2, 0), RegionPointLocation::Inside),
                (point(6, 0), RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    region.classify_point(&sample, &policy).unwrap().value,
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
                    union.value.classify_point(&sample, &policy).unwrap().value,
                    Classification::Decided(expected)
                );
            }

            let (previous_setback, next_setback) = if vertex == 0 {
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

            let (previous_setback, next_setback) = if vertex == 0 {
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
        for first_vertex in [0, 1] {
            let source = rational_endpoint_curved_parallel_cap(&policy);
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
            let second = first
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    1,
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
            let fragments = second.boundary_loops()[0].fragments();
            assert_eq!(fragments.len(), 6);
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| {
                        matches!(fragment, BezierSplitFragment2::AnalyticParallel(_))
                    })
                    .count(),
                1
            );
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| {
                        matches!(fragment, BezierSplitFragment2::AlgebraicChord(_))
                    })
                    .count(),
                2
            );
            for (sample, expected) in [
                (point(2, 0), RegionPointLocation::Inside),
                (point(6, 0), RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    second.classify_point(&sample, &policy).unwrap().value,
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
        let [BezierSplitFragment2::Materialized { curve, .. }] = outcome.value.as_slice() else {
            panic!("analytic-square trim must retain one materialized line fragment");
        };
        assert_eq!(curve.start(), &point(0, 2));
        assert_eq!(curve.end(), &point(4, 2));
    }
}

#[test]
fn curve_trim_retains_an_analytic_parallel_boundary_overlap() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = analytic_square(0, 4, &policy);
        let source = Curve2::from(LineSeg2::try_new(point(-1, 0), point(5, 0)).unwrap());
        let outcome = source.trim_inside_region(&region, &policy).unwrap();
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
        let [BezierSplitFragment2::Materialized { curve, .. }] = outcome.value.as_slice() else {
            panic!("analytic boundary overlap must retain one materialized line fragment");
        };
        assert_eq!(curve.start(), &point(0, 0));
        assert_eq!(curve.end(), &point(4, 0));
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
                    .fragments()
                    .iter()
                    .map(|fragment| match fragment {
                        BezierSplitFragment2::Materialized { curve, .. } => match curve {
                            BezierSubcurve2::Quadratic(_) => 0_u8,
                            BezierSubcurve2::Cubic(_) => 1,
                            BezierSubcurve2::RationalQuadratic(_) => 2,
                            BezierSubcurve2::Rational(_) => 3,
                        },
                        BezierSplitFragment2::AlgebraicEndpointImages { .. } => 4,
                        BezierSplitFragment2::AnalyticParallel(_) => 5,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_) => 6,
                        BezierSplitFragment2::Unresolved { .. } => 7,
                        BezierSplitFragment2::AlgebraicChord(_) => 8,
                        BezierSplitFragment2::SelectedFiber(_) => 9,
                    })
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
fn cusp_split_analytic_self_crossing_regularizes_under_both_policies() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let raw = self_crossing_cusp_split_parallel_region(&policy);
        let regularized = raw.regularized_region(&policy).unwrap().into_value();
        assert!(!regularized.is_empty());
        assert_eq!(regularized.boundary_loops().len(), 3);
    }
}
